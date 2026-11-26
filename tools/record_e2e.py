#!/usr/bin/env python3
# Copyright 2026 Thornbots
# Licensed under the Apache License, Version 2.0.

"""Record a sim suite's X11 desktop and render it at one sim second per second."""
import argparse
from bisect import bisect_right
from contextlib import ExitStack, contextmanager
import json
import math
import os
from pathlib import Path
import re
import select
import signal
import subprocess
from tempfile import TemporaryDirectory
from threading import Thread
import time

WIDTH, HEIGHT, BANNER = 1920, 1080, 86
LOCALIZATION = ('localization_drift', 'ekf')
VIEWS = {
    'localization_drift': ('rviz', 'gz'),
    'ekf': ('rviz', 'gz'),
    'aim': ('rviz',),
    'tracking': ('rviz',),
    'mcb_parked': ('rviz', 'gz'),
    'mcb_drive': ('rviz', 'gz'),
    'mcb_match': ('rviz', 'gz'),
}
EVENT = re.compile(r'\[\s*(RUN|OK|FAILED)\s*\]\s+(\S+)')
ANSI = re.compile(r'\x1b\[[0-9;]*m')
X11_START = re.compile(r'start: ([0-9.]+)')


def write_json(path, data):
    path.write_text(json.dumps(data, indent=2) + '\n')


def read_rows(path):
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def stop(process, sig=signal.SIGTERM):
    if process.poll() is None:
        process.send_signal(sig)
        try:
            process.wait(timeout=30)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def x11(args, env):
    return subprocess.run(args, env=env, capture_output=True, text=True, timeout=5)


@contextmanager
def desktop(requested, directory):
    with ExitStack() as stack:
        env = os.environ.copy()
        display = requested or env.get('DISPLAY')
        if display:
            env['DISPLAY'] = display
            if x11(['xdpyinfo'], env).returncode:
                if requested:
                    raise RuntimeError(f'Cannot access X11 display {requested}')
                display = None
        if not display:
            log = stack.enter_context((directory / 'desktop.log').open('w'))
            server = subprocess.Popen([
                'Xvfb', '-displayfd', '1', '-screen', '0', f'{WIDTH}x{HEIGHT}x24',
                '-nolisten', 'tcp'], stdout=subprocess.PIPE, stderr=log)
            stack.callback(stop, server)
            stack.callback(server.stdout.close)
            if not select.select([server.stdout], [], [], 10)[0]:
                raise RuntimeError('Xvfb did not start; see desktop.log')
            number = server.stdout.readline().decode().strip()
            if not number.isdecimal():
                raise RuntimeError('Xvfb did not allocate a display; see desktop.log')
            env['DISPLAY'] = ':' + number
            env.pop('XAUTHORITY', None)
            env.setdefault('LIBGL_ALWAYS_SOFTWARE', '1')
            manager = subprocess.Popen(['openbox'], env=env, stdout=log, stderr=log)
            stack.callback(stop, manager)
        details = x11(['xdpyinfo'], env)
        if not re.search(r'dimensions:\s+1920x1080\s', details.stdout):
            raise RuntimeError('Recording requires a 1920x1080 X11 display')
        deadline = time.monotonic() + 10
        while x11(['wmctrl', '-m'], env).returncode:
            if time.monotonic() >= deadline:
                raise RuntimeError('Recording requires an X11 window manager')
            time.sleep(.1)
        yield env


def arrange(env, views, original, placed):
    windows = x11(['wmctrl', '-l'], env).stdout.splitlines()
    current = set()
    for line in windows:
        window, _, _, *title_parts = line.split(maxsplit=3)
        if window in original:
            continue
        title = ''.join(title_parts).lower()
        view = 'rviz' if 'rviz' in title else 'gz' if re.search(r'gazebo|gz sim', title) else None
        if view not in views:
            continue
        if window not in placed:
            if views == ('rviz',):
                commands = [['xdotool', 'windowactivate', '--sync', window,
                             'key', '--window', window, 'F11']]
            else:
                x = views.index(view) * (WIDTH // 2)
                commands = [
                    ['wmctrl', '-ir', window, '-b', 'remove,maximized_vert,maximized_horz'],
                    ['wmctrl', '-ir', window, '-e', f'0,{x},35,{WIDTH // 2 - 4},{HEIGHT - 70}'],
                ]
            if any(x11(command, env).returncode for command in commands):
                continue
            placed.add(window)
        current.add(view)
    return current


def camera_request(config):
    """Format the launch camera preset for Gazebo's pose service."""
    x, y, z, roll, pitch, yaw = map(
        float, re.search(r'<camera_pose>(.*?)</camera_pose>', config)[1].split())
    cr, cp, cy = (math.cos(value / 2) for value in (roll, pitch, yaw))
    sr, sp, sy = (math.sin(value / 2) for value in (roll, pitch, yaw))
    return (f'pose: {{position: {{x: {x}, y: {y}, z: {z}}} orientation: {{'
            f'x: {sr*cp*cy-cr*sp*sy}, y: {cr*sp*cy+sr*cp*sy}, '
            f'z: {cr*cp*sy-sr*sp*cy}, w: {cr*cp*cy+sr*sp*sy}}}}}')


def copy_suite_log(source, log, events):
    with source:
        for line in source:
            log.write(line)
            clean = ANSI.sub('', line)
            if match := EVENT.search(clean):
                kind, cell = match.groups()
                events.write(json.dumps({'wall': time.time(), 'kind': kind, 'cell': cell}) + '\n')


@contextmanager
def clock_subscription(clock_log):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rosgraph_msgs.msg import Clock

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = rclpy.create_node('sim_video_clock')
        last = None

        def clock(message):
            nonlocal last
            wall = time.time()
            stamp = message.clock.sec + message.clock.nanosec * 1e-9
            if last and stamp >= last[1] and wall - last[0] < .01:
                return
            clock_log.write(json.dumps({'wall': wall, 'sim': stamp}) + '\n')
            last = (wall, stamp)

        node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data)
        try:
            yield rclpy, node
        finally:
            node.destroy_node()
    finally:
        rclpy.try_shutdown()


def finish_encoder(encoder):
    if encoder.poll() is None:
        try:
            encoder.communicate(b'q', timeout=30)
        except subprocess.TimeoutExpired:
            stop(encoder, signal.SIGKILL)
        except KeyboardInterrupt:
            stop(encoder, signal.SIGINT)
            raise


def wait_for_encoder(encoder, log):
    deadline = time.monotonic() + 10
    while encoder.poll() is None and time.monotonic() < deadline:
        if X11_START.search(log.read_text()):
            return
        time.sleep(.1)
    raise RuntimeError('X11 capture did not start; see capture.log')


def capture(args, launch_args):
    directory = Path(args.directory).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    if any(directory.iterdir()):
        raise RuntimeError('Use a fresh capture directory')
    telemetry = directory / 'telemetry'
    telemetry.mkdir()
    views = VIEWS[args.suite]
    rate = '1.0' if args.suite in ('aim', 'tracking') else '0.5'
    settings = {'real_time_factor': rate}
    settings.update(arg.split(':=', 1) for arg in launch_args)
    command = [str(Path(__file__).with_name('run_suite.sh')), args.suite,
               *(f'{key}:={value}' for key, value in settings.items()),
               'headless:=false', 'remote_dds:=false']
    if args.suite not in LOCALIZATION:
        command.append(f'log_dir:={telemetry}')
    metadata = {'suite': args.suite, 'capture_fps': args.capture_fps, 'command': command,
                'suite_exit': None, 'capture_ok': False, 'layout_wall': None}
    if args.suite.startswith('mcb_'):
        metadata['stack_checker_exit'] = None
    try:
        with ExitStack() as stack:
            env = stack.enter_context(desktop(args.display, directory))
            if not os.environ.get('ROS_DOMAIN_ID') or not os.environ.get('GZ_PARTITION'):
                raise RuntimeError('Set ROS_DOMAIN_ID and GZ_PARTITION for an idle sim session')
            from ament_index_python.packages import get_package_share_directory

            camera_pose = None
            if args.suite in LOCALIZATION:
                preset = Path(get_package_share_directory('sim')) / 'rviz/localization-gz.config'
                camera_pose = camera_request(preset.read_text())
                env.setdefault('GZ_IP', '127.0.0.1')
            os.environ.pop('ROS_AUTOMATIC_DISCOVERY_RANGE', None)
            os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(
                Path(get_package_share_directory('thornbots_pkg')) / 'config/fastdds_local.xml')
            clock_log = stack.enter_context((directory / 'clock.jsonl').open('w', buffering=1))
            rclpy, node = stack.enter_context(clock_subscription(clock_log))
            rclpy.spin_once(node, timeout_sec=1)
            active = [name for name in node.get_node_names()
                      if name != node.get_name() and not name.startswith('_ros2cli_daemon')]
            if active:
                raise RuntimeError(f'ROS domain already in use: {active}')
            video_log = stack.enter_context((directory / 'capture.log').open('w'))
            encoder = subprocess.Popen([
                'ffmpeg', '-y', '-f', 'x11grab', '-framerate', str(args.capture_fps),
                '-video_size', f'{WIDTH}x{HEIGHT}', '-draw_mouse', '0',
                '-use_wallclock_as_timestamps', '1', '-i', env['DISPLAY'], '-copyts',
                '-start_at_zero', '-an', '-c:v', 'libx264', '-threads', '2',
                '-preset', 'ultrafast', '-crf', '20', '-fps_mode', 'cfr',
                str(directory / 'wall-time.mp4')], env=env, stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL, stderr=video_log, start_new_session=True)
            stack.callback(finish_encoder, encoder)
            wait_for_encoder(encoder, directory / 'capture.log')
            log = stack.enter_context((directory / 'suite.log').open('w', buffering=1))
            events = stack.enter_context((directory / 'events.jsonl').open('w', buffering=1))
            original = {line.split()[0] for line in x11(['wmctrl', '-l'], env).stdout.splitlines()}
            launch = subprocess.Popen(command, env=env, stdin=subprocess.DEVNULL,
                                      stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                      text=True, errors='replace',
                                      start_new_session=True)
            reader = Thread(target=copy_suite_log, args=(launch.stdout, log, events))
            reader.start()
            stack.callback(reader.join)
            stack.callback(stop, launch, signal.SIGINT)
            placed = set()
            next_layout = time.monotonic()
            while launch.poll() is None:
                rclpy.spin_once(node, timeout_sec=.05)
                if encoder.poll() is not None:
                    raise RuntimeError('X11 capture failed; see capture.log')
                if time.monotonic() >= next_layout:
                    current = arrange(env, views, original, placed)
                    next_layout = time.monotonic() + .25
                    if current == set(views) and metadata['layout_wall'] is None:
                        if camera_pose:
                            result = subprocess.run(
                                ['gz', 'service', '-s', '/gui/move_to/pose',
                                 '--reqtype', 'gz.msgs.GUICamera', '--reptype', 'gz.msgs.Boolean',
                                 '--timeout', '1000', '--req', camera_pose],
                                env=env, capture_output=True, text=True, timeout=3)
                            if result.returncode or 'data: true' not in result.stdout:
                                continue
                        metadata['layout_wall'] = time.time()
            metadata['suite_exit'] = launch.returncode
        if 'stack_checker_exit' in metadata:
            with (directory / 'stack-checker.log').open('w') as log:
                checker = subprocess.run(
                    ['ros2', 'run', 'sim', 'check_bench_log', str(telemetry / 'stack.log')],
                    stdout=log, stderr=subprocess.STDOUT)
            metadata['stack_checker_exit'] = checker.returncode
        if encoder.returncode or metadata['layout_wall'] is None:
            raise RuntimeError('Incomplete X11 or /clock capture; see capture.log and suite.log')
        clock_axes(read_rows(directory / 'clock.jsonl'))
        metadata['capture_ok'] = True
    finally:
        write_json(directory / 'recording.json', metadata)
    if metadata['suite_exit'] != 0:
        raise RuntimeError(f'Suite or checker failed (exit {metadata["suite_exit"]}); see suite.log')
    if metadata.get('stack_checker_exit', 0) != 0:
        raise RuntimeError('MCB stack checker rejected the capture; see stack-checker.log')


def require_pass(metadata):
    if 'suite_exit' in metadata:
        checks = ['suite_exit']
        if metadata.get('suite', '').startswith('mcb_'):
            checks.append('stack_checker_exit')
        valid = metadata.get('capture_ok') is True
    else:
        checks = ['test_exit', 'checker_exit']
        valid = metadata.get('viewers') == ['gz', 'rviz'] and metadata.get('camera_set') is True
    passed = all(type(metadata.get(key)) is int and metadata[key] == 0 for key in checks)
    if not passed or not valid:
        raise RuntimeError('Refusing a failed suite/checker or incomplete capture; see the capture logs')


def clock_axes(rows):
    """Return increasing simulation and wall axes, collapsing a paused clock."""
    sim, wall = [], []
    previous_wall = -math.inf
    for row in rows:
        stamp, received = row['sim'], row['wall']
        if not math.isfinite(stamp) or not math.isfinite(received) or received <= previous_wall:
            raise RuntimeError('Invalid or non-increasing wall clock samples')
        if sim and stamp < sim[-1]:
            raise RuntimeError('Simulation clock moved backwards')
        previous_wall = received
        if sim and stamp == sim[-1]:
            wall[-1] = received
        else:
            sim.append(stamp)
            wall.append(received)
    if len(sim) < 2:
        raise RuntimeError('Insufficient simulation clock coverage')
    return sim, wall


def interpolate(value, x, y):
    i = bisect_right(x, value)
    if i == 0:
        return y[0]
    if i == len(x):
        return y[-1]
    fraction = (value - x[i - 1]) / (x[i] - x[i - 1])
    return y[i - 1] + fraction * (y[i] - y[i - 1])


def frame_plan(sim, wall, spans, epoch, capture_fps, fps, max_gap):
    """Map a single output frame grid over recorded spans to source frames."""
    boundaries = [0] + [i for i in range(1, len(sim)) if sim[i] - sim[i - 1] > max_gap]
    coverage = [(sim[a], sim[b - 1]) for a, b in zip(boundaries, boundaries[1:] + [len(sim)])]
    ranges = [(max(a, start), min(b, end))
              for a, b in coverage for start, end in spans
              if min(b, end) > max(a, start)]
    elapsed, frame = 0., 0
    plan = []
    for begin, end in ranges:
        next_elapsed = elapsed + end - begin
        stop_at = math.ceil(next_elapsed * fps - 1e-7)
        while frame < stop_at:
            stamp = begin + max(0., frame / fps - elapsed)
            index = round((interpolate(stamp, sim, wall) - epoch) * capture_fps)
            if index < 0 or (plan and index < plan[-1][1]):
                raise RuntimeError('Simulation clock and video timestamps disagree')
            plan.append((stamp, index))
            frame += 1
        elapsed = next_elapsed
    if not plan:
        raise RuntimeError('Requested simulation window has no recorded clock coverage')
    return ranges, plan


def scored_content(directory, suite, wall, sim):
    telemetry = directory / 'telemetry'
    if not telemetry.is_dir():
        telemetry = directory
    spans, labels, started = [], [], {}
    if suite.startswith('mcb_'):
        name = 'poses.jsonl' if suite == 'mcb_parked' else 'route.jsonl'
        groups = {}
        for row in sorted(read_rows(telemetry / name), key=lambda row: row['t']):
            cell = row.get('cell') if suite == 'mcb_parked' else row.get('segment')
            key = cell if suite == 'mcb_parked' else suite
            groups.setdefault(key, []).append(row['t'])
            if cell and (not labels or labels[-1][1] != cell):
                labels.append((row['t'], cell))
        spans = sorted((times[0], times[-1]) for times in groups.values())
    else:
        for row in read_rows(directory / 'events.jsonl'):
            stamp = interpolate(row['wall'], wall, sim)
            cell = row['cell']
            if row['kind'] == 'RUN':
                started[cell] = stamp
            elif row['kind'] == 'OK' and cell in started:
                start = started.pop(cell)
                if start < stamp:
                    spans.append((start, stamp))
                    labels.append((start, cell.split('/', 2)[-1]))
    shots = []
    for row in read_rows(telemetry / 'shots.jsonl'):
        if row.get('kind', 'mcb') != 'mcb':
            continue
        stamp = row.get('scored_at', row.get('impact_t', row.get('t_fire', row.get('t'))))
        if stamp is not None:
            shots.append((stamp, bool(row.get('enemy_hit', row.get('hit')))))
    return spans, labels, sorted(shots)


def encode_video(source, output, plan, labels, shots, fps, capture_fps, suite, gz_zoom):
    from PIL import Image, ImageDraw, ImageFont

    font = ImageFont.load_default(size=28)
    small = ImageFont.load_default(size=22)
    label_times = [stamp for stamp, _ in labels]
    shot_times = [stamp for stamp, _ in shots]
    hit_counts = [0]
    for _, hit in shots:
        hit_counts.append(hit_counts[-1] + hit)
    first = plan[0][1]
    seek = max(0., first / capture_fps - .000001)
    with ExitStack() as stack:
        decoder = stack.enter_context(subprocess.Popen([
            'ffmpeg', '-v', 'error', '-threads', '2', '-ss', f'{seek:.6f}', '-i', str(source), '-an',
            '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-threads', '1', '-'], stdout=subprocess.PIPE))
        stack.callback(stop, decoder, signal.SIGKILL)
        encoder = stack.enter_context(subprocess.Popen([
            'ffmpeg', '-v', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
            '-s', f'{WIDTH}x{HEIGHT}', '-r', str(fps), '-i', '-', '-an',
            '-c:v', 'libx264', '-threads', '2', '-preset', 'veryfast', '-crf', '20',
            '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(output)], stdin=subprocess.PIPE))
        stack.callback(stop, encoder)
        current = first - 1
        size = WIDTH * HEIGHT * 3
        frame = Image.new('RGB', (WIDTH, HEIGHT), (16, 20, 30))
        draw = ImageDraw.Draw(frame)
        for stamp, index in plan:
            while current < index:
                raw = decoder.stdout.read(size)
                if len(raw) != size:
                    raise RuntimeError(f'Capture ended before sim {stamp:.2f}s')
                current += 1
            count = bisect_right(shot_times, stamp)
            label = bisect_right(label_times, stamp)
            title = f'{suite}   |   sim {stamp:.1f}s'
            if label:
                title += '   |   ' + labels[label - 1][1]
            details = '1x sim time'
            if shots:
                hit_label = 'referee credits/shots' if suite == 'mcb_match' else 'hits/shots'
                details += f'   |   {hit_label} {hit_counts[count]}/{count}'
            desktop_frame = Image.frombytes('RGB', (WIDTH, HEIGHT), raw)
            if gz_zoom > 1:
                x, y = WIDTH * 3 // 4, HEIGHT // 2
                dx, dy = round(WIDTH / (4 * gz_zoom)), round(HEIGHT / (2 * gz_zoom))
                gz = desktop_frame.crop((x - dx, y - dy, x + dx, y + dy))
                desktop_frame.paste(gz.resize((WIDTH // 2, HEIGHT), Image.Resampling.BILINEAR),
                                    (WIDTH // 2, 0))
            desktop_frame.thumbnail((WIDTH, HEIGHT - BANNER), Image.Resampling.BILINEAR)
            frame.paste(desktop_frame, ((WIDTH - desktop_frame.width) // 2, BANNER))
            draw.rectangle((0, 0, WIDTH, BANNER - 1), fill=(16, 20, 30))
            draw.text((24, 9), title, font=font, fill=(245, 247, 250))
            draw.text((24, 49), details, font=small, fill=(180, 195, 215))
            draw.text((WIDTH - 24, 49), 'PASS', anchor='rt', font=small, fill=(120, 230, 140))
            encoder.stdin.write(frame.tobytes())
        encoder.stdin.close()
        if encoder.wait() != 0:
            raise RuntimeError('H.264 encoding failed')


def render(args):
    directory = Path(args.directory).resolve()
    metadata = json.loads((directory / 'recording.json').read_text())
    require_pass(metadata)
    suite = metadata.get('suite', metadata.get('stage'))
    sim, wall = clock_axes(read_rows(directory / 'clock.jsonl'))
    match = X11_START.search((directory / 'capture.log').read_text())
    if not match or float(match[1]) < 1e9 or float(match[1]) > wall[0] + 1:
        raise RuntimeError('X11 input start timestamp disagrees with /clock wall times')
    start, end = args.start, args.end
    if ((start is not None and start < sim[0]) or (end is not None and end > sim[-1])):
        raise RuntimeError('Requested simulation window is outside recorded /clock')
    spans, labels, shots = scored_content(directory, suite, wall, sim)
    if metadata.get('layout_wall') is not None:
        first_view = interpolate(metadata['layout_wall'], wall, sim)
    else:
        first_view = sim[0]
    if start is not None or end is not None:
        spans = [(start if start is not None else first_view, end if end is not None else sim[-1])]
    else:
        spans = [(max(first_view, a), b) for a, b in (spans or [(sim[0], sim[-1])])]
    ranges, plan = frame_plan(sim, wall, spans, float(match[1]), metadata['capture_fps'],
                              args.fps, args.max_gap)
    output = Path(args.output).resolve()
    if output.suffix != '.mp4' or output in {directory / 'wall-time.mp4', directory / 'recording.mp4'}:
        raise RuntimeError('Choose a separate .mp4 output path')
    output.parent.mkdir(parents=True, exist_ok=True)
    gz_zoom = args.gz_zoom if suite.startswith('mcb_') else 1.
    with TemporaryDirectory(dir=output.parent) as temporary:
        video = Path(temporary) / 'render.mp4'
        encode_video(directory / 'wall-time.mp4', video, plan, labels, shots, args.fps,
                     metadata['capture_fps'], suite, gz_zoom)
        if args.pr_clip:
            clip = Path(temporary) / 'pr.mp4'
            subprocess.run([
                'ffmpeg', '-v', 'error', '-i', str(video), '-t', '30', '-vf', 'scale=960:540',
                '-an', '-c:v', 'libx264', '-threads', '2', '-b:v', '1800k', '-maxrate', '2000k',
                '-bufsize', '2000k', '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
                str(clip)], check=True)
            if clip.stat().st_size >= 10_000_000:
                raise RuntimeError('PR clip exceeds the 10 MB attachment limit')
            clip.replace(output.with_name(output.stem + '-pr.mp4'))
        video.replace(output)
    write_json(output.with_suffix('.json'), {
        **metadata, 'verdict': 'PASS', 'fps': args.fps, 'frames': len(plan), 'sim_ranges': ranges,
        'gz_zoom': gz_zoom,
    })


def positive_float(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError('must be finite and positive')
    return result


def positive_int(value):
    result = int(value)
    if result <= 0:
        raise argparse.ArgumentTypeError('must be positive')
    return result


def interrupted(_signum, _frame):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    take = commands.add_parser('capture', help='run a suite and record its desktop and verdict')
    take.add_argument('suite', choices=VIEWS)
    take.add_argument('directory')
    take.add_argument('--display', help='use a specific existing X11 display')
    take.add_argument('--capture-fps', type=positive_int, default=30)
    take.add_argument('--backend', choices=['hosted', 'renode'], help='MCB backend')
    take.add_argument('--speeds', help='pass speeds:= to the suite')
    take.add_argument('--paths', help='pass paths:= to the suite')
    take.add_argument('--duration', type=positive_float, help='pass duration:= to the suite')
    movie = commands.add_parser('render', help='turn an accepted capture into a 1x MP4')
    movie.add_argument('directory')
    movie.add_argument('output')
    movie.add_argument('--fps', type=positive_int, default=30)
    movie.add_argument('--start', type=float, help='absolute sim start in seconds')
    movie.add_argument('--end', type=float, help='absolute sim end in seconds')
    movie.add_argument('--max-gap', type=positive_float, default=.5,
                       help='skip clock jumps larger than this number of sim seconds')
    movie.add_argument('--gz-zoom', type=positive_float, default=1.,
                       help='crop the Gazebo pane in MCB recordings (1 to 4)')
    movie.add_argument('--pr-clip', action='store_true', help='also write a short MP4 under 10 MB')
    args, extra = parser.parse_known_args()
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGHUP, interrupted)
    try:
        if args.command == 'capture':
            if extra[:1] == ['--']:
                extra = extra[1:]
            if any(not re.fullmatch(r'[A-Za-z_][A-Za-z_0-9]*:=.*', arg) for arg in extra):
                parser.error('suite arguments must have the form name:=value')
            protected = {'headless', 'log_dir', 'remote_dds', 'stage', 'suite'}
            if any(arg.split(':=', 1)[0] in protected for arg in extra):
                parser.error('the recorder sets headless, log_dir, remote_dds, stage and suite')
            legacy = (f'{key}:={value}' for key in ('backend', 'speeds', 'paths', 'duration')
                      if (value := getattr(args, key)) is not None)
            capture(args, [*legacy, *extra])
        else:
            if extra:
                parser.error('unrecognized arguments: ' + ' '.join(extra))
            if not 1 <= args.gz_zoom <= 4:
                parser.error('--gz-zoom must be between 1 and 4')
            bounds = [bound for bound in (args.start, args.end) if bound is not None]
            if not all(math.isfinite(bound) for bound in bounds) or (
                    len(bounds) == 2 and bounds[0] >= bounds[1]):
                parser.error('--start and --end must be finite and in increasing order')
            render(args)
    except (RuntimeError, OSError, subprocess.SubprocessError) as error:
        parser.exit(1, f'{error}\n')
    except KeyboardInterrupt:
        parser.exit(130, 'Recording interrupted\n')


if __name__ == '__main__':
    main()
