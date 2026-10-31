#!/usr/bin/env python3
# Copyright 2026 Thornbots
# Licensed under the Apache License, Version 2.0.

"""Capture a shared X11 desktop and resample it using recorded ROS /clock."""
import argparse
import bisect
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time


def capture(args):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock

    os.environ['DISPLAY'] = args.display
    directory = Path(args.directory)
    if directory.exists() and any(directory.iterdir()):
        raise RuntimeError('Use a fresh recording directory')
    directory.mkdir(parents=True, exist_ok=True)
    clock_log = (directory / 'clock.jsonl').open('w')
    video_log = (directory / 'capture.log').open('w')
    rclpy.init()
    node = rclpy.create_node('e2e_video_clock')
    until = time.monotonic() + 2
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=.1)
    active = [name for name in node.get_node_names()
              if name != 'e2e_video_clock' and not name.startswith('_ros2cli_daemon')]
    if active:
        node.destroy_node()
        rclpy.shutdown()
        clock_log.close()
        video_log.close()
        raise RuntimeError(f'ROS domain is already in use: {active}')
    last = [0.0]

    def clock(message):
        wall = time.time()
        if wall - last[0] < .02:
            return
        last[0] = wall
        clock_log.write(json.dumps({'wall': wall, 'sim': message.clock.sec +
                                   message.clock.nanosec * 1e-9}) + '\n')
        clock_log.flush()

    node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data)
    encoder = subprocess.Popen([
        'ffmpeg', '-y', '-f', 'x11grab', '-framerate', str(args.capture_fps),
        '-video_size', '1920x1080', '-draw_mouse', '0', '-use_wallclock_as_timestamps', '1',
        '-i', args.display, '-copyts', '-start_at_zero', '-an', '-c:v', 'libx264',
        '-threads', '1', '-preset', 'ultrafast', '-crf', '18',
        '-fps_mode', 'cfr', '-r', str(args.capture_fps),
        str(directory / 'wall-time.mp4')], stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL, stderr=video_log)
    command = ['ros2', 'launch', 'sim', 'e2e.launch.py', 'stage:=' + args.stage,
               'backend:=' + args.backend, 'real_time_factor:=0.25', 'foxglove:=false',
               'log_dir:=' + str(directory)]
    command += ['chase_settle_s:=' + str(args.chase_settle_s)]
    if args.stage == 'mcb_parked':
        command += ['speeds:=' + args.speeds, 'paths:=' + args.paths,
                    'duration:=' + str(args.duration)]
    launch_log = (directory / 'launch.log').open('w')
    launch = subprocess.Popen(command, stdout=launch_log, stderr=subprocess.STDOUT,
                              start_new_session=True)
    preset = 'e2e-parked-gz.config' if args.stage == 'mcb_parked' else 'e2e-gz.config'
    config = (Path(__file__).resolve().parents[1] / 'rviz' / preset).read_text()
    xyzrpy = [float(v) for v in re.search(r'<camera_pose>(.*?)</camera_pose>', config)[1].split()]
    x, y, z, roll, pitch, yaw = xyzrpy
    cr, cp, cy = [math.cos(v / 2) for v in (roll, pitch, yaw)]
    sr, sp, sy = [math.sin(v / 2) for v in (roll, pitch, yaw)]
    pose = (f'position: {{x: {x}, y: {y}, z: {z}}} orientation: {{'
            f'x: {sr*cp*cy-cr*sp*sy}, y: {cr*sp*cy+sr*cp*sy}, '
            f'z: {cr*cp*sy-sr*sp*cy}, w: {cr*cp*cy+sr*sp*sy}}}')
    camera_set = False
    placed = set()
    seen_apps = set()
    last_layout = 0.0
    try:
        while launch.poll() is None:
            rclpy.spin_once(node, timeout_sec=.05)
            if encoder.poll() is not None:
                raise RuntimeError('X11 recording failed; see capture.log')
            if time.time() - last[0] < 1 and time.time() - last_layout > 1:
                last_layout = time.time()
                windows = subprocess.run(['wmctrl', '-l'], capture_output=True,
                                         text=True, check=False).stdout
                for line in windows.splitlines():
                    window = line.split()[0]
                    if window in placed:
                        continue
                    x = 0 if 'rviz' in line.lower() else 960
                    if 'rviz' not in line.lower() and 'gazebo' not in line.lower():
                        continue
                    subprocess.run(['wmctrl', '-ir', window, '-b',
                                    'remove,maximized_vert,maximized_horz'], check=True)
                    subprocess.run(['wmctrl', '-ir', window, '-e',
                                    f'0,{x},86,960,986'], check=True)
                    placed.add(window)
                    seen_apps.add('rviz' if 'rviz' in line.lower() else 'gz')
                if 'gz' in seen_apps and not camera_set:
                    result = subprocess.run([
                        'gz', 'service', '-s', '/gui/move_to/pose', '--reqtype', 'gz.msgs.GUICamera',
                        '--reptype', 'gz.msgs.Boolean', '--timeout', '1000', '--req', 'pose: {' + pose + '}'],
                        capture_output=True, text=True, timeout=3, check=False)
                    camera_set = result.returncode == 0 and 'true' in result.stdout
    finally:
        if launch.poll() is None:
            os.killpg(launch.pid, signal.SIGINT)
            try:
                launch.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
                launch.wait()
        if encoder.poll() is None:
            encoder.communicate(b'q', timeout=30)
        clock_log.close()
        video_log.close()
        launch_log.close()
        node.destroy_node()
        rclpy.shutdown()
    checker = subprocess.run(['ros2', 'run', 'sim', 'check_bench_log',
                              str(directory / 'launch.log'), str(directory / 'stack.log')],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    (directory / 'checker.log').write_text(checker.stdout)
    (directory / 'recording.json').write_text(json.dumps({
        'stage': args.stage, 'backend': args.backend, 'capture_fps': args.capture_fps,
        'test_exit': launch.returncode, 'checker_exit': checker.returncode,
        'viewers': sorted(seen_apps), 'camera_set': camera_set,
        'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'),
        'gz_partition': os.environ.get('GZ_PARTITION')}, indent=2) + '\n')
    if seen_apps != {'rviz', 'gz'} or not camera_set:
        raise RuntimeError('Both RViz and gz windows were not observed')
    if launch.returncode or checker.returncode:
        raise RuntimeError('E2E test failed; see launch.log (capture retained)')


def render(args):
    import numpy as np
    from PIL import Image, ImageDraw, ImageFont

    directory = Path(args.directory)
    metadata = json.loads((directory / 'recording.json').read_text())
    if (metadata['test_exit'] or metadata['checker_exit'] or
            metadata['viewers'] != ['gz', 'rviz'] or not metadata.get('camera_set')):
        raise RuntimeError('Refusing a failed test/checker capture')
    rows = [json.loads(line) for line in (directory / 'clock.jsonl').read_text().splitlines()]
    clocks = []
    for row in rows:
        if not clocks or row['sim'] > clocks[-1]['sim']:
            clocks.append(row)
    shots = [json.loads(line) for line in (directory / 'shots.jsonl').read_text().splitlines()]
    shots = sorted([s for s in shots if s.get('kind') == 'mcb'], key=lambda s: s.get('scored_at', s.get('impact_t', s['t'])))
    pose_file = 'poses.jsonl' if metadata['stage'] == 'mcb_parked' else 'route.jsonl'
    poses = [json.loads(line) for line in (directory / pose_file).read_text().splitlines()]
    poses.sort(key=lambda p: p['t'])
    begin = args.start if args.start is not None else min(p['t'] for p in poses)
    end = args.end if args.end is not None else max(p['t'] for p in poses)
    log = (directory / 'capture.log').read_text()
    match = re.search(r'start: ([0-9.]+)', log)
    if not match:
        raise RuntimeError('No X11 input start timestamp in capture.log')
    epoch = float(match[1])
    if epoch < 1e9 or abs(epoch - rows[0]['wall']) > 600:
        raise RuntimeError('X11 timestamps are not wall-clock epochs')
    if begin < clocks[0]['sim'] or end > clocks[-1]['sim']:
        raise RuntimeError('Requested sim window is outside recorded /clock')
    sim_times = [row['sim'] for row in clocks]
    ranges = []
    range_start = poses[0]['t']
    previous = range_start
    for pose_row in poses[1:]:
        current_time = pose_row['t']
        if current_time - previous > .5:
            ranges.append((max(begin, range_start), min(end, previous)))
            range_start = current_time
        previous = current_time
    ranges.append((max(begin, range_start), min(end, previous)))
    ranges = [(a, b) for a, b in ranges if b > a]
    output_times = np.concatenate([np.arange(a, b, 1 / args.fps) for a, b in ranges])
    walls = np.interp(output_times, sim_times, [row['wall'] for row in clocks])
    indices = np.maximum(0, np.rint((walls - epoch) * metadata['capture_fps']).astype(int))
    decoder = subprocess.Popen(['ffmpeg', '-v', 'error', '-threads', '1', '-i', str(directory / 'wall-time.mp4'),
                                '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-'], stdout=subprocess.PIPE)
    output = Path(args.output)
    gz_width, gz_height = 960 / args.gz_zoom, 945 / args.gz_zoom
    gz_crop = (int(1440 - gz_width / 2), int(607.5 - gz_height / 2),
               int(1440 + gz_width / 2), int(607.5 + gz_height / 2))
    encoder = subprocess.Popen(['ffmpeg', '-y', '-v', 'error', '-f', 'rawvideo',
                                '-pix_fmt', 'rgb24', '-s', '1920x1080', '-r', str(args.fps),
                                '-i', '-', '-an', '-c:v', 'libx264', '-threads', '1',
                                '-preset', 'fast', '-crf', '18', '-pix_fmt', 'yuv420p',
                                '-movflags', '+faststart', str(output)], stdin=subprocess.PIPE)
    font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 28)
    small = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 22)
    pose_times = [p['t'] for p in poses]
    shot_times = [s.get('scored_at', s.get('impact_t', s['t'])) for s in shots]
    current, raw, frame_size = -1, None, 1920 * 1080 * 3
    try:
        for sim_t, wanted in zip(output_times, indices):
            while current < wanted:
                raw = decoder.stdout.read(frame_size)
                if len(raw) != frame_size:
                    raise RuntimeError('Capture ended before requested sim window')
                current += 1
            count = bisect.bisect_right(shot_times, sim_t)
            hits = sum(bool(s.get('enemy_hit', s.get('hit', False))) for s in shots[:count])
            cell = ''
            if metadata['stage'] == 'mcb_parked':
                pose = poses[max(0, bisect.bisect_right(pose_times, sim_t)-1)]
                cell = f' | target {pose.get("path", "")} {pose.get("speed", 0):g}m/s spin {pose.get("spin_hz", 0):g}Hz'
            banner = Image.new('RGB', (1920, 86), (16, 20, 30))
            draw = ImageDraw.Draw(banner)
            draw.text((24, 8), f'{metadata["stage"]}{cell}  |  sim {sim_t:.1f}s  |  MCB hits/shots {hits}/{count}',
                      font=font, fill=(245, 247, 250))
            backend = 'Renode board ELF' if metadata['backend'] == 'renode' else 'Hosted MCBV3'
            draw.text((24, 48), f'{backend} | truth detections | 1x sim time | RViz left / gz right | cyan aim / green hit / red miss',
                      font=small, fill=(180, 195, 215))
            desktop = Image.frombytes('RGB', (1920, 1080), raw)
            frame = Image.new('RGB', (1920, 1080))
            frame.paste(banner, (0, 0))
            frame.paste(desktop.crop((16, 166, 944, 1062)).resize(
                (960, 994), Image.Resampling.LANCZOS), (0, 86))
            frame.paste(desktop.crop(gz_crop).resize(
                (960, 994), Image.Resampling.LANCZOS), (960, 86))
            encoder.stdin.write(frame.tobytes())
    finally:
        encoder.stdin.close()
        encoder.wait()
        decoder.kill()
        decoder.stdout.close()
        decoder.wait()
    if encoder.returncode:
        raise RuntimeError('H.264 encoding failed')
    output.with_suffix('.json').write_text(json.dumps({
        **metadata, 'sim_start': begin, 'sim_end': end, 'sim_ranges': ranges, 'fps': args.fps,
        'resolution': [1920, 1080], 'gz_zoom': args.gz_zoom, 'retiming': 'ROS /clock to X11 wall timestamps; 1x sim time',
        'hits': sum(bool(s.get('enemy_hit', s.get('hit', False))) for s in shots if begin <= s.get('scored_at', s.get('impact_t', s['t'])) < end),
        'shots': sum(begin <= s.get('scored_at', s.get('impact_t', s['t'])) < end for s in shots)}, indent=2) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    capture_parser = commands.add_parser('capture')
    capture_parser.add_argument('stage', choices=['mcb_parked', 'mcb_drive', 'mcb_match'])
    capture_parser.add_argument('directory')
    capture_parser.add_argument('--backend', choices=['hosted', 'renode'], default='hosted')
    capture_parser.add_argument('--display', default=os.environ.get('DISPLAY', ':99'))
    capture_parser.add_argument('--capture-fps', type=int, default=20)
    capture_parser.add_argument('--speeds', default='0 1')
    capture_parser.add_argument('--paths', default='lateral')
    capture_parser.add_argument('--duration', type=float, default=35)
    capture_parser.add_argument('--chase-settle-s', type=float, default=.025)
    render_parser = commands.add_parser('render')
    render_parser.add_argument('directory')
    render_parser.add_argument('output')
    render_parser.add_argument('--fps', type=int, default=30)
    render_parser.add_argument('--gz-zoom', type=float, default=1.4,
                               help='Crop around the gz pane centre; 1 keeps the full arena')
    render_parser.add_argument('--start', type=float)
    render_parser.add_argument('--end', type=float)
    args = parser.parse_args()
    if args.command == 'render' and not 1 <= args.gz_zoom <= 4:
        parser.error('--gz-zoom must be between 1 and 4')
    (capture if args.command == 'capture' else render)(args)


if __name__ == '__main__':
    main()
