"""
Relays gz sim GUI's fixed-name joint-slider topic
(/model/<model>/joint/<joint>/0/cmd_pos, not ROS-bridgeable) into the
custom topics headlink/headpitch's JointPositionController plugins
listen on (see sentry.urdf.xacro), so both the GUI slider and
/head_pan_cmd|/head_pitch_cmd can drive the same controller. Shells out
to `ign topic` (no gz-transport Python bindings here); reader/publisher
run on separate threads to avoid input lag -- see README.md for why.
"""
import re
import subprocess
import sys
import threading

RELAYS = [
    ('/model/sentry/joint/headlink/0/cmd_pos',
     '/model/sentry/joint/headlink/cmd_pos'),
    ('/model/sentry/joint/headpitch/0/cmd_pos',
     '/model/sentry/joint/headpitch/cmd_pos'),
]

DATA_LINE = re.compile(r'^\s*data:\s*(-?[0-9.eE+-]+)\s*$')


def relay_one(src_topic, dst_topic):
    lock = threading.Lock()
    latest = {'value': None}
    has_update = threading.Event()

    def read_slider():
        # stdbuf -oL: `ign topic -e` fully-buffers its stdout when it isn't
        # a TTY (writing to this pipe), so without forcing line buffering
        # here its "data: X" lines never actually reach us in real time --
        # they'd only show up once the OS pipe buffer happens to fill,
        # which for a human dragging a slider slowly could be effectively
        # never.
        echo = subprocess.Popen(
            ['stdbuf', '-oL', 'ign', 'topic', '-e', '-t', src_topic],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
        )
        for line in echo.stdout:
            match = DATA_LINE.match(line)
            if not match:
                continue
            with lock:
                latest['value'] = match.group(1)
            has_update.set()

    def publish_latest():
        while True:
            has_update.wait()
            has_update.clear()
            with lock:
                value = latest['value']
            subprocess.run(
                ['ign', 'topic', '-t', dst_topic, '-m', 'ignition.msgs.Double',
                 '-p', f'data: {value}'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )

    reader = threading.Thread(target=read_slider, daemon=True)
    publisher = threading.Thread(target=publish_latest, daemon=True)
    reader.start()
    publisher.start()
    reader.join()


def main(args=None):
    threads = [
        threading.Thread(target=relay_one, args=(src, dst), daemon=True)
        for src, dst in RELAYS
    ]
    for t in threads:
        t.start()
    try:
        for t in threads:
            t.join()
    except KeyboardInterrupt:
        sys.exit(0)


if __name__ == '__main__':
    main()
