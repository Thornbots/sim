"""
Relays gz sim GUI's "Joint Position Controller" panel slider commands into
the ROS-bridgeable topics headlink/headpitch's JointPositionController
system plugins actually listen on (see sentry.urdf.xacro).

Why this exists: the GUI slider panel always publishes to gz-transport's
own auto-generated default topic for a joint,
  /model/<model>/joint/<joint>/<axis>/cmd_pos
(axis is always 0 for these single-DOF joints) -- this is not configurable
from the GUI side. sentry.urdf.xacro's plugins instead listen on a custom
topic without the axis segment (/model/sentry/joint/headlink/cmd_pos etc.)
specifically so sim.launch.py's ros_gz_bridge Nodes can remap them to clean
ROS topics (/head_pan_cmd, /head_pitch_cmd, used by e.g. head_sweep.py) --
ROS2 topic names can't have a namespace token starting with a digit, so
the GUI's own default topic can never be bridged into ROS directly
(confirmed: ros_gz_bridge's parameter_bridge raises
InvalidTopicNameError/RCLInvalidROSArgsError on '.../0/cmd_pos' whether or
not it's used as a remap target). Since gz-sim's JointPositionController
only accepts one <topic> per instance, both control paths can't target the
plugin directly at once either -- this script is the bridge between them,
letting the GUI slider and /head_pan_cmd|/head_pitch_cmd both drive the
same controller instance.

No gz-transport Python bindings are installed in this image (checked:
no `ignition.transport`/`gz.transport*` module), so this shells out to the
`ign topic` CLI (ignition-transport11-cli) for both the subscribe side
(`ign topic -e`, kept running as a long-lived subprocess) and the publish
side (`ign topic -p`, invoked fresh per relayed message -- each call pays
gz-transport's discovery overhead, so this is fine for a human dragging a
slider but not a low-latency control path).
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
    # stdbuf -oL: `ign topic -e` fully-buffers its stdout when it isn't a
    # TTY (writing to this pipe), so without forcing line buffering here
    # its "data: X" lines never actually reach us in real time -- they'd
    # only show up once the OS pipe buffer happens to fill, which for a
    # human dragging a slider slowly could be effectively never.
    echo = subprocess.Popen(
        ['stdbuf', '-oL', 'ign', 'topic', '-e', '-t', src_topic],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
    )
    for line in echo.stdout:
        match = DATA_LINE.match(line)
        if not match:
            continue
        value = match.group(1)
        subprocess.run(
            ['ign', 'topic', '-t', dst_topic, '-m', 'ignition.msgs.Double',
             '-p', f'data: {value}'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )


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
