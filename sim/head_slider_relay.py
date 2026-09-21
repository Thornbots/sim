# Copyright 2026 Thornbots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Relays gz sim GUI's fixed-name joint-slider topic into the head's custom controller topics.

The GUI topic is /model/<model>/joint/<joint>/0/cmd_pos, not
ROS-bridgeable; the custom topics are declared in sentry.urdf.xacro, so
both the GUI slider and /head_pan_cmd|/head_pitch_cmd can drive the
same JointPositionController plugin on headlink/headpitch. Shells out
to `ign topic` (no gz-transport Python bindings here); reader and
publisher run on separate threads to avoid input lag -- see README.md
for why.
"""
import ctypes
import re
import signal
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

PR_SET_PDEATHSIG = 1
_prctl = ctypes.CDLL(None, use_errno=True).prctl


def _die_with_parent():
    # `ign` execs into ign-transport-topic, which keeps this setting. Without
    # it, launch's SIGINT to this process alone orphaned the echo forever.
    _prctl(PR_SET_PDEATHSIG, int(signal.SIGTERM))


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
            preexec_fn=_die_with_parent,
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
