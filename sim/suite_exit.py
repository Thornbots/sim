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

"""Shut down completed suites and return a failed launch for failed native test runs."""
import os

from ament_index_python.packages import get_package_prefix

from launch.actions import EmitEvent, LogInfo
from launch.events import Shutdown


def suite_command(executable):
    """
    Return the argv prefix running a sim suite executable directly.

    `ros2 run` is a Python wrapper that ignores a lone SIGINT, so launch could not
    stop a suite started through it without a group-wide signal.
    """
    return [os.path.join(get_package_prefix('sim'), 'lib', 'sim', executable)]


def finish_suite(event, context, name):
    """Keep child suite failures visible to ros2 launch and shell callers."""
    if event.returncode:
        raise RuntimeError(f'{name} tests failed with code {event.returncode}')
    return [LogInfo(msg=f'{name} tests exited with code 0'),
            EmitEvent(event=Shutdown(reason=f'{name} tests finished'))]
