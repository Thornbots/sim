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

"""Per-stack default for the MCB emulator's PTY link path."""
import os
import re


def default_mcb_link():
    """Return /tmp/mcb_emulator_pty-<GZ_PARTITION>-<ROS_DOMAIN_ID>.

    Stacks on different partitions/domains share /tmp, so a fixed path lets one
    stack steal another's UART. Pass `mcb_device_link` to override.
    """
    tag = '-'.join(re.sub(r'[^A-Za-z0-9_.]', '_', os.environ.get(name) or default)
                   for name, default in (('GZ_PARTITION', 'default'), ('ROS_DOMAIN_ID', '0')))
    return f'/tmp/mcb_emulator_pty-{tag}'
