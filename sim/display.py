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

"""Python launch API adapter for the native display check."""
import os
import subprocess

from ament_index_python.packages import get_package_prefix


def display_error():
    """Return the native check's reason, or None when windows can open."""
    executable = os.path.join(get_package_prefix('sim'), 'lib', 'sim', 'display_probe')
    result = subprocess.run([executable], check=True, capture_output=True, text=True)
    return result.stdout.strip() or None
