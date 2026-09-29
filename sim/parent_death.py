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
Signal a child process when the process that started it dies.

Linux's PR_SET_PDEATHSIG. macOS has no equivalent, so there the child
outlives a parent killed hard; a clean exit still stops it.
"""
import ctypes
import sys

PR_SET_PDEATHSIG = 1
_prctl = ctypes.CDLL(None, use_errno=True).prctl if sys.platform.startswith('linux') else None


def die_with_parent(sig):
    """Return a Popen preexec_fn that sends sig to the child when its parent dies."""
    if _prctl is None:
        return None
    return lambda: _prctl(PR_SET_PDEATHSIG, int(sig))
