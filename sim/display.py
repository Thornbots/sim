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
Whether an X window can open here.

Benches and sim.launch.py drop the gz and rviz windows where none can (the
Mac container has no display) and are watched in Foxglove instead. Native
macOS opens them on the screen through Cocoa, with no X server.
"""
import ctypes
import os
import sys


def display_error():
    """Return why no X window can open here, or None if one can."""
    if sys.platform == 'darwin':
        return None
    display = os.environ.get('DISPLAY')
    if not display:
        return 'DISPLAY is unset'
    try:
        x11 = ctypes.cdll.LoadLibrary('libX11.so.6')
    except OSError:
        return None  # can't check; gz and rviz2 will say
    x11.XOpenDisplay.restype = ctypes.c_void_p
    x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
    x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
    handle = x11.XOpenDisplay(display.encode())
    if not handle:
        return f'cannot open DISPLAY={display}'
    x11.XCloseDisplay(handle)
    return None
