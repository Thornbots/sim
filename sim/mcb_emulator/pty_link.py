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
The emulator's end of the UART: a pty whose slave is linked at a fixed path.

The slave goes raw at once, so the line discipline never rewrites a byte
before the bridge opens it, and stays open here so the pty outlives a
bridge restart. Writes nobody reads are dropped, as on a UART.
"""
import errno
import os
import tty

_AGAIN = (errno.EAGAIN, errno.EWOULDBLOCK, errno.EIO)


class PtyLink:

    def __init__(self, link):
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        os.set_blocking(self.master, False)
        self.link = link
        if os.path.lexists(link):
            os.unlink(link)
        os.symlink(os.ttyname(self.slave), link)
        self.dropped = 0

    def read(self):
        data = bytearray()
        while True:
            try:
                chunk = os.read(self.master, 4096)
            except OSError as e:
                if e.errno in _AGAIN:
                    break
                raise
            if not chunk:
                break
            data += chunk
        return bytes(data)

    def write(self, data):
        try:
            sent = os.write(self.master, data)
        except OSError as e:
            if e.errno not in _AGAIN:
                raise
            sent = 0
        self.dropped += len(data) - sent

    def close(self):
        if os.path.islink(self.link):
            os.unlink(self.link)
        os.close(self.master)
        os.close(self.slave)
