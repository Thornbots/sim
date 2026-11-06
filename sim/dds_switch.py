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
Remote-DDS and Foxglove switches for the sim launches; both default off.

THORNBOTS_REMOTE_DDS / `remote_dds:=` and THORNBOTS_FOXGLOVE / `foxglove:=`, see
README.md "Remote DDS and Foxglove". Off means no off-machine DDS: the range is
LOCALHOST and no profile lists initial peers (a peers profile defeats the range).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument, SetEnvironmentVariable, UnsetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression

REMOTE_DDS_DEFAULT = EnvironmentVariable('THORNBOTS_REMOTE_DDS', default_value='false')
FOXGLOVE_DEFAULT = EnvironmentVariable('THORNBOTS_FOXGLOVE', default_value='false')
# Declared or not, an argument given on the command line or the environment wins.
_REMOTE = ["'", LaunchConfiguration('remote_dds', default=REMOTE_DDS_DEFAULT),
           "'.lower() in ('1', 'true')"]


def remote_dds_argument():
    return DeclareLaunchArgument(
        'remote_dds', default_value=REMOTE_DDS_DEFAULT,
        description='true/false, default $THORNBOTS_REMOTE_DDS or false. Off keeps DDS on '
        "this machine; on restores the container profile with the robots' Tailscale peers")


def dds_environment():
    """Process environment for everything this launch starts (list of launch actions)."""
    on, off = IfCondition(PythonExpression(_REMOTE)), UnlessCondition(PythonExpression(_REMOTE))
    return [
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST', condition=off),
        UnsetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', condition=off),
        UnsetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', condition=on),
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            EnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE',
                                default_value='/etc/fastdds/profile.xml'),
            condition=on),
    ]


def udp_profile():
    """Return thornbots_pkg's UDP-only profile, with Tailscale peers only if remote."""
    base = os.path.join(get_package_share_directory('thornbots_pkg'), 'config',
                        'fastdds_udp_only')
    return PythonExpression(["'", base, "' + ('' if ", *_REMOTE, " else '_local') + '.xml'"])


def udp_env():
    """
    additional_env for a node pinned to UDP-only (no SHM).

    SYSTEM_DEFAULT, not the environment's LOCALHOST, which would add SHM back.
    """
    return {'FASTRTPS_DEFAULT_PROFILES_FILE': udp_profile(),
            'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SYSTEM_DEFAULT'}
