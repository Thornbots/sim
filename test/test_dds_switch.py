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

"""The remote-DDS and Foxglove switches default off and honour the environment."""
import os

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

from sim.dds_switch import (
    dds_environment, FOXGLOVE_DEFAULT, remote_dds_argument, udp_env, udp_profile)


def _perform(context, value):
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


def _resolve(environment, arguments=None):
    saved = dict(os.environ)
    os.environ.clear()
    os.environ.update(environment, AMENT_PREFIX_PATH=saved['AMENT_PREFIX_PATH'])
    try:
        context = LaunchContext()
        context.launch_configurations.update(arguments or {})
        remote_dds_argument().visit(context)
        for action in dds_environment():
            action.visit(context)
        return (dict(context.environment),
                _perform(context, udp_profile()),
                {k: _perform(context, v) for k, v in udp_env().items()},
                _perform(context, FOXGLOVE_DEFAULT))
    finally:
        os.environ.clear()
        os.environ.update(saved)


def test_defaults_keep_dds_and_foxglove_off():
    environment, profile, pinned, foxglove = _resolve(
        {'FASTRTPS_DEFAULT_PROFILES_FILE': '/etc/fastdds/profile.xml'})
    assert 'ROS_AUTOMATIC_DISCOVERY_RANGE' not in environment
    assert environment['FASTRTPS_DEFAULT_PROFILES_FILE'].endswith('config/fastdds_local.xml')
    assert profile.endswith('fastdds_udp_only_local.xml')
    assert pinned['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'SYSTEM_DEFAULT'
    assert foxglove == 'false'


def test_remote_dds_on_by_argument_or_environment():
    for environment, arguments in (({}, {'remote_dds': 'true'}),
                                   ({'THORNBOTS_REMOTE_DDS': '1'}, {})):
        environment, profile, _, _ = _resolve(
            dict(environment, ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST'), arguments)
        assert 'ROS_AUTOMATIC_DISCOVERY_RANGE' not in environment
        assert environment['FASTRTPS_DEFAULT_PROFILES_FILE'] == '/etc/fastdds/profile.xml'
        assert profile.endswith('fastdds_udp_only.xml')


def test_foxglove_environment_default():
    assert _resolve({'THORNBOTS_FOXGLOVE': 'true'})[3] == 'true'
    assert isinstance(remote_dds_argument(), DeclareLaunchArgument)
