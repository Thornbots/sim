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
Launch gz sim as the launch's direct child, with no shell in between.

ros_gz_sim's gz_sim.launch.py runs `sh -c "ruby gz sim ..."`. Launch signals
only that shell, which dies and leaves the gz server (the ruby process)
orphaned, so a graceful shutdown needed a group-wide signal or SIGKILL.
The environment still comes from ros_gz_sim's own helper.
"""
import importlib.util
import os
import shutil

from ament_index_python.packages import get_package_share_directory

from launch.actions import ExecuteProcess

GZ_VERSION = '8'


def _ros_gz_paths():
    """Return ros_gz_sim's (model, plugin) path lists, from its launch file."""
    path = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    spec = importlib.util.spec_from_file_location('ros_gz_sim_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.GazeboRosPaths.get_paths()


def gz_sim_process(arguments, name='gazebo'):
    """Return an ExecuteProcess running `gz sim <arguments>` without a shell."""
    gz = shutil.which('gz')
    if gz is None:
        raise RuntimeError('gz is not on PATH; run install-sim.sh first')
    model_paths, plugin_paths = _ros_gz_paths()
    env = {
        'GZ_SIM_SYSTEM_PLUGIN_PATH': os.pathsep.join([
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
            os.environ.get('LD_LIBRARY_PATH', ''),
            plugin_paths]),
        'GZ_SIM_RESOURCE_PATH': os.pathsep.join([
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''), model_paths]),
    }
    return ExecuteProcess(
        cmd=['ruby', gz, 'sim', *arguments, '--force-version', GZ_VERSION],
        name=name, output='screen', additional_env=env, shell=False)
