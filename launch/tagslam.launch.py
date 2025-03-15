# -----------------------------------------------------------------------------
# Copyright 2025 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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
#
#

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Create simple node."""
    node = Node(
        package="tagslam",
        executable="tagslam_node",
        output="screen",
        # prefix=['xterm -e gdb -ex run --args'],
        parameters=[{"cameras": LaunchConfig("cameras"),
                     "camera_poses": LaunchConfig("camera_poses"),
                     "tagslam_config": LaunchConfig("tagslam_config"),
                     "use_sim_time": LaunchConfig("use_sim_time"),
                     "use_approximate_sync": LaunchConfig("use_approximate_sync")}],
        remappings=[],
    )
    return [node]


def generate_launch_description():
    """Create simple node by calling opaque function."""
    return launch.LaunchDescription(
        [
            LaunchArg("cameras", default_value=["cameras.yaml"], description="name of cameras calib file"),
            LaunchArg("camera_poses", default_value=["camera_poses.yaml"], description="name of camera poses file"),
            LaunchArg("tagslam_config", default_value=["tagslam.yaml"], description="name of tagslam config file"),
            LaunchArg("use_sim_time", default_value=["False"], description="whether to use sim time"),
            LaunchArg("use_approximate_sync", default_value=["True"], description="whether to use approximate sync"),
            OpaqueFunction(function=launch_setup),
        ]
    )
