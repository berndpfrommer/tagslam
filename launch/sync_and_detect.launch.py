# -----------------------------------------------------------------------------
# Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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
        package="rosbag2_transport",
        executable="player",
        output="screen",
        # prefix=['xterm -e gdb -ex run --args'],
        name=cam_name,
        parameters=[
            trigger_config,  # loads the whole file
            {
                "use_multithreading": False,
                "statistics_print_interval": 2.0,
                # 'bias_file': bias_config,
                "camerainfo_url": "",
                "frame_id": "",
                "serial": LaunchConfig("serial"),
                "erc_mode": "enabled",
                "erc_rate": 100000000,
                # 'roi': [0, 0, 100, 100],
                # valid: 'external', 'loopback', 'disabled'
                "trigger_in_mode": "external",
                # valid: 'enabled', 'disabled'
                "trigger_out_mode": "enabled",
                "trigger_out_period": 100000,  # in usec
                "trigger_duty_cycle": 0.5,  # fraction high/low
                "event_message_time_threshold": 1.0e-3,
            },
        ],
        remappings=[("~/events", cam_str + "/events")],
    )
    return [node]


def generate_launch_description():
    """Create simple node by calling opaque function."""
    return launch.LaunchDescription(
        [
            LaunchArg("camera_name", default_value=["event_camera"], description="camera name"),
            LaunchArg("serial", default_value=[""], description="serial number of camera"),
            OpaqueFunction(function=launch_setup),
        ]
    )
