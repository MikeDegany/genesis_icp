# Copyright (c) 2026 Mike Degany <mike.degany@gmail.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# Offline genesis_icp: read two rosbag2 recordings directly (no relays, no ROS_DOMAIN_ID).

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('genesis_icp')
    default_fusion = os.path.join(pkg, 'config', 'genesis_icp.yaml')
    default_offline = os.path.join(pkg, 'config', 'offline_genesis_icp.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'fusion_config',
            default_value=default_fusion,
            description='Karto / fusion parameters (same as live genesis_icp node)',
        ),
        DeclareLaunchArgument(
            'offline_config',
            default_value=default_offline,
            description='Bag paths, per-robot topics, and offline-only options',
        ),
        Node(
            package='genesis_icp',
            executable='genesis_icp_offline_node',
            name='genesis_icp',
            output='screen',
            parameters=[
                LaunchConfiguration('fusion_config'),
                LaunchConfiguration('offline_config'),
            ],
        ),
    ])
