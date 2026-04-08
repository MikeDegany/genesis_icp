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

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    cfg = LaunchConfiguration('config_file')
    fusion_domain = LaunchConfiguration('fusion_ros_domain_id')
    domain_a = LaunchConfiguration('robot_a_ros_domain_id')
    domain_b = LaunchConfiguration('robot_b_ros_domain_id')
    ns_a = LaunchConfiguration('robot_a_namespace')
    ns_b = LaunchConfiguration('robot_b_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    max_time_delta = LaunchConfiguration('max_time_delta_sec')
    scan_best_effort = LaunchConfiguration('scan_use_best_effort_qos')
    tf_static_tl = LaunchConfiguration('tf_static_use_transient_local')

    declare_use_sim = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Set true when using ros2 bag play --clock so TF and scans use /clock')

    declare_max_dt = DeclareLaunchArgument(
        'max_time_delta_sec',
        default_value='10.0',
        description='Max |t_A - t_B| for pairing scans (seconds); use >=5 when replaying bags with offset start times')

    declare_scan_qos = DeclareLaunchArgument(
        'scan_use_best_effort_qos',
        default_value='false',
        description='Relays: true = SensorDataQoS (typical live laser); false = reliable+volatile (rosbag2 play)')

    declare_tf_static_qos = DeclareLaunchArgument(
        'tf_static_use_transient_local',
        default_value='false',
        description='Relays: true for live /tf_static (robot_state_publisher); false for rosbag2 play (volatile)')

    declare_cfg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            get_package_share_directory('genesis_icp'),
            'config',
            'genesis_icp.yaml'),
        description='YAML parameters for the fusion node (genesis_icp)')

    declare_fusion_domain = DeclareLaunchArgument(
        'fusion_ros_domain_id',
        default_value='0',
        description='ROS_DOMAIN_ID for fusion node (default 0)')

    declare_da = DeclareLaunchArgument(
        'robot_a_ros_domain_id',
        default_value='3',
        description='ROS_DOMAIN_ID for robot A relay')

    declare_db = DeclareLaunchArgument(
        'robot_b_ros_domain_id',
        default_value='5',
        description='ROS_DOMAIN_ID for robot B relay')

    declare_ns_a = DeclareLaunchArgument(
        'robot_a_namespace',
        default_value='JK3',
        description='ROS namespace for robot A relay (must match slam_toolbox namespace)')

    declare_ns_b = DeclareLaunchArgument(
        'robot_b_namespace',
        default_value='JK5',
        description='ROS namespace for robot B relay')

    relay_a_params = {
        'tcp_host': '127.0.0.1',
        'tcp_port': 4403,
        'scan_topic': 'sensors/lidar2d_0/scan',
        'odom_frame': 'odom',
        'base_frame': 'base_link',
    }

    relay_b_params = {
        'tcp_host': '127.0.0.1',
        'tcp_port': 4405,
        'scan_topic': 'sensors/lidar2d_0/scan',
        'odom_frame': 'odom',
        'base_frame': 'base_link',
    }

    relay_a = GroupAction([
        SetEnvironmentVariable(name='ROS_DOMAIN_ID', value=domain_a),
        PushRosNamespace(ns_a),
        Node(
            package='genesis_icp',
            executable='genesis_icp_socket_relay',
            name='genesis_icp_socket_relay_a',
            output='screen',
            # TransformListener hardcodes /tf and /tf_static; bags use /JK3/tf.
            remappings=[
                ('/tf', '/JK3/tf'),
                ('/tf_static', '/JK3/tf_static'),
            ],
            parameters=[
                relay_a_params,
                {
                    'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                    'scan_use_best_effort_qos': ParameterValue(scan_best_effort, value_type=bool),
                    'tf_static_use_transient_local': ParameterValue(tf_static_tl, value_type=bool),
                },
            ],
        ),
    ])

    relay_b = GroupAction([
        SetEnvironmentVariable(name='ROS_DOMAIN_ID', value=domain_b),
        PushRosNamespace(ns_b),
        Node(
            package='genesis_icp',
            executable='genesis_icp_socket_relay',
            name='genesis_icp_socket_relay_b',
            output='screen',
            remappings=[
                ('/tf', '/JK5/tf'),
                ('/tf_static', '/JK5/tf_static'),
            ],
            parameters=[
                relay_b_params,
                {
                    'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                    'scan_use_best_effort_qos': ParameterValue(scan_best_effort, value_type=bool),
                    'tf_static_use_transient_local': ParameterValue(tf_static_tl, value_type=bool),
                },
            ],
        ),
    ])

    fusion = GroupAction([
        SetEnvironmentVariable(name='ROS_DOMAIN_ID', value=fusion_domain),
        Node(
            package='genesis_icp',
            executable='genesis_icp_node',
            name='genesis_icp',
            output='screen',
            parameters=[
                cfg,
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
                {'max_time_delta_sec': ParameterValue(max_time_delta, value_type=float)},
            ],
        ),
    ])

    return LaunchDescription([
        declare_use_sim,
        declare_max_dt,
        declare_scan_qos,
        declare_tf_static_qos,
        declare_cfg,
        declare_fusion_domain,
        declare_da,
        declare_db,
        declare_ns_a,
        declare_ns_b,
        relay_a,
        relay_b,
        fusion,
    ])
