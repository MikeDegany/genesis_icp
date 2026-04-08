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
# Offline genesis_icp: play two rosbags (one per robot ROS domain) and run socket relays + fusion.
# Configuration: config/offline_genesis_icp.yaml (or offline_config:=/path/to/file.yaml).

import os
from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, OpaqueFunction, SetEnvironmentVariable
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def _resolve_bag_path(raw: str, config_dir: Path) -> str:
    if not raw or not str(raw).strip():
        raise ValueError('offline_genesis_icp.robot_a_bag / robot_b_bag must be set to a bag path')
    p = Path(os.path.expanduser(str(raw).strip()))
    if not p.is_absolute():
        p = (config_dir / p).resolve()
    if not p.exists():
        raise FileNotFoundError(f'Bag path does not exist: {p}')
    return str(p)


def _build_bag_play_cmd(uri: str, bag_play: dict) -> list:
    cmd = ['ros2', 'bag', 'play', uri]
    if bag_play.get('clock', True):
        cmd.append('--clock')
    rate = float(bag_play.get('rate', 1.0))
    if rate != 1.0:
        cmd.extend(['--rate', str(rate)])
    if bag_play.get('loop', False):
        cmd.append('--loop')
    start_off = float(bag_play.get('start_offset', 0.0))
    if start_off > 0.0:
        cmd.extend(['--start-offset', str(start_off)])
    extra = bag_play.get('additional_args') or []
    if isinstance(extra, list):
        cmd.extend(str(x) for x in extra)
    return cmd


def _env_with_domain(domain: int) -> dict:
    e = os.environ.copy()
    e['ROS_DOMAIN_ID'] = str(int(domain))
    return e


def _launch_setup(context, *args, **kwargs):
    cfg_path = Path(context.launch_configurations['offline_config']).expanduser().resolve()
    if not cfg_path.is_file():
        raise FileNotFoundError(f'offline_config not found: {cfg_path}')

    with cfg_path.open('r', encoding='utf-8') as f:
        doc = yaml.safe_load(f)
    if not doc or 'offline_genesis_icp' not in doc:
        raise ValueError(f'{cfg_path} must contain an "offline_genesis_icp:" mapping')
    oc = doc['offline_genesis_icp']

    config_dir = cfg_path.parent
    bag_a = _resolve_bag_path(oc['robot_a_bag'], config_dir)
    bag_b = _resolve_bag_path(oc['robot_b_bag'], config_dir)

    domain_a = int(oc.get('robot_a_ros_domain_id', 3))
    domain_b = int(oc.get('robot_b_ros_domain_id', 5))
    domain_f = int(oc.get('fusion_ros_domain_id', 0))
    ns_a = str(oc.get('robot_a_namespace', 'JK3'))
    ns_b = str(oc.get('robot_b_namespace', 'JK5'))

    use_sim_time = bool(oc.get('use_sim_time', True))
    scan_best_effort = bool(oc.get('scan_use_best_effort_qos', False))
    tf_static_tl = bool(oc.get('tf_static_use_transient_local', False))
    max_time_delta = float(oc.get('max_time_delta_sec', 10.0))

    bag_play = oc.get('bag_play') or {}

    default_fusion_yaml = os.path.join(
        get_package_share_directory('genesis_icp'), 'config', 'genesis_icp.yaml')
    fusion_file = oc.get('fusion_params_file')
    if fusion_file is None or str(fusion_file).strip() == '':
        fusion_yaml = default_fusion_yaml
    else:
        fp = Path(os.path.expanduser(str(fusion_file).strip()))
        if not fp.is_absolute():
            fp = (config_dir / fp).resolve()
        if not fp.is_file():
            raise FileNotFoundError(f'fusion_params_file not found: {fp}')
        fusion_yaml = str(fp)

    extras = oc.get('extra_fusion_parameters') or {}
    if not isinstance(extras, dict):
        raise ValueError('extra_fusion_parameters must be a mapping')

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

    play_a = ExecuteProcess(
        cmd=_build_bag_play_cmd(bag_a, bag_play),
        additional_env=_env_with_domain(domain_a),
        output='screen',
        shell=False,
    )
    play_b = ExecuteProcess(
        cmd=_build_bag_play_cmd(bag_b, bag_play),
        additional_env=_env_with_domain(domain_b),
        output='screen',
        shell=False,
    )

    relay_a = GroupAction([
        SetEnvironmentVariable(name='ROS_DOMAIN_ID', value=str(domain_a)),
        PushRosNamespace(ns_a),
        Node(
            package='genesis_icp',
            executable='genesis_icp_socket_relay',
            name='genesis_icp_socket_relay_a',
            output='screen',
            remappings=[
                ('/tf', f'/{ns_a}/tf'),
                ('/tf_static', f'/{ns_a}/tf_static'),
            ],
            parameters=[
                relay_a_params,
                {
                    'use_sim_time': use_sim_time,
                    'scan_use_best_effort_qos': scan_best_effort,
                    'tf_static_use_transient_local': tf_static_tl,
                },
            ],
        ),
    ])

    relay_b = GroupAction([
        SetEnvironmentVariable(name='ROS_DOMAIN_ID', value=str(domain_b)),
        PushRosNamespace(ns_b),
        Node(
            package='genesis_icp',
            executable='genesis_icp_socket_relay',
            name='genesis_icp_socket_relay_b',
            output='screen',
            remappings=[
                ('/tf', f'/{ns_b}/tf'),
                ('/tf_static', f'/{ns_b}/tf_static'),
            ],
            parameters=[
                relay_b_params,
                {
                    'use_sim_time': use_sim_time,
                    'scan_use_best_effort_qos': scan_best_effort,
                    'tf_static_use_transient_local': tf_static_tl,
                },
            ],
        ),
    ])

    fusion_params = [
        fusion_yaml,
        extras,
        {'use_sim_time': use_sim_time, 'max_time_delta_sec': max_time_delta},
    ]

    fusion = GroupAction([
        SetEnvironmentVariable(name='ROS_DOMAIN_ID', value=str(domain_f)),
        Node(
            package='genesis_icp',
            executable='genesis_icp_node',
            name='genesis_icp',
            output='screen',
            parameters=fusion_params,
        ),
    ])

    return [play_a, play_b, relay_a, relay_b, fusion]


def generate_launch_description():
    default_offline = os.path.join(
        get_package_share_directory('genesis_icp'),
        'config',
        'offline_genesis_icp.yaml',
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'offline_config',
            default_value=default_offline,
            description='YAML file with offline_genesis_icp.bag paths and options',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
