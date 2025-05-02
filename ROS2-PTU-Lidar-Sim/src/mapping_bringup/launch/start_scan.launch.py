"""
Copyright (C) 2025 Intel Corporation
SPDX-License-Identifier: Apache-2.0
+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
 +File name: start_scan.launch.py
 +Description: Launch file to start the world scan client node.
 +Author: Javier Felix-Rendon
 +Mail: javier.felix.rendon@intel.com
 +Version: 1.0
 +Date: 5/2/2025
+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_mapping_interfaces = get_package_share_directory('mapping_interfaces')
    pkg_project_mapping_custom_description = get_package_share_directory('mapping_custom_description')
    pkg_project_mapping_bringup = get_package_share_directory('mapping_bringup')
    pkg_project_mapping_gazebo = get_package_share_directory('mapping_gazebo')
    pkg_project_mapping_applications = get_package_share_directory('mapping_applications')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Load yaml parameters
    config = os.path.join(
        pkg_project_mapping_bringup,
        'config',
        'params.yaml'
    )

    world_scan_client_start = Node(
        package='mapping_applications',
        executable='world_scan_client',
        parameters=[config]
    )

    return LaunchDescription([
        world_scan_client_start,
    ])