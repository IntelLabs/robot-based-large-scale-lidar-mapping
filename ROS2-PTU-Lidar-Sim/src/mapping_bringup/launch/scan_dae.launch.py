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

    #Load yaml parameters
    config = os.path.join(
    pkg_project_mapping_bringup,
    'config',
    'params.yaml'
    )

    # Load the SDF file from "description" package
    sdf_file  =  os.path.join(pkg_project_mapping_custom_description, 'models', 'ptu_150cm', 'model_description.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_mapping_gazebo,
            'worlds',
            'ptu_lidar.sdf'
        ])}.items(),
    )

     # Load World to scan
    file = os.path.join(
        pkg_project_mapping_custom_description,
        'models',
        'dae',
        'model.sdf'

    )
    gz_world_create = ExecuteProcess(
        cmd=[[
            'ros2 run ros_gz_sim create --args -file "',
            file,
            '" -name build_model'          
        ]],
        shell=True
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_mapping_bringup, 'config', 'bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    wss = Node(
        package='mapping_applications',
        executable='world_scan_server'
    )

    tl2w = Node(
        package='mapping_applications',
        executable='transform_lidar2world',
        parameters= [config]
    )

    l360 = Node(
        package='mapping_applications',
        executable='l360_scan_node'
    )

    normal = Node(
        package='mapping_applications',
        executable='compute_normal'
    )

    ply_conv = Node(
        package='mapping_applications',
        executable='ply_converter',
        parameters= [config]
    )

    np = Node(
        package='mapping_applications',
        executable='pc2numpy.py',
        parameters= [config]
    )

    np_low = Node(
        package='mapping_applications',
        executable='pc2numpy_low.py',
        parameters= [config]
    )

    #Unpause simulation
    bridge_unpause = ExecuteProcess(
        cmd=[[
            'ros2 run ros_gz_bridge parameter_bridge /world/ptu_lidar_world/control@ros_gz_interfaces/srv/ControlWorld'
        ]],
        shell=True
    )

    unpause = ExecuteProcess(
        cmd=[[
            'ros2 service call /world/ptu_lidar_world/control ros_gz_interfaces/srv/ControlWorld "{world_control: {pause: false}}"'
        ]],
        shell=True
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        robot_state_publisher,
        wss,
        tl2w,
        l360,
        normal,
        ply_conv,
        np,
        #np_low,
        gz_world_create,
        bridge_unpause,
        unpause,
        
    ])