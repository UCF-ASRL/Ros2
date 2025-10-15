# Gazebo Launch File for Roam Base
# Created By Bastian Weiss
#---------------------------------------------
# Desired Output: Creates quadwheel base model in rviz and gazebo.
#---------------------------------------------

# Imports
import os

from ament_index_python.packages import get_package_share_directory  # type: ignore
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription  # type: ignore
from launch.launch_description_sources import PythonLaunchDescriptionSource  # type: ignore
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution  # type: ignore
from launch_ros.actions import Node  # type: ignore
from launch_ros.substitutions import FindPackageShare  # type: ignore

from ros_gz_bridge.actions import RosGzBridge  # type: ignore
from ros_gz_sim.actions import GzServer  # type: ignore

import xacro  # type: ignore


#-------------------------------------------------------------------------------
# Launch
def generate_launch_description():

    # Names
    package_name = 'roam_moveit'
    robot_name = 'ASRL_ROAM'
    world_name = 'lab_world.world'
    urdf_folder_name = 'model'

    # Controller Setup
    # WHEN CHANGING CONTROLLERS MAKE SURE TO CHANGE THESE VALUES
    # AND THE CONTROLLER FILE NAME IN ROS2_CONTROL.XACRO FILE!!!!
    controller_file = 'ros2_controllers.yaml'
    base_controller_name = 'omni_cont'
    bridge_param_file = 'gz_bridge_params.yaml'

    #-------------------------------------------------------------------------------
    # Paths
    path_to_urdf = os.path.join(get_package_share_directory('roam_moveit'),'model','roam_urdf.urdf.xacro')
    path_to_world = os.path.join(get_package_share_directory(package_name), urdf_folder_name, world_name)
    path_to_controller = os.path.join(get_package_share_directory(package_name), 'config', controller_file)
    path_to_bridge_params = os.path.join(get_package_share_directory(package_name), 'config', bridge_param_file)

    # Robot Description 
    robot_description = xacro.process_file(path_to_urdf).toxml()

    # Find ros_gz_sim package
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'))

    # Launch Configs
    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=path_to_world,
        description='Full path to the world model file to load'
    )

    #--------------------------------------------------------------------
    # Launch Gazebo
    gz_server = IncludeLaunchDescription(
        gz_launch_path,
        launch_arguments={
            'gz_args': [' -r -v4 ', path_to_world],
            'on_exit_shutdown': 'true'
        }.items(),
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
            'use_ros2_control': True
        }],
    )

    # Clean Up Rollers
    dummy_joint_node = Node(
    package="joint_state_publisher",
    executable="joint_state_publisher",
    output="screen",
    parameters=[os.path.join(get_package_share_directory("roam_moveit"), "config", "dummy_rollers.yaml")]
    )


    # Spawn Gazebo Model
    spawn_model_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', robot_name,
            '-z', '1'
        ],
        output='screen',
    )

    ## Controller Section
    # Controller Spawner
    base_controller_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=[
        base_controller_name,
        '--param-file', path_to_controller,
        '--controller-ros-args',
        '--ros-args -r /omni_cont/tf_odometry:=/tf -r /omni_cont/reference:=/cmd_vel'
    ],
    output='screen',
    )

    # Arm Controller Spawner
    arm_controller_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['arm_controller', '--param-file', path_to_controller],
    output='screen',
    )


    # Ros Gz Bridge
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={path_to_bridge_params}',
        ],
    )

    # Camera Bridge
    ros_gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/camera_1/image",
            "/camera_2/image",
            "/camera_3/image",
            "/camera_4/image",
            "/camera_5/image",
            "/camera_6/image",
        ],
    )

    # Static Map Publisher
    static_map_pub_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )

    #--------------------------------------------------------------------
    # Launch Description
    ld = LaunchDescription()

    ld.add_action(declare_world_cmd)

    ld.add_action(gz_server)
    ld.add_action(spawn_model_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(dummy_joint_node)

    ld.add_action(base_controller_spawner)
    ld.add_action(arm_controller_spawner)

    ld.add_action(ros_gz_bridge_node)
    ld.add_action(ros_gz_image_bridge_node)
    ld.add_action(static_map_pub_node)

    return ld
