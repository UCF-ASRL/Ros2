# Physical Launch File for ROME
# Created By Bastian Weiss
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
    package_name = 'rome_moveit'

    # Controller Setup
    # WHEN CHANGING CONTROLLERS MAKE SURE TO CHANGE THESE VALUES
    # AND THE CONTROLLER FILE NAME IN ROS2_CONTROL.XACRO FILE!!!!
    controller_file = 'ros2_controllers.yaml'
    base_controller_name = 'omni_cont'

    #-------------------------------------------------------------------------------
    # Paths
    path_to_controller = os.path.join(get_package_share_directory(package_name), 'config', controller_file)

    ## Controller Section 
    controller_manager_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[path_to_controller],
    output="both",
	)

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad', '--controller-manager', '/controller_manager'],
    )
    # Controller Spawner
    base_controller_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=[
        base_controller_name,
        '--controller-manager', '/controller_manager',
        '--controller-ros-args',
        '--ros-args -r /omni_cont/tf_odometry:=/tf -r /omni_cont/reference:=/cmd_vel'
    ],
    output='screen',
    )

    # Arm Controller Spawner
    arm_controller_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['arm_controller','--controller-manager', '/controller_manager'],
    output='screen',
    )

    # Fake Base Controller Spawner
    fake_base_controller_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['fake_base_controller','--controller-manager', '/controller_manager'],
    output='screen',
    )

    # Moveit Base Bridge
    moveit_base_bridge_node = Node(
    package='rome_base_bridge',
    executable='trajectory_to_twist_bridge',
    name='trajectory_to_twist_bridge',
    output='screen'
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
    ld.add_action(controller_manager_node)
    ld.add_action(joint_broad_spawner)
    ld.add_action(base_controller_spawner)
    ld.add_action(arm_controller_spawner)
    ld.add_action(fake_base_controller_spawner)
    ld.add_action(moveit_base_bridge_node)

    ld.add_action(static_map_pub_node)

    return ld
