# Physical Robot Launch File for Rome Base
# Created By Bastian Weiss
#---------------------------------------------
# Desired Output: Creates quadwheel base model in rviz and connects to Arduino.
#---------------------------------------------

# Imports
import os
import xacro  # type: ignore

# ROS2 Launch Imports
from ament_index_python.packages import get_package_share_directory  # type: ignore
import launch_ros  # type: ignore
from launch_ros.actions import Node  # type: ignore

from launch import LaunchDescription
from launch.actions import RegisterEventHandler  # type: ignore
from launch.event_handlers import OnProcessExit  # type: ignore
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution  # type: ignore

# Gazebo
from ros_gz_bridge.actions import RosGzBridge  # type: ignore
from ros_gz_sim.actions import GzServer  # type: ignore

#----------------------------------------------------------------------------
# Launch
def generate_launch_description():

    # Names
    package_name = 'rome_moveit'
    urdf_name = 'main.xacro'
    urdf_folder_name = 'model'
    rviz_param_file = 'rviz_config.rviz'

    # Controller Setup
    # WHEN CHANGING CONTROLLERS MAKE SURE TO CHANGE THESE VALUES !!AND!! THE CONTROLLER FILE NAME IN ROS2_CONTROL.XACRO FILE!!!!
    controller_file = 'ros2_controllers.yaml'
    controller_name = 'omni_cont'

    #-------------------------------------------------------------------------------
    # Paths
    path_to_urdf = os.path.join(get_package_share_directory(package_name), urdf_folder_name, urdf_name)
    path_to_controller = os.path.join(get_package_share_directory(package_name), 'config', controller_file)
    path_to_rviz_params = os.path.join(get_package_share_directory(package_name), 'config', rviz_param_file)

    # Robot Description

    #-------------------------------------------------------------------------------
    # Nodes

    # RVIZ
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', path_to_rviz_params],
    )

    # Controller Manager Node
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[path_to_controller],
        output="both",
    )

    # Joint State Broadcaster
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["joint_broad"],
    )

    # Fake Base Controller Spawner
    fake_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fake_base_controller', '--param-file', path_to_controller],
        output='screen',
    )

    # Wheel Controller Spawner
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            controller_name,
            '--param-file', path_to_controller,
            '--controller-ros-args', '-r /omni_cont/tf_odometry:=/tf',
            '--controller-ros-args', '-r /omni_cont/reference:=/cmd_vel',
        ],
    )

    # MoveIt Base Bridge
    moveit_base_bridge_node = Node(
        package='rome_base_bridge',
        executable='trajectory_to_twist_bridge',
        name='trajectory_to_twist_bridge',
        output='screen',
    )

    #-------------------------------------------------------------------------------
    # Event Handlers

    # Delay RVIZ start after joint_state_broadcaster
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of wheel controller after joint_state_broadcaster
    delay_motor_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[controller_spawner],
        )
    )

    #-------------------------------------------------------------------------------
    # Launch Description
    ld = LaunchDescription()

    # Add Nodes
    ld.add_action(controller_manager_node)
    ld.add_action(joint_broad_spawner)
    ld.add_action(fake_base_controller_spawner)
    ld.add_action(moveit_base_bridge_node)
    ld.add_action(delay_rviz_after_joint_state_broadcaster_spawner)
    ld.add_action(delay_motor_controller_spawner_after_joint_state_broadcaster_spawner)

    return ld
