# Physical Robot Launch File for Roam Base
# Created By Bastian Weiss
#---------------------------------------------
# Desired Output: Creates quadwheel base model in rviz and connects to Arduino.
#---------------------------------------------

# Imports
# For os path commands
import os

# Importing Model and world and launch files
from ament_index_python.packages import get_package_share_directory # type: ignore

# Import Ros Launch
import launch_ros # type: ignore
from launch_ros.actions import Node # type: ignore
from launch_ros.substitutions import FindPackageShare # type: ignore

# Core structure
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler # type: ignore
from launch.event_handlers import OnProcessExit # type: ignore
from launch.launch_description_sources import PythonLaunchDescriptionSource # type: ignore
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution # type: ignore

# Gazebo
from ros_gz_bridge.actions import RosGzBridge # type: ignore
from ros_gz_sim.actions import GzServer # type: ignore

# Xacro
import xacro # type: ignore

#----------------------------------------------------------------------------
# Launch
def generate_launch_description():

	# Names
	# Base Files
	package_name = 'roam_base'
	urdf_name = 'main.xacro'
	urdf_folder_name = 'model'
	rviz_param_file = 'rviz_config.rviz'

	# Controller Setup
	## WHEN CHANGING CONTROLLERS MAKE SURE TO CHANGE THESE VALUES !!AND!! THE CONTROLLER FILE NAME IN ROS2_CONTROL.XACRO FILE!!!!
	controller_file = 'omni_controller.yaml'
	controller_name = 'omni_cont'

	
	#-------------------------------------------------------------------------------
	# Paths
	path_to_urdf = os.path.join(get_package_share_directory(package_name),urdf_folder_name,urdf_name)
	path_to_controller = os.path.join(get_package_share_directory(package_name),'config',controller_file)
	path_to_rviz_params = os.path.join(get_package_share_directory(package_name),'config',rviz_param_file)

	# Robot Description 
	robot_description = xacro.process_file(path_to_urdf).toxml()
	
	# Publishers
	# Robot State Publisher
	robot_state_publisher_node = Node(
   		package='robot_state_publisher',
        executable='robot_state_publisher',
		output='screen',
        	parameters=[{'robot_description': robot_description,
			     'use_sim_time': True,
				 'use_ros2_control': True
		           }],     
	)
	
    # Joint State Publisher
	joint_state_publisher_gui_node = Node(
        	package='joint_state_publisher_gui',
          	executable='joint_state_publisher_gui',
        	name='joint_state_publisher_gui',
			parameters=[{'use_sim_time' : True}],
	)
	
	# RVIZ
	rviz_node = Node(
    		package='rviz2',
        	executable='rviz2',
        	name='rviz2',
        	output='screen',
			arguments=['-d',path_to_rviz_params],
    	)
	
	## Controller Section 
	joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["joint_broad"],
    )

	# Controller Spawner
	# Wheel Controller
	controller_spawner = Node(
		package='controller_manager',
		executable='spawner',
		parameters=[{'robot_description': robot_description}],
        arguments=[
			controller_name,
			'--param-file',
			path_to_controller,
            '--controller-ros-args',
            '-r /omni_cont/tf_odometry:=/tf',
            '--controller-ros-args',
            '-r /omni_cont/reference:=/cmd_vel',
			],
	)

	# delay rviz start after `joint_state_broadcaster`
	delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
		event_handler=OnProcessExit(
			target_action=joint_broad_spawner,
			on_exit=[rviz_node],
		)
	)

    # delay start of robot_controller after `joint_state_broadcaster`
	delay_motor_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
		event_handler=OnProcessExit(
			target_action=joint_broad_spawner,
			on_exit=[controller_spawner],
		)
	)

	#-----------------------------------------------------------S
	# here we create an empty launch description object
	ld = LaunchDescription()

	# Add Launch Nodes
	ld.add_action(robot_state_publisher_node)
	ld.add_action(joint_state_publisher_gui_node)
	ld.add_action(rviz_node)

	ld.add_action(joint_broad_spawner)
	ld.add_action(controller_spawner)

	ld.add_action(delay_rviz_after_joint_state_broadcaster_spawner)
	ld.add_action(delay_motor_controller_spawner_after_joint_state_broadcaster_spawner)
	
	return ld
