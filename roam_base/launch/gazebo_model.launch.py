# Gazebo Launch File for Roam Base
# Created By Bastian Weiss
#---------------------------------------------
# Desired Output: Creates quadwheel base model in rviz and gazebo.
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
	robot_name = 'roam_base'
	world_name = 'lab_world.world'
	urdf_name = 'main.xacro'
	urdf_folder_name = 'model'
	rviz_param_file = 'rviz_config.rviz'

	# Controller Setup
	## WHEN CHANGING CONTROLLERS MAKE SURE TO CHANGE THESE VALUES !!AND!! THE CONTROLLER FILE NAME IN ROS2_CONTROL.XACRO FILE!!!!
	controller_file = 'omni_controller.yaml'
	controller_name = 'omni_cont'
	bridge_param_file = 'gz_bridge_params.yaml'

	
	#-------------------------------------------------------------------------------
	# Variables
	world = LaunchConfiguration('world') 
	
	# Paths
	path_to_urdf = os.path.join(get_package_share_directory(package_name),urdf_folder_name,urdf_name)
	path_to_world = os.path.join(get_package_share_directory(package_name),urdf_folder_name,world_name)
	path_to_controller = os.path.join(get_package_share_directory(package_name),'config',controller_file)
	path_to_bridge_params = os.path.join(get_package_share_directory(package_name),'config',bridge_param_file)
	path_to_rviz_params = os.path.join(get_package_share_directory(package_name),'config',rviz_param_file)

	# Robot Description 
	robot_description = xacro.process_file(path_to_urdf).toxml()
	
	#Find ros_gz_sim package
	pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
	gz_launch_path = PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'))

	#Launch Configs
	declare_world_cmd = DeclareLaunchArgument(
    		name='world',
    		default_value=path_to_world,
    		description='Full path to the world model file to load'
	)

	# Launch Gazebo -  (-r -v4 is for debug, shudown closes everything when gazebo does.)
	gz_server = IncludeLaunchDescription(gz_launch_path,
		launch_arguments={'gz_args': [' -r -v4 ', path_to_world], 'on_exit_shutdown': 'true'}.items(),
	)

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
		
	#joint_state_publisher_gui_node = Node(
    #    	package='joint_state_publisher_gui',
    #   	executable='joint_state_publisher_gui',
    #    	name='joint_state_publisher_gui',
	#		parameters=[{'use_sim_time' : True}],
	#)
	
	# RVIZ
	rviz_node = Node(
    		package='rviz2',
        	executable='rviz2',
        	name='rviz2',
        	output='screen',
			arguments=['-d',path_to_rviz_params],
    	)
	
	# Spawn Gazebo Model - Same as gazebo_ros spawn_eneity.py from old versions
	spawn_model_node = Node(
		package='ros_gz_sim',
		executable='create',
		arguments=['-topic',
			'robot_description',
			'-name',
			robot_name,
			'-z', '1'], #spawn robot in sky for a second fix later
			output='screen',
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
        arguments=["/camera_1/image",
				   "/camera_2/image",
				   "/camera_3/image",
				   "/camera_4/image",
				   "/camera_5/image",
				   "/camera_6/image"],
    )

	# Static Transfrom Publishers for Camera Point Clouds
	camera_1_transform_node = Node(
		package="tf2_ros",
		executable="static_transform_publisher",
		arguments = ["-2.44","-2.64","2.54",
			   "-0.1463","0.3534","0.3534","0.8537",
				 "odom", "camera_1_optical"], #X Y Z YAW PITCH ROLL ??????
	)

	camera_2_transform_node = Node(
		package="tf2_ros",
		executable="static_transform_publisher",
		arguments = ["0","-2.64","2.54","1.5707","0.785","0", "odom", "camera_2_link"],
	)
	



	#-----------------------------------------------------------S
	# here we create an empty launch description object
	ld = LaunchDescription()

	# Add Launch Nodes
	ld.add_action(declare_world_cmd)
	ld.add_action(gz_server)
	ld.add_action(spawn_model_node)
	ld.add_action(robot_state_publisher_node)
	# ld.add_action(joint_state_publisher_node)
	# ld.add_action(joint_state_publisher_gui_node)
	ld.add_action(rviz_node)

	ld.add_action(joint_broad_spawner)
	ld.add_action(controller_spawner)
	
	ld.add_action(ros_gz_bridge_node)
	ld.add_action(ros_gz_image_bridge_node)

	ld.add_action(camera_1_transform_node)
	ld.add_action(camera_2_transform_node)

	

	return ld
