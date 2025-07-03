# Gazebo Launch File for Quadwheel Base
# Created By Bastian Weiss
#---------------------------------------------
# Desired Output: Sends Publisher Values to Joint Group Velocity Controller
#---------------------------------------------

# Imports
# For os path commands
import os

# Importing Model and world and launch files
from ament_index_python.packages import get_package_share_directory # type: ignore

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution # type: ignore
from launch_ros.actions import Node # type: ignore
from launch_ros.substitutions import FindPackageShare # type: ignore


def generate_launch_description():

    # Names
    package_name = 'quadwheel'
    publisher_file = 'velocity_publisher.yaml'
    publisher_name = 'publisher_forward_velocity_controller'

    #-------------------------------------------------------------------------------
    # Paths
    path_to_publisher = os.path.join(get_package_share_directory(package_name),'config',publisher_file)

    publisher_node = Node(
        package="ros2_controllers_test_nodes",
        executable="publisher_forward_position_controller",
        name=publisher_name,
        parameters=[path_to_publisher],
        output="both",
    )

    #-----------------------------------------------------------
    ld = LaunchDescription()

    ld.add_action(publisher_node)

    return ld