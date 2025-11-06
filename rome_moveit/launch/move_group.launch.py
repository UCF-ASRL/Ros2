import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

# Xacro
import xacro # type: ignore


def load_yaml(package_name, file_name):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_name)
    with open(absolute_file_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    # Robot Description
    path_to_urdf = os.path.join(get_package_share_directory('rome_moveit'),'model','rome_urdf.urdf.xacro')
    # robot_description_xml = xacro.process_file(path_to_urdf).toxml()

    hardware_choice = LaunchConfiguration("hardware")
    base_port_choice = LaunchConfiguration("base_port")
    arm_port_choice = LaunchConfiguration("arm_port")
    robot_description_xml = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        path_to_urdf,
        " ",
        "hardware:=",
        hardware_choice,
        " ",
        "base_port:=",
        base_port_choice,
        " ",
        "arm_port:=",
        arm_port_choice,
    ])

    robot_description = {"robot_description": robot_description_xml}

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_xml,
            'use_sim_time': True,
            'use_ros2_control': True
        }],
    )

    # MoveIt Configuration
    robot_description_semantic = {
    "robot_description_semantic": open(
        os.path.join(get_package_share_directory("rome_moveit"), "model", "rome_urdf.srdf"), "r"
    ).read()
    }

    robot_description_kinematics = {
        "robot_description_kinematics":
        load_yaml(
            "rome_moveit",
            os.path.join("config", "kinematics.yaml"),
        )
    }

    joint_limits = ParameterFile(
        PathJoinSubstitution([
            FindPackageShare("rome_moveit"),
            "config/joint_limits.yaml"
        ]),
        allow_substs=True,
    )

    # Planning Configuration
    ompl_planning_yaml = load_yaml("rome_moveit",
                                   "config/ompl_planning.yaml")
    planning_pipeline_config = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["ompl"],
        "ompl": ompl_planning_yaml,

    }

    # Trajectory Execution Configuration
    moveit_controllers = ParameterFile(
        PathJoinSubstitution([
            FindPackageShare("rome_moveit"),
            "config/moveit_controllers.yaml"
        ]),
        allow_substs=True,
    )

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "control_multi_dof_joint_variables": True,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description_semantic": True,
        "monitored_planning_scene.filters.exclude_joints": ["roller_.*"],
        "monitored_planning_scene.filters.ignore_missing_joints": True,
        "planning_frame": ["odom"],
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": True},
            {"ros__log_level": "error"},
        ],
    )

    # rviz with moveit configuration
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("rome_moveit"), "config", "moveit.rviz"])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            planning_pipeline_config,
            robot_description_kinematics,
            {
                "use_sim_time": True
            },
        ],
    )

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument("hardware",
                              default_value="gazebo",
                              choices=["gazebo","real"],
                              description="Which hardware? Gazebo or Real?"))
    ld.add_action(
        DeclareLaunchArgument("base_port",
                              default_value="0",
                              description="Which ACM Port for base?"))
    ld.add_action(
        DeclareLaunchArgument("arm_port",
                              default_value="0",
                              description="Which ACM Port for arm?"))
    
    ld.add_action(robot_state_publisher_node)
    ld.add_action(move_group_node)
    ld.add_action(rviz_node)
    return ld
