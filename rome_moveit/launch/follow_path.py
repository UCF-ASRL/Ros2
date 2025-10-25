#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit2 import MoveIt2
from moveit2.ros_planning_interface import MoveGroupInterface

class MoveToPose(Node):
    def __init__(self):
        super().__init__('move_to_pose')

        # Initialize MoveIt interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=[
                "arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5", "arm_joint_6"
            ],
            base_link_name="base_link",
            end_effector_name="ee_link",
            group_name="arm"
        )

        # Create goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "odom"
        goal_pose.pose.position.x = 0.4
        goal_pose.pose.position.y = 0.2
        goal_pose.pose.position.z = 0.5
        goal_pose.pose.orientation.w = 1.0

        # Execute the motion
        self.get_logger().info("Planning and executing to target pose...")
        self.moveit2.move_to_pose(goal_pose)
        self.get_logger().info("Motion complete.")

def main():
    rclpy.init()
    node = MoveToPose()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
