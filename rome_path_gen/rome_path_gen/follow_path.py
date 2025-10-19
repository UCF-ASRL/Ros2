#!/usr/bin/env python3
import os
import csv
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from ament_index_python.packages import get_package_share_directory
from moveit_py.core import MoveItPy

class FollowPathNode(Node):
    def __init__(self):
        super().__init__('follow_path_node')

        # Load MoveItPy instance
        self.moveit = MoveItPy(node_name="moveit_py")
        self.arm = self.moveit.get_planning_component("base_with_arm")
        self.scene = self.moveit.get_planning_scene_monitor()

        # Settings
        self.path_file = os.path.join(
            get_package_share_directory("rome_path_gen"), "paths", "rome_path.csv"
        )
        self.loop_forever = True
        self.pose_offset = [0.0, 0.0, 0.0]

        self.main_loop()

    def load_path(self):
        waypoints = []
        if self.path_file.endswith(".csv"):
            with open(self.path_file) as f:
                reader = csv.DictReader(f)
                for row in reader:
                    pose = Pose()
                    pose.position.x = float(row["x"]) + self.pose_offset[0]
                    pose.position.y = float(row["y"]) + self.pose_offset[1]
                    pose.position.z = float(row["z"]) + self.pose_offset[2]
                    pose.orientation.w = 1.0
                    waypoints.append(pose)
        elif self.path_file.endswith(".json"):
            with open(self.path_file) as f:
                data = json.load(f)
                for p in data["points"]:
                    pose = Pose()
                    pose.position.x = p["x"] + self.pose_offset[0]
                    pose.position.y = p["y"] + self.pose_offset[1]
                    pose.position.z = p["z"] + self.pose_offset[2]
                    pose.orientation.w = 1.0
                    waypoints.append(pose)
        self.get_logger().info(f"Loaded {len(waypoints)} waypoints.")
        return waypoints

    def follow_path(self, waypoints):
        for i, pose in enumerate(waypoints):
            self.arm.set_goal_state(pose_stamped=pose)
            plan_result = self.arm.plan()
            if plan_result:
                self.arm.execute()
                self.get_logger().info(f"✅ Executed waypoint {i+1}/{len(waypoints)}")
            else:
                self.get_logger().warn(f"⚠️ Failed to plan waypoint {i+1}")

    def main_loop(self):
        waypoints = self.load_path()
        if self.loop_forever:
            self.get_logger().info("♻️ Looping path forever.")
            while rclpy.ok():
                self.follow_path(waypoints)
        else:
            self.follow_path(waypoints)

def main():
    rclpy.init()
    node = FollowPathNode()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
