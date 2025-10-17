#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointTrajectoryControllerState
import time

class TrajectoryToTwistBridge(Node):
    def __init__(self):
        super().__init__('trajectory_to_twist_bridge')

        self.declare_parameter('controller_state_topic', '/fake_base_controller/controller_state')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('zero_debounce', 0.02)

        controller_topic = self.get_parameter('controller_state_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.zero_debounce = self.get_parameter('zero_debounce').value

        self.subscription = self.create_subscription(
            JointTrajectoryControllerState,
            controller_topic,
            self.controller_state_callback,
            10)
        self.publisher = self.create_publisher(TwistStamped, cmd_vel_topic, 10)

        self.last_velocities = [0.0, 0.0, 0.0]
        self.zero_candidate_since = None

        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_twist)
        self.get_logger().info(f"Bridge running: {controller_topic} → {cmd_vel_topic}")

    def controller_state_callback(self, msg):
        # Extract only reference velocities
        velocities = getattr(msg.reference, 'velocities', None)
        if velocities and len(velocities) >= 3:
            vx, vy, omega = velocities[:3]
            if any(abs(v) > 1e-8 for v in [vx, vy, omega]):
                self.last_velocities = [vx, vy, omega]
                self.zero_candidate_since = None
            else:
                if self.zero_candidate_since is None:
                    self.zero_candidate_since = time.monotonic()

    def publish_twist(self):
        now = time.monotonic()
        if self.zero_candidate_since and (now - self.zero_candidate_since) >= self.zero_debounce:
            self.last_velocities = [0.0, 0.0, 0.0]
            self.zero_candidate_since = None

        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'base_footprint'
        twist_msg.twist.linear.x = self.last_velocities[0]
        twist_msg.twist.linear.y = self.last_velocities[1]
        twist_msg.twist.angular.z = self.last_velocities[2]
        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryToTwistBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
