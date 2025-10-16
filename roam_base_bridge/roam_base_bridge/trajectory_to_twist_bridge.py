#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointTrajectoryControllerState


class TrajectoryToTwistBridge(Node):
    def __init__(self):
        super().__init__('trajectory_to_twist_bridge')

        # Parameters
        self.declare_parameter('controller_state_topic', '/fake_base_controller/controller_state')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('timeout', 0.2)

        controller_topic = self.get_parameter('controller_state_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value

        # Publisher
        self.cmd_vel_pub = self.create_publisher(TwistStamped, cmd_vel_topic, 10)

        # Subscriber
        self.subscription = self.create_subscription(
            JointTrajectoryControllerState,
            controller_topic,
            self.controller_state_callback,
            10
        )

        # Internal state
        self.prev_positions = None
        self.prev_time = None
        self.last_update_time = self.get_clock().now()

        # Timer to send zero cmd when idle
        self.create_timer(0.05, self.zero_check_timer_cb)

        self.get_logger().info(f"Subscribed to: {controller_topic}")
        self.get_logger().info(f"Publishing TwistStamped to: {cmd_vel_topic}")

    def controller_state_callback(self, msg: JointTrajectoryControllerState):
        # Try to get positions (Jazzy: msg.feedback, earlier: msg.actual)
        positions = []
        try:
            if hasattr(msg, 'feedback') and hasattr(msg.feedback, 'positions'):
                positions = msg.feedback.positions
            elif hasattr(msg, 'actual') and hasattr(msg.actual, 'positions'):
                positions = msg.actual.positions
            elif hasattr(msg, 'positions'):
                positions = msg.positions
        except Exception as e:
            self.get_logger().error(f"Failed to read positions: {e}")
            return

        if len(positions) < 3:
            self.get_logger().warn("Controller state message does not contain at least 3 positions.")
            return

        # Compute velocities by finite difference
        now = self.get_clock().now()
        if self.prev_positions is not None and self.prev_time is not None:
            dt = (now - self.prev_time).nanoseconds * 1e-9
            if dt > 0:
                vx = (positions[0] - self.prev_positions[0]) / dt
                vy = (positions[1] - self.prev_positions[1]) / dt
                omega = (positions[2] - self.prev_positions[2]) / dt

                twist_msg = TwistStamped()
                twist_msg.header.stamp = now.to_msg()
                twist_msg.header.frame_id = self.frame_id
                twist_msg.twist.linear.x = vx
                twist_msg.twist.linear.y = vy
                twist_msg.twist.angular.z = omega
                self.cmd_vel_pub.publish(twist_msg)

        # Save for next update
        self.prev_positions = positions
        self.prev_time = now
        self.last_update_time = now

    def zero_check_timer_cb(self):
        """Publish zero Twist if idle."""
        now = self.get_clock().now()
        if (now - self.last_update_time).nanoseconds * 1e-9 > self.timeout:
            twist_msg = TwistStamped()
            twist_msg.header.stamp = now.to_msg()
            twist_msg.header.frame_id = self.frame_id
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.linear.y = 0.0
            twist_msg.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryToTwistBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
