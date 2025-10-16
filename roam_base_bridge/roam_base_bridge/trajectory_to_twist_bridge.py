#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class MoveItBaseConverter(Node):
    def __init__(self):
        super().__init__('moveit_base_converter')

        # Subscribe to MoveIt planned trajectory for the base
        self.subscription = self.create_subscription(
            JointTrajectory,
            '/fake_base_controller/joint_trajectory',  # should match MoveIt output topic
            self.trajectory_callback,
            10
        )

        # Publish velocity commands for omniwheel controller
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publish joint states for fake base so MoveIt sees them
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Store previous point for velocity estimation
        self.prev_point = None
        self.prev_time = None

        # Store latest joint positions
        self.current_positions = [0.0, 0.0, 0.0]  # base_x, base_y, base_theta

    def trajectory_callback(self, msg: JointTrajectory):
        if not msg.points:
            return

        for point_idx, point in enumerate(msg.points):
            dt = 0.01  # fallback small dt
            vx, vy, omega = 0.0, 0.0, 0.0

            if point.velocities and len(point.velocities) >= 3:
                # Use MoveIt velocities directly
                vx, vy, omega = point.velocities[:3]
            elif len(point.positions) >= 3:
                # Numerical differentiation
                if self.prev_point is not None:
                    t_now = point.time_from_start.sec + point.time_from_start.nanosec*1e-9
                    dt = t_now - self.prev_time
                    if dt <= 0:
                        dt = 0.01
                    vx = (point.positions[0] - self.prev_point[0]) / dt
                    vy = (point.positions[1] - self.prev_point[1]) / dt
                    omega = (point.positions[2] - self.prev_point[2]) / dt

                self.prev_point = point.positions[:3]
                self.prev_time = point.time_from_start.sec + point.time_from_start.nanosec*1e-9

            # Publish as Twist
            twist = Twist()
            twist.linear.x = vx
            twist.linear.y = vy
            twist.angular.z = omega
            self.cmd_pub.publish(twist)

            # Update current base positions
            self.current_positions[0] += vx * dt
            self.current_positions[1] += vy * dt
            self.current_positions[2] += omega * dt

            # Publish JointState for MoveIt
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = ['base_x', 'base_y', 'base_theta']
            js.position = self.current_positions
            self.js_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = MoveItBaseConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
