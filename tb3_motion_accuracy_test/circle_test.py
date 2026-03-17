"""
circle_test.py

Drive the robot in a circular path using constant linear and angular velocity.

Goal:
- Complete one full circle (2π radians of rotation)
- Measure how close the robot returns to its starting position
"""

import time
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from tb3_base_validation.result_utils import append_result


# ===== Test Settings =====
LINEAR_SPEED = 0.15        # m/s
ANGULAR_SPEED = 0.3        # rad/s

TARGET_ANGLE = 2 * math.pi   # full circle (360 deg)
ANGLE_TOL = math.radians(2)  # tolerance


class CircleTest(Node):
    def __init__(self):
        super().__init__('circle_test')

        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # Position tracking
        self.start_x = None
        self.start_y = None
        self.current_x = None
        self.current_y = None

        # Orientation tracking
        self.current_yaw = None
        self.start_yaw = None

        # Rotation tracking
        self.total_rotation = 0.0
        self.prev_yaw = None

        # Shutdown control
        self.done = False
        self.finish_time = None

        self.timer = self.create_timer(0.1, self.loop)

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Convert quaternion → yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.start_yaw = yaw
            self.get_logger().info('Captured starting position')

        # Track accumulated rotation (handles wrap-around)
        if self.prev_yaw is not None:
            delta = self.angle_diff(yaw, self.prev_yaw)
            self.total_rotation += abs(delta)

        self.prev_yaw = yaw

        self.current_x = x
        self.current_y = y
        self.current_yaw = yaw

    def publish(self, x=0.0, z=0.0):
        msg = TwistStamped()
        msg.twist.linear.x = x
        msg.twist.angular.z = z
        self.pub.publish(msg)

    def angle_diff(self, a, b):
        d = a - b
        return math.atan2(math.sin(d), math.cos(d))

    def stop_and_exit(self, final_error):
        self.publish()

        append_result(
            test_name='circle_test',
            status='PASS',
            measurement=f'{final_error:.3f} m',
            notes='loop closure error after full circle'
        )

        self.get_logger().info(
            f'Circle complete. Final position error: {final_error:.3f} m'
        )

        self.done = True
        self.finish_time = time.time()

    def loop(self):
        if self.done:
            if time.time() - self.finish_time > 0.5:
                self.get_logger().info('Exiting circle_test')
                rclpy.shutdown()
            return

        if self.current_x is None:
            return

        # Check if full rotation complete
        if self.total_rotation < TARGET_ANGLE - ANGLE_TOL:
            self.publish(x=LINEAR_SPEED, z=ANGULAR_SPEED)
        else:
            final_error = math.sqrt(
                (self.current_x - self.start_x) ** 2 +
                (self.current_y - self.start_y) ** 2
            )
            self.stop_and_exit(final_error)


def main(args=None):
    rclpy.init(args=args)
    node = CircleTest()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()