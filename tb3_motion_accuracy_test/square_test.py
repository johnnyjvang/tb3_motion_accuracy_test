"""
square_test.py

Drive the robot in a square path:
- Move forward a set distance
- Rotate 90 degrees
- Repeat 4 times

At the end, report how far the robot is from the starting point
(loop closure error).
"""

import time
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from tb3_motion_accuracy_test.result_utils import append_result


# ===== Test Settings =====
SIDE_LENGTH = 1.0                    # meters per side
LINEAR_SPEED = 0.15                  # m/s
MAX_ANGULAR_SPEED = 0.5              # rad/s
MIN_ANGULAR_SPEED = 0.1              # rad/s

TARGET_TURN = math.radians(90.0)     # 90 degrees
ANGLE_TOL = math.radians(1.0)        # 1 degree tolerance


class SquareTest(Node):
    def __init__(self):
        super().__init__('square_test')

        # Publisher for robot velocity commands
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Subscriber for odometry feedback
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # Position tracking
        self.start_x = None
        self.start_y = None
        self.current_x = None
        self.current_y = None

        # Orientation tracking
        self.current_yaw = None

        # Test state machine
        self.state = 'forward'   # forward -> rotate -> forward ...
        self.side_count = 0

        # Per-segment references
        self.ref_x = None
        self.ref_y = None
        self.ref_yaw = None

        # Shutdown control
        self.done = False
        self.finish_time = None

        # Run the control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.loop)

    def odom_cb(self, msg):
        # Current position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Save first odometry reading as the starting point
        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.get_logger().info('Captured starting position')

        self.current_x = x
        self.current_y = y
        self.current_yaw = yaw

    def publish(self, x=0.0, z=0.0):
        msg = TwistStamped()
        msg.twist.linear.x = x
        msg.twist.angular.z = z
        self.pub.publish(msg)

    def angle_diff(self, a, b):
        """
        Return the wrapped angle difference a - b in radians,
        limited to [-pi, pi].
        """
        d = a - b
        return math.atan2(math.sin(d), math.cos(d))

    def stop_and_exit(self, final_error):
        # Stop the robot
        self.publish()

        # Write result to JSON/CSV
        append_result(
            test_name='square_test',
            status='PASS',
            measurement=f'{final_error:.3f} m',
            notes='loop closure error'
        )

        self.get_logger().info(
            f'Square test complete. Final position error: {final_error:.3f} m'
        )

        self.done = True
        self.finish_time = time.time()

    def loop(self):
        # After finishing, wait briefly, then exit cleanly
        if self.done:
            if time.time() - self.finish_time > 0.5:
                self.get_logger().info('Exiting square_test')
                rclpy.shutdown()
            return

        # Wait until odometry has been received
        if self.current_x is None or self.current_yaw is None:
            return

        # ===== FORWARD STATE =====
        if self.state == 'forward':
            if self.ref_x is None:
                self.ref_x = self.current_x
                self.ref_y = self.current_y
                self.get_logger().info(
                    f'Side {self.side_count + 1}: moving forward'
                )

            dist = math.sqrt(
                (self.current_x - self.ref_x) ** 2 +
                (self.current_y - self.ref_y) ** 2
            )

            if dist < SIDE_LENGTH:
                self.publish(x=LINEAR_SPEED)
            else:
                self.publish()
                self.get_logger().info(
                    f'Side {self.side_count + 1} complete. '
                    f'Distance traveled: {dist:.3f} m'
                )
                self.ref_x = None
                self.ref_y = None
                self.state = 'rotate'

        # ===== ROTATE STATE =====
        elif self.state == 'rotate':
            if self.ref_yaw is None:
                self.ref_yaw = self.current_yaw
                self.get_logger().info(
                    f'Corner {self.side_count + 1}: rotating 90 degrees'
                )

            turned = abs(self.angle_diff(self.current_yaw, self.ref_yaw))
            remaining = TARGET_TURN - turned

            if remaining > ANGLE_TOL:
                # Slow down as robot approaches target angle
                speed_scale = remaining / TARGET_TURN
                cmd_z = max(MIN_ANGULAR_SPEED, MAX_ANGULAR_SPEED * speed_scale)
                self.publish(z=cmd_z)
            else:
                self.publish()
                self.get_logger().info(
                    f'Corner {self.side_count + 1} complete. '
                    f'Rotation traveled: {math.degrees(turned):.1f} deg'
                )

                self.ref_yaw = None
                self.side_count += 1

                if self.side_count >= 4:
                    final_error = math.sqrt(
                        (self.current_x - self.start_x) ** 2 +
                        (self.current_y - self.start_y) ** 2
                    )
                    self.stop_and_exit(final_error)
                else:
                    self.state = 'forward'


def main(args=None):
    rclpy.init(args=args)
    node = SquareTest()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()