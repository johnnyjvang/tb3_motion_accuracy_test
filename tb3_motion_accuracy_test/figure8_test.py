"""
figure8_test.py

Drive the robot in a figure-8 path:
- First loop: circle in one direction
- Second loop: circle in the opposite direction

Goal:
- Complete a full figure 8
- Measure how close the robot returns to its starting position
"""

import time
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from tb3_motion_accuracy_test.result_utils import append_result


# ===== Test Settings =====
LINEAR_SPEED = 0.15         # m/s
ANGULAR_SPEED = 0.30        # rad/s magnitude

TARGET_LOOP_ANGLE = 2.0 * math.pi   # one full loop
ANGLE_TOL = math.radians(2.0)       # tolerance


class Figure8Test(Node):
    def __init__(self):
        super().__init__('figure8_test')

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
        self.prev_yaw = None

        # Figure-8 state
        self.phase = 'left_loop'   # left_loop -> right_loop -> done
        self.phase_rotation = 0.0

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

        # Track accumulated rotation for current phase
        if self.prev_yaw is not None:
            delta = self.angle_diff(yaw, self.prev_yaw)
            self.phase_rotation += abs(delta)

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
        """
        Return wrapped angle difference a - b in radians,
        limited to [-pi, pi].
        """
        d = a - b
        return math.atan2(math.sin(d), math.cos(d))

    def start_next_phase(self, new_phase):
        self.phase = new_phase
        self.phase_rotation = 0.0
        self.prev_yaw = self.current_yaw

    def stop_and_exit(self, final_error):
        # Stop the robot
        self.publish()

        append_result(
            test_name='figure8_test',
            status='PASS',
            measurement=f'{final_error:.3f} m',
            notes='loop closure error after figure 8'
        )

        self.get_logger().info(
            f'Figure 8 complete. Final position error: {final_error:.3f} m'
        )

        self.done = True
        self.finish_time = time.time()

    def loop(self):
        # After finishing, wait briefly, then exit cleanly
        if self.done:
            if time.time() - self.finish_time > 0.5:
                self.get_logger().info('Exiting figure8_test')
                rclpy.shutdown()
            return

        # Wait until odometry has been received
        if self.current_x is None or self.current_yaw is None:
            return

        # ===== LEFT LOOP =====
        if self.phase == 'left_loop':
            if self.phase_rotation == 0.0:
                self.get_logger().info('Starting left loop')

            if self.phase_rotation < TARGET_LOOP_ANGLE - ANGLE_TOL:
                self.publish(x=LINEAR_SPEED, z=ANGULAR_SPEED)
            else:
                self.publish()
                self.get_logger().info(
                    f'Left loop complete. Rotation traveled: '
                    f'{math.degrees(self.phase_rotation):.1f} deg'
                )
                self.start_next_phase('right_loop')

        # ===== RIGHT LOOP =====
        elif self.phase == 'right_loop':
            if self.phase_rotation == 0.0:
                self.get_logger().info('Starting right loop')

            if self.phase_rotation < TARGET_LOOP_ANGLE - ANGLE_TOL:
                self.publish(x=LINEAR_SPEED, z=-ANGULAR_SPEED)
            else:
                self.publish()
                self.get_logger().info(
                    f'Right loop complete. Rotation traveled: '
                    f'{math.degrees(self.phase_rotation):.1f} deg'
                )

                final_error = math.sqrt(
                    (self.current_x - self.start_x) ** 2 +
                    (self.current_y - self.start_y) ** 2
                )
                self.stop_and_exit(final_error)


def main(args=None):
    rclpy.init(args=args)
    node = Figure8Test()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()