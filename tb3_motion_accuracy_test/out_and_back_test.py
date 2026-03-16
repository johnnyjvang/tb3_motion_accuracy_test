"""
out_and_back_test.py

Drive forward 1 meter, rotate 180 degrees, then drive forward 1 meter
back toward the starting point. After stopping, report the final
position error relative to the start.
"""

import time
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from tb3_motion_accuracy_test.result_utils import append_result

# Test settings
TARGET_DISTANCE = 1.0
TARGET_ROTATION_DEG = 180.0
ROTATION_STOP_TOLERANCE_DEG = 0.5
LINEAR_SPEED = 0.10
ANGULAR_SPEED_FAST = 0.30
ANGULAR_SPEED_SLOW = 0.10


def normalize_angle(angle):
    # Keep angle in [-pi, pi] to avoid wraparound issues
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class OutAndBackTest(Node):
    def __init__(self):
        super().__init__('out_and_back_test')

        # Publisher for robot velocity commands
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Subscriber for odometry feedback
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # Start and current position tracking
        self.start_x = None
        self.start_y = None
        self.current_x = None
        self.current_y = None

        # Start and current yaw tracking
        self.start_yaw = None
        self.current_yaw = None

        # Track pose at the start of each phase
        self.phase_start_x = None
        self.phase_start_y = None
        self.phase_start_yaw = None

        # Measurements for final report
        self.forward_distance = 0.0
        self.rotation_angle = 0.0
        self.return_distance = 0.0

        # Test phase state
        self.phase = 'forward_1'

        # Logging control so console does not spam too hard
        self.last_progress_log_time = 0.0

        # Shutdown control
        self.done = False
        self.finish_time = None

        # Run the control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.loop)

    def odom_cb(self, msg):
        # Read current odometry position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Convert quaternion orientation into yaw
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        # Save the first odometry reading as the global start point
        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.start_yaw = yaw
            self.get_logger().info('Captured starting pose')

            # Initialize the first phase start pose
            self.phase_start_x = x
            self.phase_start_y = y
            self.phase_start_yaw = yaw

        self.current_x = x
        self.current_y = y
        self.current_yaw = yaw

    def publish(self, x=0.0, z=0.0):
        # Publish a TwistStamped command
        msg = TwistStamped()
        msg.twist.linear.x = x
        msg.twist.angular.z = z
        self.pub.publish(msg)

    def set_new_phase(self, new_phase):
        # Save the current pose as the start of the next phase
        self.phase = new_phase
        self.phase_start_x = self.current_x
        self.phase_start_y = self.current_y
        self.phase_start_yaw = self.current_yaw
        self.last_progress_log_time = 0.0
        self.get_logger().info(f'Starting phase: {new_phase}')

    def stop_and_exit(self, message, final_error):
        # Stop the robot and mark node ready for shutdown
        self.publish()

        # Print full motion summary
        self.get_logger().info('----- Motion Accuracy Summary -----')
        self.get_logger().info(f'Forward distance traveled: {self.forward_distance:.3f} m')
        self.get_logger().info(f'Rotation traveled: {self.rotation_angle:.1f} deg')
        self.get_logger().info(f'Return distance traveled: {self.return_distance:.3f} m')
        self.get_logger().info(f'Final closure error: {final_error:.3f} m')

        # Write test result
        append_result(
            test_name='out_and_back_test',
            status='PASS',
            measurement=f'{final_error:.3f} m',
            notes='closure error'
        )

        self.get_logger().info(message)
        self.done = True
        self.finish_time = time.time()

    def maybe_log_progress(self, message):
        # Log progress about once per second
        now = time.time()
        if now - self.last_progress_log_time > 1.0:
            self.get_logger().info(message)
            self.last_progress_log_time = now

    def loop(self):
        # After finishing, wait briefly, then exit cleanly
        if self.done:
            if time.time() - self.finish_time > 0.5:
                self.get_logger().info('Exiting out_and_back_test')
                rclpy.shutdown()
            return

        # Wait until odometry has been received
        if self.start_x is None or self.current_x is None or self.current_yaw is None:
            return

        # Phase 1: move forward 1 meter
        if self.phase == 'forward_1':
            dist = math.sqrt(
                (self.current_x - self.phase_start_x) ** 2 +
                (self.current_y - self.phase_start_y) ** 2
            )

            self.maybe_log_progress(
                f'Forward leg distance: {dist:.3f} m / {TARGET_DISTANCE:.3f} m'
            )

            if dist < TARGET_DISTANCE:
                self.publish(x=LINEAR_SPEED)
            else:
                self.publish()
                self.forward_distance = dist
                self.get_logger().info(
                    f'Forward leg complete. Distance traveled: {dist:.3f} m'
                )
                self.set_new_phase('rotate_180')

        # Phase 2: rotate 180 degrees
        elif self.phase == 'rotate_180':
            delta_yaw = normalize_angle(self.current_yaw - self.phase_start_yaw)
            delta_deg = math.degrees(delta_yaw)
            abs_delta_deg = abs(delta_deg)

            remaining_error = TARGET_ROTATION_DEG - abs_delta_deg

            self.maybe_log_progress(
                f'Rotation progress: {abs_delta_deg:.1f} deg / {TARGET_ROTATION_DEG:.1f} deg'
            )

            if remaining_error > 20.0:
                self.publish(z=ANGULAR_SPEED_FAST)
            elif remaining_error > ROTATION_STOP_TOLERANCE_DEG:
                self.publish(z=ANGULAR_SPEED_SLOW)
            else:
                self.publish()
                self.rotation_angle = abs_delta_deg
                self.get_logger().info(
                    f'Rotation complete. Rotated: {abs_delta_deg:.1f} deg'
                )
                self.set_new_phase('forward_2')

        # Phase 3: move forward 1 meter back toward the start
        elif self.phase == 'forward_2':
            dist = math.sqrt(
                (self.current_x - self.phase_start_x) ** 2 +
                (self.current_y - self.phase_start_y) ** 2
            )

            self.maybe_log_progress(
                f'Return leg distance: {dist:.3f} m / {TARGET_DISTANCE:.3f} m'
            )

            if dist < TARGET_DISTANCE:
                self.publish(x=LINEAR_SPEED)
            else:
                self.publish()
                self.return_distance = dist
                self.get_logger().info(
                    f'Return leg complete. Distance traveled: {dist:.3f} m'
                )

                # Compute final position error from the original start point
                final_error = math.sqrt(
                    (self.current_x - self.start_x) ** 2 +
                    (self.current_y - self.start_y) ** 2
                )

                self.stop_and_exit(
                    f'Out-and-back complete. Final position error: {final_error:.3f} m',
                    final_error
                )


def main(args=None):
    rclpy.init(args=args)
    node = OutAndBackTest()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()