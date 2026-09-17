#!/usr/bin/env python3

import math
import signal
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class TwistToWheels(Node):

    def __init__(self):

        super().__init__('twist_to_wheels')

        # =====================================================
        # TOPICS
        # =====================================================

        self.declare_parameter(
            'cmd_vel_topic',
            '/cmd_vel'
        )

        self.declare_parameter(
            'wheel_command_topic',
            '/wheel_velocity_controller/commands'
        )

        # =====================================================
        # ROBOT GEOMETRY
        # =====================================================

        self.declare_parameter(
            'wheel_radius_m',
            0.06
        )

        self.declare_parameter(
            'track_width_m',
            0.335
        )

        # =====================================================
        # WHEEL COMMAND LIMIT
        # =====================================================

        self.declare_parameter(
            'max_wheel_speed_radps',
            10.0
        )

        # =====================================================
        # WHEEL SIGN CALIBRATION
        # =====================================================

        self.declare_parameter(
            'left_wheel_sign',
            1.0
        )

        self.declare_parameter(
            'right_wheel_sign',
            1.0
        )

        # =====================================================
        # CMD_VEL WATCHDOG
        # =====================================================

        self.declare_parameter(
            'cmd_vel_timeout',
            0.5
        )

        self.declare_parameter(
            'watchdog_rate',
            20.0
        )

        # =====================================================
        # SHUTDOWN SETTINGS
        # =====================================================

        self.declare_parameter(
            'shutdown_stop_count',
            20
        )

        self.declare_parameter(
            'shutdown_stop_interval',
            0.05
        )

        # =====================================================
        # READ PARAMETERS
        # =====================================================

        self.cmd_vel_topic = str(
            self.get_parameter(
                'cmd_vel_topic'
            ).value
        )

        self.wheel_command_topic = str(
            self.get_parameter(
                'wheel_command_topic'
            ).value
        )

        self.wheel_radius = float(
            self.get_parameter(
                'wheel_radius_m'
            ).value
        )

        self.track_width = float(
            self.get_parameter(
                'track_width_m'
            ).value
        )

        self.max_wheel_speed = float(
            self.get_parameter(
                'max_wheel_speed_radps'
            ).value
        )

        self.left_sign = float(
            self.get_parameter(
                'left_wheel_sign'
            ).value
        )

        self.right_sign = float(
            self.get_parameter(
                'right_wheel_sign'
            ).value
        )

        self.cmd_vel_timeout = float(
            self.get_parameter(
                'cmd_vel_timeout'
            ).value
        )

        self.watchdog_rate = float(
            self.get_parameter(
                'watchdog_rate'
            ).value
        )

        self.shutdown_stop_count = int(
            self.get_parameter(
                'shutdown_stop_count'
            ).value
        )

        self.shutdown_stop_interval = float(
            self.get_parameter(
                'shutdown_stop_interval'
            ).value
        )

        # =====================================================
        # VALIDATION
        # =====================================================

        if self.wheel_radius <= 0.0:
            self.wheel_radius = 0.06

        if self.track_width <= 0.0:
            self.track_width = 0.335

        if self.max_wheel_speed <= 0.0:
            self.max_wheel_speed = 10.0

        if self.cmd_vel_timeout <= 0.0:
            self.cmd_vel_timeout = 0.5

        if self.watchdog_rate <= 0.0:
            self.watchdog_rate = 20.0

        if self.shutdown_stop_count < 1:
            self.shutdown_stop_count = 20

        if self.shutdown_stop_interval < 0.0:
            self.shutdown_stop_interval = 0.05

        # Only ±1 is meaningful.

        self.left_sign = (
            1.0
            if self.left_sign >= 0.0
            else -1.0
        )

        self.right_sign = (
            1.0
            if self.right_sign >= 0.0
            else -1.0
        )

        # =====================================================
        # ROS INTERFACES
        # =====================================================

        self.subscriber = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )

        self.publisher = self.create_publisher(
            Float64MultiArray,
            self.wheel_command_topic,
            10
        )

        # =====================================================
        # STATE
        # =====================================================

        self.last_cmd_time = self.get_clock().now()

        self.shutdown_requested = False

        self.watchdog_stop_active = False

        # =====================================================
        # WATCHDOG
        # =====================================================

        watchdog_period = 1.0 / self.watchdog_rate

        self.watchdog_timer = self.create_timer(
            watchdog_period,
            self.watchdog_callback
        )

        # =====================================================
        # STARTUP
        # =====================================================

        self.get_logger().info(
            'Twist-to-wheels bridge started'
        )

        self.get_logger().info(
            f'Input: {self.cmd_vel_topic}'
        )

        self.get_logger().info(
            f'Output: {self.wheel_command_topic}'
        )

        self.get_logger().info(
            f'Wheel radius: {self.wheel_radius:.3f} m'
        )

        self.get_logger().info(
            f'Track width: {self.track_width:.3f} m'
        )

        self.get_logger().info(
            f'Wheel speed limit: '
            f'{self.max_wheel_speed:.2f} rad/s'
        )

        self.get_logger().info(
            'Wheel order: [FL, FR, RL, RR]'
        )

        self.get_logger().info(
            f'cmd_vel timeout: '
            f'{self.cmd_vel_timeout:.2f} s'
        )

    # =========================================================
    # CMD_VEL CALLBACK
    # =========================================================

    def cmd_vel_callback(
        self,
        msg: Twist
    ) -> None:

        if self.shutdown_requested:
            return

        v = float(msg.linear.x)

        omega = float(msg.angular.z)

        self.last_cmd_time = self.get_clock().now()

        if self.watchdog_stop_active:

            self.watchdog_stop_active = False

            self.get_logger().info(
                'cmd_vel restored -> wheel control resumed'
            )

        # =====================================================
        # DIFFERENTIAL DRIVE
        # =====================================================

        left_linear_velocity = (
            v -
            omega * self.track_width / 2.0
        )

        right_linear_velocity = (
            v +
            omega * self.track_width / 2.0
        )

        # =====================================================
        # LINEAR -> ANGULAR
        # =====================================================

        left_wheel_speed = (
            left_linear_velocity /
            self.wheel_radius
        )

        right_wheel_speed = (
            right_linear_velocity /
            self.wheel_radius
        )

        # =====================================================
        # APPLY SIGNS
        # =====================================================

        front_left = (
            self.left_sign *
            left_wheel_speed
        )

        rear_left = (
            self.left_sign *
            left_wheel_speed
        )

        front_right = (
            self.right_sign *
            right_wheel_speed
        )

        rear_right = (
            self.right_sign *
            right_wheel_speed
        )

        # =====================================================
        # LIMIT
        # =====================================================

        front_left = self.clamp(
            front_left,
            -self.max_wheel_speed,
            self.max_wheel_speed
        )

        front_right = self.clamp(
            front_right,
            -self.max_wheel_speed,
            self.max_wheel_speed
        )

        rear_left = self.clamp(
            rear_left,
            -self.max_wheel_speed,
            self.max_wheel_speed
        )

        rear_right = self.clamp(
            rear_right,
            -self.max_wheel_speed,
            self.max_wheel_speed
        )

        # =====================================================
        # PUBLISH
        # =====================================================

        self.publish_wheel_commands(
            front_left,
            front_right,
            rear_left,
            rear_right
        )

        # =====================================================
        # DEBUG
        # =====================================================

        self.get_logger().info(
            f'v={v:+.3f} m/s | '
            f'w={omega:+.3f} rad/s | '
            f'FL={front_left:+.3f} | '
            f'FR={front_right:+.3f} | '
            f'RL={rear_left:+.3f} | '
            f'RR={rear_right:+.3f}'
        )

    # =========================================================
    # WATCHDOG
    # =========================================================

    def watchdog_callback(self) -> None:

        if self.shutdown_requested:
            return

        now = self.get_clock().now()

        elapsed = (
            now - self.last_cmd_time
        ).nanoseconds / 1e9

        if elapsed > self.cmd_vel_timeout:

            self.publish_zero_wheels()

            if not self.watchdog_stop_active:

                self.watchdog_stop_active = True

                self.get_logger().warn(
                    f'No /cmd_vel for '
                    f'{elapsed:.2f} s -> STOPPING WHEELS'
                )

    # =========================================================
    # PUBLISH WHEEL COMMANDS
    # =========================================================

    def publish_wheel_commands(
        self,
        front_left,
        front_right,
        rear_left,
        rear_right
    ) -> None:

        if not rclpy.ok():
            return

        output = Float64MultiArray()

        output.data = [
            float(front_left),
            float(front_right),
            float(rear_left),
            float(rear_right)
        ]

        self.publisher.publish(output)

    # =========================================================
    # PUBLISH ZERO
    # =========================================================

    def publish_zero_wheels(self) -> None:

        self.publish_wheel_commands(
            0.0,
            0.0,
            0.0,
            0.0
        )

    # =========================================================
    # SAFE SHUTDOWN
    # =========================================================

    def safe_shutdown(self) -> None:

        if self.shutdown_requested:
            return

        self.shutdown_requested = True

        # Stop watchdog from doing normal control.

        try:
            self.watchdog_timer.cancel()
        except Exception:
            pass

        # -----------------------------------------------------
        # SEND ZERO COMMANDS
        # -----------------------------------------------------

        print(
            '\n[SAFETY] Sending ZERO wheel commands...'
        )

        for _ in range(
            self.shutdown_stop_count
        ):

            try:

                self.publish_zero_wheels()

                # Give DDS time to transmit.

                time.sleep(
                    self.shutdown_stop_interval
                )

            except Exception as exc:

                print(
                    f'[SAFETY] Stop publish error: {exc}'
                )

                break

        print(
            '[SAFETY] Wheel stop commands sent.'
        )

    # =========================================================
    # CLAMP
    # =========================================================

    @staticmethod
    def clamp(
        value,
        minimum,
        maximum
    ):

        return max(
            minimum,
            min(
                maximum,
                value
            )
        )


# =============================================================
# GLOBAL SIGNAL HANDLER
# =============================================================

node_instance = None


def signal_handler(
    signum,
    frame
):

    global node_instance

    print(
        '\n[SIGNAL] Ctrl+C received.'
    )

    if node_instance is not None:

        node_instance.safe_shutdown()

    # Now shut down ROS.

    if rclpy.ok():

        rclpy.shutdown()


# =============================================================
# MAIN
# =============================================================

def main(args=None):

    global node_instance

    rclpy.init(args=args)

    node_instance = TwistToWheels()

    # ---------------------------------------------------------
    # INSTALL SIGINT HANDLER
    # ---------------------------------------------------------

    signal.signal(
        signal.SIGINT,
        signal_handler
    )

    signal.signal(
        signal.SIGTERM,
        signal_handler
    )

    try:

        rclpy.spin(
            node_instance
        )

    except KeyboardInterrupt:

        # Signal handler normally handles this.

        pass

    finally:

        if node_instance is not None:

            if not node_instance.shutdown_requested:

                node_instance.safe_shutdown()

            try:
                node_instance.destroy_node()
            except Exception:
                pass

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()