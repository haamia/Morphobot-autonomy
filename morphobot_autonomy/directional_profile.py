#!/usr/bin/env python3

import math
from typing import List

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


class DirectionalProfile(Node):

    def __init__(self):
        super().__init__('directional_profile')


        self.declare_parameter(
            'input_topic',
            '/filtered_scan'
        )

        self.declare_parameter(
            'output_topic',
            '/directional_clearance'
        )

        self.declare_parameter(
            'min_angle_deg',
            -90.0
        )

        self.declare_parameter(
            'max_angle_deg',
            90.0
        )

        self.declare_parameter(
            'bin_width_deg',
            5.0
        )

        self.input_topic = self.get_parameter(
            'input_topic'
        ).value

        self.output_topic = self.get_parameter(
            'output_topic'
        ).value

        self.min_angle_deg = float(
            self.get_parameter(
                'min_angle_deg'
            ).value
        )

        self.max_angle_deg = float(
            self.get_parameter(
                'max_angle_deg'
            ).value
        )

        self.bin_width_deg = float(
            self.get_parameter(
                'bin_width_deg'
            ).value
        )

        number_of_bins = int(
            round(
                (
                    self.max_angle_deg
                    - self.min_angle_deg
                )
                / self.bin_width_deg
            )
        ) + 1

        self.headings = [
            self.min_angle_deg
            + i * self.bin_width_deg
            for i in range(number_of_bins)
        ]


        self.scan_sub = self.create_subscription(
            LaserScan,
            self.input_topic,
            self.scan_callback,
            10
        )

        self.clearance_pub = self.create_publisher(
            Float32MultiArray,
            self.output_topic,
            10
        )

        self.get_logger().info(
            'Directional profile node started'
        )

        self.get_logger().info(
            f'Heading range: '
            f'{self.min_angle_deg:.1f}° '
            f'to '
            f'{self.max_angle_deg:.1f}°'
        )

        self.get_logger().info(
            f'Bin width: '
            f'{self.bin_width_deg:.1f}°'
        )

        self.get_logger().info(
            f'Number of headings: '
            f'{len(self.headings)}'
        )


    def scan_callback(
        self,
        msg: LaserScan
    ) -> None:


        clearances: List[float] = [
            msg.range_max
            for _ in self.headings
        ]


        for index, raw_range in enumerate(
            msg.ranges
        ):

            # Invalid measurement

            if math.isnan(raw_range):
                continue

            # No return

            if math.isinf(raw_range):
                raw_range = msg.range_max

            # Sensor limits

            if raw_range < msg.range_min:
                continue

            if raw_range > msg.range_max:
                raw_range = msg.range_max


            angle_rad = (
                msg.angle_min
                + index * msg.angle_increment
            )

            angle_deg = math.degrees(
                angle_rad
            )

            if (
                angle_deg < self.min_angle_deg
                or
                angle_deg > self.max_angle_deg
            ):
                continue


            bin_index = int(
                round(
                    (
                        angle_deg
                        - self.min_angle_deg
                    )
                    / self.bin_width_deg
                )
            )

            if (
                bin_index < 0
                or
                bin_index >= len(clearances)
            ):
                continue

          

            if raw_range < clearances[bin_index]:

                clearances[bin_index] = raw_range


        output = Float32MultiArray()

        output.data = clearances

        self.clearance_pub.publish(
            output
        )


        self.print_profile(
            clearances
        )


    def print_profile(
        self,
        clearances: List[float]
    ) -> None:

        # Print only the important forward range
        # in a compact form.

        values = []

        for angle, distance in zip(
            self.headings,
            clearances
        ):

            values.append(
                f'{angle:+.0f}°:{distance:.2f}'
            )

        self.get_logger().info(
            ' | '.join(values)
        )


def main(args=None):

    rclpy.init(args=args)

    node = DirectionalProfile()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()