#!/usr/bin/env python3

import math
from typing import Dict, List

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


class LidarProcessor(Node):

    def __init__(self):
        super().__init__('lidar_processor')

        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter(
            'output_topic',
            '/environment_state'
        )
        self.declare_parameter(
            'filtered_scan_topic',
            '/filtered_scan'
        )

        self.input_topic = self.get_parameter(
            'input_topic'
        ).value

        self.output_topic = self.get_parameter(
            'output_topic'
        ).value

        self.filtered_scan_topic = self.get_parameter(
            'filtered_scan_topic'
        ).value

        self.robot_x_min = -0.45
        self.robot_x_max = 0.45

        self.robot_y_min = -0.45
        self.robot_y_max = 0.45


        self.sectors = {
            'front': (-15.0, 15.0),

            'front_left': (15.0, 45.0),

            'front_right': (-45.0, -15.0),

            'left': (45.0, 100.0),

            'right': (-100.0, -45.0),

            'rear': (100.0, 180.0),
        }

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.input_topic,
            self.scan_callback,
            10
        )


        self.state_pub = self.create_publisher(
            Float32MultiArray,
            self.output_topic,
            10
        )

        
        self.filtered_scan_pub = self.create_publisher(
            LaserScan,
            self.filtered_scan_topic,
            10
        )

        self.get_logger().info(
            'LiDAR processor started'
        )


    def scan_callback(
        self,
        msg: LaserScan
    ) -> None:


        filtered_scan = LaserScan()

        filtered_scan.header = msg.header

        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment

        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time

        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max

        filtered_ranges = list(msg.ranges)


        for index, raw_range in enumerate(msg.ranges):

            # Invalid measurements stay invalid.

            if math.isnan(raw_range):
                filtered_ranges[index] = float('nan')
                continue

            # Infinity means no obstacle within sensor range.

            if math.isinf(raw_range):
                continue

            # Ignore measurements outside sensor limits.

            if (
                raw_range < msg.range_min
                or raw_range > msg.range_max
            ):
                filtered_ranges[index] = float('nan')
                continue


            angle = (
                msg.angle_min
                + index * msg.angle_increment
            )

            x = raw_range * math.cos(angle)
            y = raw_range * math.sin(angle)

            if self.is_inside_robot(x, y):

                filtered_ranges[index] = float('nan')

        filtered_scan.ranges = filtered_ranges
        filtered_scan.intensities = list(msg.intensities)


        self.filtered_scan_pub.publish(
            filtered_scan
        )


        sector_distances: Dict[str, float] = {}

        for sector_name, (
            start_deg,
            end_deg
        ) in self.sectors.items():

            distance = self.get_sector_min_distance(
                msg,
                start_deg,
                end_deg
            )

            sector_distances[
                sector_name
            ] = distance

        output = Float32MultiArray()

        output.data = [
            sector_distances['front'],
            sector_distances['front_left'],
            sector_distances['front_right'],
            sector_distances['left'],
            sector_distances['right'],
            sector_distances['rear'],
        ]

        self.state_pub.publish(output)


        self.get_logger().info(
            f"F={sector_distances['front']:.2f} m | "
            f"FL={sector_distances['front_left']:.2f} m | "
            f"FR={sector_distances['front_right']:.2f} m | "
            f"L={sector_distances['left']:.2f} m | "
            f"R={sector_distances['right']:.2f} m | "
            f"Rear={sector_distances['rear']:.2f} m"
        )


    def get_sector_min_distance(
        self,
        msg: LaserScan,
        start_deg: float,
        end_deg: float
    ) -> float:

        valid_ranges: List[float] = []

        start_rad = math.radians(start_deg)
        end_rad = math.radians(end_deg)

        for index, raw_range in enumerate(msg.ranges):

            angle = (
                msg.angle_min
                + index * msg.angle_increment
            )

            if angle < start_rad or angle > end_rad:
                continue

            if math.isnan(raw_range):
                continue

            if math.isinf(raw_range):
                raw_range = msg.range_max

            if raw_range < msg.range_min:
                continue

            if raw_range > msg.range_max:
                raw_range = msg.range_max

            x = raw_range * math.cos(angle)
            y = raw_range * math.sin(angle)

            if self.is_inside_robot(x, y):
                continue

            valid_ranges.append(
                raw_range
            )

        if not valid_ranges:
            return msg.range_max

        return min(valid_ranges)

    def is_inside_robot(
        self,
        x: float,
        y: float
    ) -> bool:

        return (
            self.robot_x_min <= x <= self.robot_x_max
            and
            self.robot_y_min <= y <= self.robot_y_max
        )


def main(args=None):

    rclpy.init(args=args)

    node = LidarProcessor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()