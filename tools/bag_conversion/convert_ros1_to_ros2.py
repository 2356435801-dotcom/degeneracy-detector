#!/usr/bin/env python3
"""Convert a ROS1 LaserScan bag to a ROS2 SQLite3 bag without changing fields."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore


ROS1_TYPE = "sensor_msgs/msg/LaserScan"
ROS2_TYPE = "sensor_msgs/msg/LaserScan"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_bag", type=Path)
    parser.add_argument("output_bag", type=Path)
    return parser.parse_args()


def convert(input_bag: Path, output_bag: Path) -> int:
    if not input_bag.is_file():
        raise FileNotFoundError(input_bag)
    if output_bag.exists():
        raise FileExistsError(
            f"Refusing to overwrite existing output directory: {output_bag}"
        )

    ros1_store = get_typestore(Stores.ROS1_NOETIC)
    ros2_store = get_typestore(Stores.ROS2_HUMBLE)
    ros2_scan = ros2_store.types[ROS2_TYPE]
    ros2_header = ros2_store.types["std_msgs/msg/Header"]
    ros2_time = ros2_store.types["builtin_interfaces/msg/Time"]

    count = 0
    with Reader(input_bag) as reader:
        scan_connections = [
            connection
            for connection in reader.connections
            if connection.topic == "/scan"
        ]
        if len(scan_connections) != 1:
            raise RuntimeError(
                f"Expected exactly one /scan connection, got {len(scan_connections)}"
            )
        connection = scan_connections[0]
        if connection.msgtype != ROS1_TYPE:
            raise RuntimeError(
                f"Expected {ROS1_TYPE} on /scan, got {connection.msgtype}"
            )

        with Writer(output_bag, version=8) as writer:
            out_connection = writer.add_connection(
                "/scan", ROS2_TYPE, typestore=ros2_store
            )
            for current_connection, record_time, raw in reader.messages(
                connections=[connection]
            ):
                if current_connection != connection:
                    raise RuntimeError("Reader yielded an unexpected connection")
                source = ros1_store.deserialize_ros1(raw, ROS1_TYPE)
                stamp = source.header.stamp
                header = ros2_header(
                    stamp=ros2_time(sec=stamp.sec, nanosec=stamp.nanosec),
                    frame_id=source.header.frame_id,
                )
                target = ros2_scan(
                    header=header,
                    angle_min=source.angle_min,
                    angle_max=source.angle_max,
                    angle_increment=source.angle_increment,
                    time_increment=source.time_increment,
                    scan_time=source.scan_time,
                    range_min=source.range_min,
                    range_max=source.range_max,
                    ranges=np.asarray(source.ranges, dtype=np.float32),
                    intensities=np.asarray(source.intensities, dtype=np.float32),
                )
                serialized = ros2_store.serialize_cdr(target, ROS2_TYPE)
                writer.write(out_connection, record_time, serialized)
                count += 1

    print(f"converted_messages={count}")
    print(f"output_bag={output_bag}")
    return count


if __name__ == "__main__":
    arguments = parse_args()
    convert(arguments.input_bag, arguments.output_bag)
