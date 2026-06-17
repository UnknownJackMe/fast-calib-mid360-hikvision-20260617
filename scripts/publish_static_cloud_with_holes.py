#!/usr/bin/env python3
import argparse
import struct
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


HOLES = [
    ("upper +Y", (3.57454, 0.07096, 0.11873), (1.0, 0.0, 0.0, 1.0)),
    ("upper -Y", (3.55390, -0.43180, 0.14098), (0.0, 0.2, 1.0, 1.0)),
    ("lower +Y", (3.54801, 0.06100, -0.27500), (0.0, 0.85, 0.2, 1.0)),
    ("lower -Y", (3.52758, -0.45020, -0.25500), (1.0, 0.85, 0.0, 1.0)),
]


def read_ascii_ply_xyz(path):
    vertex_count = None
    points = []

    with path.open("r", encoding="ascii") as f:
        for line in f:
            stripped = line.strip()
            if stripped.startswith("element vertex "):
                vertex_count = int(stripped.split()[-1])
            if stripped == "end_header":
                break

        if vertex_count is None:
            raise ValueError(f"{path} does not declare an element vertex count")

        for _ in range(vertex_count):
            line = f.readline()
            if not line:
                break
            values = line.split()
            if len(values) < 3:
                continue
            points.append((float(values[0]), float(values[1]), float(values[2])))

    return points


def make_cloud(points, frame_id):
    header = Header()
    header.frame_id = frame_id

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    data = bytearray(12 * len(points))
    for i, point in enumerate(points):
        struct.pack_into("<fff", data, i * 12, point[0], point[1], point[2])

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(points)
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * len(points)
    msg.data = bytes(data)
    msg.is_dense = True
    return msg


class StaticCloudWithHolesPublisher(Node):
    def __init__(self, cloud_path, frame_id, rate_hz):
        super().__init__("publish_static_cloud_with_holes")
        points = read_ascii_ply_xyz(cloud_path)
        self.cloud_msg = make_cloud(points, frame_id)
        self.frame_id = frame_id

        self.cloud_pub = self.create_publisher(PointCloud2, "/static_accumulated_cloud", 1)
        self.marker_pub = self.create_publisher(MarkerArray, "/detected_holes", 1)
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_all)
        self.get_logger().info(
            f"Loaded {len(points)} points from {cloud_path}; publishing static cloud at {rate_hz:.2f} Hz"
        )

    def publish_all(self):
        stamp = self.get_clock().now().to_msg()
        self.cloud_msg.header.stamp = stamp
        self.cloud_pub.publish(self.cloud_msg)
        self.marker_pub.publish(self.make_markers(stamp))

    def make_markers(self, stamp):
        msg = MarkerArray()
        for i, (label, position, color) in enumerate(HOLES):
            sphere = Marker()
            sphere.header.frame_id = self.frame_id
            sphere.header.stamp = stamp
            sphere.ns = "detected_holes"
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = position[0]
            sphere.pose.position.y = position[1]
            sphere.pose.position.z = position[2]
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.35
            sphere.scale.y = 0.35
            sphere.scale.z = 0.35
            sphere.color.r = color[0]
            sphere.color.g = color[1]
            sphere.color.b = color[2]
            sphere.color.a = color[3]
            msg.markers.append(sphere)

            text = Marker()
            text.header.frame_id = self.frame_id
            text.header.stamp = stamp
            text.ns = "detected_hole_labels"
            text.id = 100 + i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = position[0]
            text.pose.position.y = position[1]
            text.pose.position.z = position[2] + 0.25
            text.pose.orientation.w = 1.0
            text.scale.z = 0.18
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = label
            msg.markers.append(text)

        return msg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--cloud",
        default="output/final_success_20260617/filtered_cloud.ply",
        help="ASCII PLY file to publish as a static accumulated cloud",
    )
    parser.add_argument("--frame-id", default="livox_frame")
    parser.add_argument("--rate", type=float, default=0.2)
    args = parser.parse_args()

    cloud_path = Path(args.cloud).expanduser().resolve()
    if args.rate <= 0:
        raise ValueError("--rate must be positive")

    rclpy.init()
    node = StaticCloudWithHolesPublisher(cloud_path, args.frame_id, args.rate)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
