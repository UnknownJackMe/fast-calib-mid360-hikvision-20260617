#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


HOLES = [
    ("hole +Y", (3.57454, 0.07096, 0.11873), (1.0, 0.0, 0.0, 1.0)),
    ("hole -Y", (3.55390, -0.43180, 0.14098), (0.0, 0.25, 1.0, 1.0)),
]


class HoleMarkerPublisher(Node):
    def __init__(self):
        super().__init__("publish_detected_holes")
        self.publisher = self.create_publisher(MarkerArray, "/detected_holes", 1)
        self.timer = self.create_timer(0.2, self.publish_markers)

    def publish_markers(self):
        msg = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for i, (label, position, color) in enumerate(HOLES):
            sphere = Marker()
            sphere.header.frame_id = "livox_frame"
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
            text.header.frame_id = "livox_frame"
            text.header.stamp = stamp
            text.ns = "detected_hole_labels"
            text.id = 100 + i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = position[0]
            text.pose.position.y = position[1]
            text.pose.position.z = position[2] + 0.22
            text.pose.orientation.w = 1.0
            text.scale.z = 0.18
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = label
            msg.markers.append(text)
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = HoleMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
