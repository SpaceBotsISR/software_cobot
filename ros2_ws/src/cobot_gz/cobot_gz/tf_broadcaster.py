#!/usr/bin/env python3
"""Publish TF frames for the Space Cobot model."""

from __future__ import annotations

import math
from typing import List

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def make_static_transform(
    parent: str,
    child: str,
    xyz: List[float] | tuple[float, float, float],
    rpy: List[float] | tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> TransformStamped:
    """Utility to create a zero-rate transform from XYZ + RPY."""
    transform = TransformStamped()
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.translation.x = float(xyz[0])
    transform.transform.translation.y = float(xyz[1])
    transform.transform.translation.z = float(xyz[2])

    # Convert roll / pitch / yaw to quaternion.
    roll, pitch, yaw = rpy
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    transform.transform.rotation.w = cr * cp * cy + sr * sp * sy
    transform.transform.rotation.x = sr * cp * cy - cr * sp * sy
    transform.transform.rotation.y = cr * sp * cy + sr * cp * sy
    transform.transform.rotation.z = cr * cp * sy - sr * sp * cy
    return transform


class SpaceCobotTfPublisher(Node):
    """Broadcast dynamic and static transforms for Space Cobot."""

    def __init__(self) -> None:
        super().__init__("space_cobot_tf_broadcaster")

        self.declare_parameter("world_frame", "world")
        self.declare_parameter("base_frame", "body")
        self.declare_parameter("camera_frame", "space_cobot/depth_cam_link")
        self.declare_parameter(
            "depth_sensor_frame", "space_cobot/depth_cam_link/depth_camera"
        )
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("tf_publish_rate_hz", 100.0)

        self._world_frame = (
            self.get_parameter("world_frame").get_parameter_value().string_value
            or "world"
        )
        self._base_frame = (
            self.get_parameter("base_frame").get_parameter_value().string_value
            or "body"
        )
        self._camera_frame = (
            self.get_parameter("camera_frame").get_parameter_value().string_value
            or "space_cobot/depth_cam_link"
        )
        self._depth_sensor_frame = (
            self.get_parameter("depth_sensor_frame").get_parameter_value().string_value
            or "space_cobot/depth_cam_link/depth_camera"
        )
        self._map_frame = (
            self.get_parameter("map_frame").get_parameter_value().string_value or ""
        )
        tf_rate = (
            self.get_parameter("tf_publish_rate_hz")
            .get_parameter_value()
            .double_value
        )
        self._tf_publish_period = (
            Duration(seconds=1.0 / tf_rate) if tf_rate > 0.0 else None
        )
        self._last_tf_publish_time = None

        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_transforms()

        self._pose_publisher = self.create_publisher(
            PoseStamped, "/space_cobot/pose", 10
        )
        self.create_subscription(PoseStamped, "/cobot_gz/poses", self._relay_pose, 10)
        self.create_subscription(
            PoseStamped, "/space_cobot/pose", self._handle_pose, 10
        )

        self.get_logger().info(
            f"Space Cobot TF broadcaster active. Dynamic frame {self._world_frame} -> {self._base_frame}. "
            "Static frames registered for camera."
        )

    def _publish_static_transforms(self) -> None:
        transforms = [
            make_static_transform(
                self._base_frame, self._camera_frame, (0.13, 0.0, 0.13)
            ),
            make_static_transform(
                self._camera_frame, self._depth_sensor_frame, (0.0, 0.0, 0.0)
            ),
        ]
        if self._map_frame and self._map_frame != self._world_frame:
            transforms.append(
                make_static_transform(
                    self._map_frame, self._world_frame, (0.0, 0.0, 0.0)
                )
            )
        now = self.get_clock().now().to_msg()
        for transform in transforms:
            transform.header.stamp = now
        self._static_broadcaster.sendTransform(transforms)

    def _relay_pose(self, msg: PoseStamped) -> None:
        if msg.header.frame_id == "default":
            self._pose_publisher.publish(msg)

    def _handle_pose(self, msg: PoseStamped) -> None:
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        if transform.header.stamp.sec == 0 and transform.header.stamp.nanosec == 0:
            transform.header.stamp = self.get_clock().now().to_msg()
        publish_time = self.get_clock().now()
        if self._tf_publish_period is not None:
            if (
                self._last_tf_publish_time is not None
                and (publish_time - self._last_tf_publish_time)
                < self._tf_publish_period
            ):
                return
            self._last_tf_publish_time = publish_time
        transform.header.frame_id = self._world_frame
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = msg.pose.position.x
        transform.transform.translation.y = msg.pose.position.y
        transform.transform.translation.z = msg.pose.position.z
        transform.transform.rotation.x = msg.pose.orientation.x
        transform.transform.rotation.y = msg.pose.orientation.y
        transform.transform.rotation.z = msg.pose.orientation.z
        transform.transform.rotation.w = msg.pose.orientation.w
        self._tf_broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = SpaceCobotTfPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
