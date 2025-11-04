#!/usr/bin/env python3
"""Bridge body-frame wrench and twist commands into Gazebo-friendly topics."""

from __future__ import annotations

from copy import deepcopy

import rclpy
from geometry_msgs.msg import Twist, Vector3Stamped, Wrench
from rclpy.node import Node
from rclpy.time import Time
from ros_gz_interfaces.msg import Entity, EntityWrench
from tf2_geometry_msgs import do_transform_vector3
from tf2_ros import Buffer, TransformException, TransformListener


class CmdBridge(Node):
    """Rotate body-frame commands into the world frame for Gazebo."""

    def __init__(self) -> None:
        super().__init__("cmd_bridge")

        self.declare_parameter("world_frame", "world")
        self.declare_parameter("command_frame", "body")
        self.declare_parameter("force_publish_period", 0.1)
        self.declare_parameter("vel_publish_period", 0.05)
        self.declare_parameter("force_topic", "/space_cobot/cmd_force")
        self.declare_parameter("vel_topic", "/space_cobot/cmd_vel")
        self.declare_parameter("vel_output_topic", "/space_cobot/cmd_vel_world")
        self.declare_parameter(
            "force_output_topic", "/world/default/wrench/persistent"
        )

        self._world_frame = (
            self.get_parameter("world_frame").get_parameter_value().string_value
            or "world"
        )
        self._command_frame = (
            self.get_parameter("command_frame").get_parameter_value().string_value
            or "body"
        )
        force_period = float(
            self.get_parameter("force_publish_period")
            .get_parameter_value()
            .double_value
            or 0.1
        )
        vel_period = float(
            self.get_parameter("vel_publish_period")
            .get_parameter_value()
            .double_value
            or 0.05
        )
        force_topic = (
            self.get_parameter("force_topic").get_parameter_value().string_value
            or "/space_cobot/cmd_force"
        )
        vel_topic = (
            self.get_parameter("vel_topic").get_parameter_value().string_value
            or "/space_cobot/cmd_vel"
        )
        self._force_output_topic = (
            self.get_parameter("force_output_topic")
            .get_parameter_value()
            .string_value
            or "/world/default/wrench/persistent"
        )
        self._vel_output_topic = (
            self.get_parameter("vel_output_topic").get_parameter_value().string_value
            or "/space_cobot/cmd_vel_world"
        )

        self._force_publisher = self.create_publisher(
            EntityWrench,
            self._force_output_topic,
            10,
        )
        self._vel_publisher = self.create_publisher(
            Twist,
            self._vel_output_topic,
            10,
        )

        self._force_subscription = self.create_subscription(
            Wrench,
            force_topic,
            self._handle_force,
            10,
        )
        self._vel_subscription = self.create_subscription(
            Twist,
            vel_topic,
            self._handle_velocity,
            10,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._last_wrench = Wrench()
        self._last_twist = Twist()

        self._force_timer = self.create_timer(
            force_period, self._publish_current_wrench
        )
        self._vel_timer = self.create_timer(vel_period, self._publish_current_twist)

        self.get_logger().info(
            f"cmd_bridge active (force->{self._force_output_topic}, vel->{self._vel_output_topic})"
        )

    def _handle_force(self, msg: Wrench) -> None:
        self._last_wrench = deepcopy(msg)

    def _handle_velocity(self, msg: Twist) -> None:
        self._last_twist = deepcopy(msg)

    def _publish_current_wrench(self) -> None:
        transform = self._lookup_transform()
        if transform is None:
            return

        entity_wrench = EntityWrench()
        entity_wrench.header.stamp = self.get_clock().now().to_msg()
        entity_wrench.header.frame_id = self._world_frame

        entity = Entity()
        entity.id = 0
        entity.name = "space_cobot::body"
        entity.type = Entity.LINK
        entity_wrench.entity = entity

        entity_wrench.wrench.force = self._transform_vector(
            self._last_wrench.force, transform
        )
        entity_wrench.wrench.torque = self._transform_vector(
            self._last_wrench.torque, transform
        )
        self._force_publisher.publish(entity_wrench)

    def _publish_current_twist(self) -> None:
        transform = self._lookup_transform()
        if transform is None:
            return

        rotated = Twist()
        rotated.linear = self._transform_vector(self._last_twist.linear, transform)
        rotated.angular = self._transform_vector(self._last_twist.angular, transform)
        self._vel_publisher.publish(rotated)

    def _lookup_transform(self):
        try:
            return self._tf_buffer.lookup_transform(
                self._world_frame,
                self._command_frame,
                Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Missing transform {self._command_frame} -> {self._world_frame}: {exc}",
                throttle_duration_sec=5.0,
            )
            return None

    def _transform_vector(self, vector, transform):
        stamped = Vector3Stamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self._command_frame
        stamped.vector = vector

        transformed = do_transform_vector3(stamped, transform)
        return transformed.vector

def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CmdBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
