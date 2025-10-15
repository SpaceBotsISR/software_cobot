import math
from typing import Optional, Tuple

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from rclpy.node import Node


def _stamp_to_float(stamp: Time) -> float:
    """Convert ROS stamp to float seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


def _quat_conjugate(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    """Return quaternion conjugate (x, y, z, w)."""
    x, y, z, w = q
    return (-x, -y, -z, w)


def _quat_multiply(
    q1: Tuple[float, float, float, float], q2: Tuple[float, float, float, float]
) -> Tuple[float, float, float, float]:
    """Hamilton product of two quaternions (x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _normalize_quaternion(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    """Normalize quaternion; fall back to identity if norm underflows."""
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    inv = 1.0 / norm
    return (x * inv, y * inv, z * inv, w * inv)


class PoseTwistBridge(Node):
    """Derive twist from pose samples coming from Gazebo."""

    def __init__(self) -> None:
        super().__init__("space_cobot_state_bridge")

        pose_topic = self.declare_parameter("pose_topic", "/space_cobot/pose").value
        twist_topic = self.declare_parameter("twist_topic", "/space_cobot/twist").value
        twist_stamped_topic = self.declare_parameter(
            "twist_stamped_topic", "/space_cobot/twist_stamped"
        ).value
        frame_id = self.declare_parameter("frame_id", "world").value

        self._frame_id = frame_id
        self._prev_pose: Optional[PoseStamped] = None
        self._prev_time: Optional[float] = None

        qos_depth = self.declare_parameter("queue_depth", 10).value
        self._pose_sub = self.create_subscription(
            PoseStamped, pose_topic, self._on_pose, qos_depth
        )
        self._twist_pub = self.create_publisher(Twist, twist_topic, qos_depth)
        self._twist_stamped_pub = self.create_publisher(
            TwistStamped, twist_stamped_topic, qos_depth
        )

        self.get_logger().info(
            f"PoseTwistBridge listening on {pose_topic} -> publishing "
            f"{twist_topic} / {twist_stamped_topic}"
        )

    def _on_pose(self, msg: PoseStamped) -> None:
        current_time = _stamp_to_float(msg.header.stamp)
        if self._prev_pose is None or self._prev_time is None:
            self._store_previous(msg, current_time)
            return

        dt = current_time - self._prev_time
        if dt <= 1e-6:
            self._store_previous(msg, current_time)
            return

        linear = self._compute_linear_velocity(self._prev_pose, msg, dt)
        angular = self._compute_angular_velocity(self._prev_pose, msg, dt)

        twist = Twist()
        twist.linear.x, twist.linear.y, twist.linear.z = linear
        twist.angular.x, twist.angular.y, twist.angular.z = angular

        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = msg.header.stamp
        twist_stamped.header.frame_id = self._frame_id
        twist_stamped.twist = twist

        self._twist_pub.publish(twist)
        self._twist_stamped_pub.publish(twist_stamped)

        self._store_previous(msg, current_time)

    @staticmethod
    def _compute_linear_velocity(
        prev_pose: PoseStamped, current_pose: PoseStamped, dt: float
    ) -> Tuple[float, float, float]:
        dx = current_pose.pose.position.x - prev_pose.pose.position.x
        dy = current_pose.pose.position.y - prev_pose.pose.position.y
        dz = current_pose.pose.position.z - prev_pose.pose.position.z
        inv_dt = 1.0 / dt
        return (dx * inv_dt, dy * inv_dt, dz * inv_dt)

    @staticmethod
    def _compute_angular_velocity(
        prev_pose: PoseStamped, current_pose: PoseStamped, dt: float
    ) -> Tuple[float, float, float]:
        prev_q = (
            prev_pose.pose.orientation.x,
            prev_pose.pose.orientation.y,
            prev_pose.pose.orientation.z,
            prev_pose.pose.orientation.w,
        )
        curr_q = (
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w,
        )
        prev_norm = _normalize_quaternion(prev_q)
        curr_norm = _normalize_quaternion(curr_q)
        # Relative rotation from previous to current.
        dq = _quat_multiply(curr_norm, _quat_conjugate(prev_norm))
        dq = _normalize_quaternion(dq)
        vx, vy, vz, vw = dq
        sin_half_angle = math.sqrt(vx * vx + vy * vy + vz * vz)

        if sin_half_angle < 1e-8:
            return (0.0, 0.0, 0.0)

        angle = 2.0 * math.atan2(sin_half_angle, max(-1.0, min(1.0, vw)))
        inv_mag = 1.0 / sin_half_angle
        axis = (vx * inv_mag, vy * inv_mag, vz * inv_mag)

        inv_dt = 1.0 / dt
        return (axis[0] * angle * inv_dt, axis[1] * angle * inv_dt, axis[2] * angle * inv_dt)

    def _store_previous(self, pose: PoseStamped, stamp_sec: float) -> None:
        self._prev_pose = pose
        self._prev_time = stamp_sec


def main() -> None:
    rclpy.init()
    node = PoseTwistBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
