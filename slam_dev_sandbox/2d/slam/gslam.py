import gtsam
import numpy as np
from gtsam.symbol_shorthand import X
from scipy.spatial.transform import Rotation as R


class GraphSlam:
    def __init__(self):
        self.isam = gtsam.ISAM2()
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimate = gtsam.Values()

        # TODO: tune noise model
        n = 0.1
        self.pose_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([n, n, 0, 0, 0, n]))

        self.current_pose = gtsam.Pose3()  # Initialize with default pose
        self.time_step = 0

    @staticmethod
    def roll_pitch_yaw_from_R(
        rotation_matrix: np.ndarray,
    ) -> tuple[float, float, float]:
        """
        Extract roll, pitch, yaw from rotation matrix

        :param rotation_matrix: 3x3 rotation matrix

        :return: roll, pitch, yaw
        """

        rotation = R.from_matrix(rotation_matrix)

        roll, pitch, yaw = rotation.as_euler(
            "zyx", degrees=True
        )  # Use 'radians' if you prefer radians over degrees

        return roll, pitch, yaw

    def add_imu_measurments(
        self, position_increment: np.array, c. : np.array, delta_t: float
    ) -> tuple[float, float, float]:
        """
        Add IMU measurements to the graph and get updated pose

        :param position_increment: 3x1 numpy array with position increment
        :param rotation_xyz: 3x1 numpy array with rotation increment (Euler angles)
        :param delta_t: Time interval in seconds

        :return: Updated pose
        """
        self.time_step += 1

        # Update pose
        current_position = self.current_pose.translation()
        new_position = current_position + position_increment * delta_t

        # Orientation
        current_orientation = R.from_matrix(self.current_pose.rotation().matrix())
        delta_rotation = R.from_euler("xyz", rotation_xyz * delta_t)
        new_orientation = delta_rotation * current_orientation
        new_orientation_quat = new_orientation.as_quat()

        # Update current pose
        self.current_pose = gtsam.Pose3(
            gtsam.Rot3(new_orientation.as_matrix()), new_position
        )
        # Add the current pose as an initial estimate for the new time step
        self.initial_estimate.insert(X(self.time_step), self.current_pose)

        # Add a factor to the graph for this new pose
        pose_factor = gtsam.PriorFactorPose3(
            X(self.time_step), self.current_pose, self.pose_noise
        )
        self.graph.add(pose_factor)

        # Update ISAM2 and calculate the best estimate
        self.isam.update(self.graph, self.initial_estimate)
        self.graph.resize(0)
        self.initial_estimate.clear()

        current_estimate = self.isam.calculateEstimate()
        return (
            current_estimate.atPose3(X(self.time_step)).x(),
            current_estimate.atPose3(X(self.time_step)).y(),
            current_estimate.atPose3(X(self.time_step)).rotation().yaw(),
        )


def _test():
    pose_estimator = GraphSlam()
    position_increment = np.array([500, 0.0, 0.0])  # Example position increment
    rotation_xyz = np.array(
        [0.0, 0.0, 0.1]
    )  # Example rotation increment (Euler angles)
    delta_t = 0.1  # Time interval in seconds

    pose_estimator.add_imu_measurments(position_increment, rotation_xyz, delta_t)

    position_increment = np.array([500, 0.0, 0.0])  # Example position increment
    rotation_xyz = np.array(
        [0.0, 0.0, 0.1]
    )  # Example rotation increment (Euler angles)
    delta_t = 0.1  # Time interval in seconds

    x, y, yaw = pose_estimator.add_imu_measurments(
        position_increment, rotation_xyz, delta_t
    )

    print(f"Pose: {x}, {y}, {yaw}")


if __name__ == "__main__":
    _test()
