import numpy as np
import gtsam
from gtsam.symbol_shorthand import X, V

from imu import IMU  # Assuming this is similar to the IMUHandler class


def vector3(x, y, z):
    return np.array([x, y, z], dtype=float)


class GraphSlam:
    def __init__(
        self,
        timestamp: float,
        accelerometer_cov: float = 1e-3,
        gyroscope_cov: float = 1e-4,
        integration_cov: float = 1e-4,
        g: float = 9.81,
    ) -> None:
        self.imu = IMU(timestamp)

        self.key = 0  # Incremental key for the factor graph
        self.isam = gtsam.ISAM2()
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimate = gtsam.Values()

        # Set initial conditions
        self.initial_pose = gtsam.Pose3()
        self.initial_velocity = gtsam.Vector3(0, 0, 0)
        self.initial_bias = gtsam.imuBias.ConstantBias()

        self._add_initial_priors()

    def _add_initial_priors(self):
        """
        Add initial priors to the graph and initial estimates.
        """
        # Noise models
        pose_prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1] * 6))
        velocity_prior_noise = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)
        bias_prior_noise = gtsam.noiseModel.Isotropic.Sigma(6, 0.1)

        # Add pose, velocity, and bias priors to the graph
        self.graph.add(
            gtsam.PriorFactorPose3(X(self.key), self.initial_pose, pose_prior_noise)
        )
        self.graph.add(
            gtsam.PriorFactorVector(
                V(self.key), self.initial_velocity, velocity_prior_noise
            )
        )
        self.graph.add(
            gtsam.PriorFactorConstantBias(
                gtsam.symbol("b", self.key), self.initial_bias, bias_prior_noise
            )
        )

        # Add pose, velocity, and bias priors to the initial estimate
        self.initial_estimate.insert(X(self.key), self.initial_pose)
        self.initial_estimate.insert(V(self.key), self.initial_velocity)
        self.initial_estimate.insert(gtsam.symbol("b", self.key), self.initial_bias)

    def add_imu_data(
        self,
        timestamp: float,
        linear_acceleration: np.ndarray,
        angular_velocity: np.ndarray,
    ) -> None:
        dt = timestamp - self.imu.last_timestamp
        self.imu.add_measurement(linear_acceleration, angular_velocity, dt)

        new_key = self.key + 1
        self.key += 1

        # Predict the new state
        predicted_pose = self.initial_pose.retract(np.random.rand(6) * 0.01)
        predicted_velocity = self.initial_velocity + np.random.rand(3) * 0.01

        # Add the predicted state to the initial estimate
        self.initial_estimate.insert(X(new_key), predicted_pose)
        self.initial_estimate.insert(V(new_key), predicted_velocity)

        # Add IMU factor to the graph
        self.graph.add(
            self.create_imu_factor(
                X(self.key - 1), V(self.key - 1), X(new_key), V(new_key)
            )
        )

        self.isam.update(self.graph, self.initial_estimate)  # Update iSAM

        # Clear the graph and estimates for the next iteration
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimate = gtsam.Values()
        self.reset_preintegrator()

    def get_current_pose(self) -> gtsam.Pose3:
        current_estimate = self.isam.calculateEstimate()
        return current_estimate.atPose3(X(self.key))
