import numpy as np
import gtsam as gs
from gtsam.symbol_shorthand import B, V, X  # Bias, Vel, Pose


G = 9.81


class IMU:
    def __init__(self) -> None:
        pass

    def preintegartion_parameters(
        self,
        accelerometer_cov: float,
        gyroscope_cov: float,
        integration_cov: float,
    ):
        """
        Creates IMU preintegration parameters

        Args:
            accelerometer_cov (float): Accelerometer covariance
            gyroscope_cov (float): Gyroscope covariance
            integration_cov (float): Integration covariance
        """
        # Set params
        PARAMS = gs.PreintegrationParams.MakeSharedU(G)
        I = np.identity(3, float)
        PARAMS.setAccelerometerCovariance(I * accelerometer_cov)
        PARAMS.setGyroscopeCovariance(I * gyroscope_cov)
        PARAMS.setIntegrationCovariance(I * integration_cov)
        PARAMS.setUse2ndOrderCoriolis(False)
        PARAMS.setOmegaCoriolis(gs.Vector3(0, 0, 0))

        # Set bias covariance (6x6 matrix with 0.1 variance on diagonal)
        BIAS_COVARIANCE = gs.noiseModel.Isotropic.Variance(6, 0.1)

        DELTA = gs.Pose3(gs.Rot3.Rodrigues(0, 0, 0), gs.Point3(0.0, 0, 0))  # pose

        return PARAMS, BIAS_COVARIANCE, DELTA
