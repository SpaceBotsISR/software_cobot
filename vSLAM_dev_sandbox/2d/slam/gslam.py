import numpy as np
import gtsam as gs


class GraphSlam:
    def __init__(self) -> None:
        self.isam = gs.ISAM2()
        self.graph = gs.NonlinearFactorGraph()
        self.initial_estimate = gs.Values()

    def add_imu_factor(self, imu_factor: gs.PreintegratedImuMeasurements) -> None:
        self.graph.push_back(imu_factor)
