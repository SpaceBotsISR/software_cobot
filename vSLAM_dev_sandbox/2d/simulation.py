import numpy as np


class Simulation:
    def __init__(
        self,
        initial_pose: tuple[float, float, float],
        fov_distance: float,
        fov_angle: float,
        landmarks: list[tuple[float, float]],
        odom_sd: float = 0.1,
        camera_range_sd: float = 0.05,
        camera_angle_sd: float = 0.02,
    ) -> None:
        self.fov = fov_angle
        self.fov_distance = fov_distance
        self.x = initial_pose[0]
        self.y = initial_pose[1]
        self.theta = initial_pose[2]

        self.vx = 0.0
        self.vy = 0.0
        self.w = 0.0

        self.landmarks = landmarks
        self.odom_sd = odom_sd
        self.camera_range_sd = camera_range_sd
        self.camera_angle_sd = camera_angle_sd

    def odom_noise(self, dt) -> None:
        noise = 0.0

        for t in range(int(dt) + 1):
            noise += np.random.normal(0, self.odom_sd)
        return noise

    def camera_range_noise(self) -> None:
        return np.random.normal(0, self.camera_range_sd)

    def camera_angle_noise(self) -> None:
        return np.random.normal(0, self.camera_angle_sd)

    def compute_next_pose(
        self, vx: float, vy: float, w: float, dt: float
    ) -> tuple[float, float, float]:
        self.vx = vx
        self.vy = vy
        self.w = w

        print(f"prev -> x: {self.x}, y: {self.y}, theta: {np.degrees(self.theta)}")
        self.theta += w * dt
        self.x += (vx * np.cos(self.theta) + vy * np.sin(self.theta)) * dt
        self.y += (vx * np.sin(self.theta) + vy * np.cos(self.theta)) * dt

        return self.x, self.y, self.theta

    def get_odometry(self, dt) -> tuple[float, float, float]:
        odom_vx = self.vx + self.odom_noise(dt)
        odom_vy = self.vy + self.odom_noise(dt)
        odom_w = self.w + self.odom_noise(dt)

        return odom_vx, odom_vy, odom_w

    def is_landmark_in_fov_angle(self, landmark_x: float, landmark_y: float) -> bool:
        dx = landmark_x - self.x
        dy = landmark_y - self.y

        angle = np.arctan2(dy, dx) - self.theta

        in_fov = np.abs(angle) < self.fov / 2
        in_range = np.sqrt(dx**2 + dy**2) < self.fov_distance

        return in_fov and in_range

    def get_camera_measurements(self) -> list[dict]:
        camera_measurements = []

        for landmark in self.landmarks:
            landmark_x = landmark[0]
            landmark_y = landmark[1]

            if self.is_landmark_in_fov_angle(landmark_x, landmark_y):
                distance = (
                    np.sqrt((landmark_x - self.x) ** 2 + (landmark_y - self.y) ** 2)
                    + self.camera_range_noise()
                )
                angle = (
                    np.arctan2(landmark_y - self.y, landmark_x - self.x) - self.theta
                ) + self.camera_angle_noise()

                measurment = {
                    "distance": distance,
                    "angle": angle,
                    "id": landmark.index(),
                }
                camera_measurements.append(measurment)

        return camera_measurements
