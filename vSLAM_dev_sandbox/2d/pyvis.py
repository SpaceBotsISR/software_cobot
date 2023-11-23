import matplotlib.pyplot as plt
import numpy as np


class PyVis:
    def __init__(
        self,
        fov_distance: float,
        fov_angle: float,
        landmarks: list[tuple[float, float]],
    ) -> None:
        self.fov_distance = fov_distance
        self.fov_angle = fov_angle
        self.landmarks = landmarks
        self.odom_points = []
        self.ground_truth_points = []
        self.slam_points = []

        plt.ion()  # Enable interactive mode
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(0, 40)
        self.ax.set_ylim(0, 40)
        self.ax.scatter(*zip(*self.landmarks), marker="*", color="b", label="Landmarks")

        for i, (x, y) in enumerate(self.landmarks):
            self.ax.text(
                x, y, "id = " + str(i), fontsize=8, ha="center", va="bottom", color="b"
            )

        self.ax.grid()
        self.ax.legend()

    def add_odom_point(self, x: float, y: float) -> None:
        self.odom_points.append((x, y))
        self.ax.scatter(x, y, marker="o", color="g", label="Odom Points")
        self.ax.legend()
        self.fig.canvas.flush_events()  # Update the plot

    def add_ground_truth_point(self, x: float, y: float, theta: float) -> None:
        self.ground_truth_points.append((x, y))
        self.ax.scatter(x, y, marker="o", color="r", label="Ground Truth Points")

        # Calculate FoV line endpoints
        left_fov_x, left_fov_y = self.calculate_fov_endpoint(
            x, y, theta, direction="left"
        )
        right_fov_x, right_fov_y = self.calculate_fov_endpoint(
            x, y, theta, direction="right"
        )

        # Draw FoV lines
        self.ax.plot(
            [x, left_fov_x], [y, left_fov_y], linestyle=":", color="grey", alpha=0.5
        )
        self.ax.plot(
            [x, right_fov_x], [y, right_fov_y], linestyle=":", color="grey", alpha=0.5
        )

        self.ax.legend()
        self.fig.canvas.flush_events()  # Update the plot

    def calculate_fov_endpoint(
        self, x: float, y: float, theta: float, direction: float = "left"
    ) -> tuple[float, float]:
        # Calculate the half angle offset based on the direction
        angle_offset = self.fov_angle / 2
        if direction == "right":
            angle_offset = -angle_offset

        # Calculate the absolute angle considering the robot's orientation
        # Assuming the robot is facing upwards (90 degrees)
        absolute_angle = theta + angle_offset

        # Calculate the endpoint coordinates
        fov_x = x + self.fov_distance * np.cos(absolute_angle)
        fov_y = y + self.fov_distance * np.sin(absolute_angle)

        return fov_x, fov_y

    def add_slam_point(self, x: float, y: float) -> None:
        self.slam_points.append((x, y))
        self.ax.scatter(x, y, marker="o", color="m", label="SLAM Points")
        self.ax.legend()
        self.fig.canvas.flush_events()  # Update the plot

    def set_initial_state(self, x: float, y: float, theta: float) -> None:
        self.add_odom_point(x, y, theta)
        self.add_slam_point(x, y, theta)
        self.add_ground_truth_point(x, y, theta)

    def hold(self) -> None:
        plt.ioff()  # Disable interactive mode
        plt.show()
