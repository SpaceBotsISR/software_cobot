import numpy as np

X, Y, THETA = 0, 1, 2


class Simulation:
    def __init__(self) -> None:
        self._ground_truth_poses = np.zeros((3, 1))
        self._control_inputs = np.array(
            [
                [1, 1, 1, 0, 0, 0, 0, 0, 1, 3],
                [0, 0, 0, 2, 0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, np.pi / 4, np.pi / 4, 0, 0, 0, 0],
            ]
        )
        self._landmarks = np.array(
            [
                [0, 0],
                [1, 1],
                [2, 2],
                [3, 3],
                [4, 4],
                [5, 5],
                [6, 6],
                [7, 7],
                [8, 8],
                [9, 9],
            ]
        )

        self.compute_ground_truth_poses()

    def compute_ground_truth_poses(self):
        for t in range(self._control_inputs.shape[1]):
            x = self._ground_truth_poses[X, -1] + (
                self._control_inputs[X, t] * np.cos(self._ground_truth_poses[THETA, -1])
                - self._control_inputs[Y, t]
                * np.sin(self._ground_truth_poses[THETA, -1])
            )
            y = self._ground_truth_poses[Y, -1] + (
                self._control_inputs[X, t] * np.sin(self._ground_truth_poses[THETA, -1])
                + self._control_inputs[Y, t]
                * np.cos(self._ground_truth_poses[THETA, -1])
            )
            theta = self._ground_truth_poses[THETA, -1] + self._control_inputs[THETA, t]
            pose = np.array([[x], [y], [theta]])
            self._ground_truth_poses = np.hstack([self._ground_truth_poses, pose])
        print(self._ground_truth_poses.shape)


Simulation()
