from simulation import Simulation
from pyvis import PyVis

import numpy as np

FOV_DISTANCE = 20
FOV_ANGLE = np.radians(45)
LANDMARKS = [
    (26, 19),
    (37, 9),
    (8, 5),
    (12, 36),
    (35, 28),
    (3, 33),
    (21, 2),
    (31, 15),
    (18, 24),
    (4, 14),
    (38, 22),
    (29, 6),
    (11, 10),
    (4, 25),
    (20, 7),
]


def start_simulation():
    sim = Simulation(
        FOV_DISTANCE,
        FOV_ANGLE,
        LANDMARKS,
        odom_sd=0.1,
        camera_range_sd=0.05,
        camera_angle_sd=0.02,
    )
    pyvis = PyVis(FOV_DISTANCE, FOV_ANGLE, LANDMARKS)
    pyvis.add_ground_truth_point(1, 1, np.radians(45))
    pyvis.add_odom_point(10, 15, np.radians(90))
    pyvis.add_slam_point(20, 20, np.radians(180))

    return sim, pyvis


def main():
    sim, pyvis = start_simulation()

    pyvis.hold()


if __name__ == "__main__":
    main()
