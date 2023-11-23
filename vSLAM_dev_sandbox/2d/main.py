from simulation import Simulation
from pyvis import PyVis

import numpy as np

from dataclasses import dataclass

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


@dataclass
class OdomPose:
    x: float
    y: float
    theta: float


def start_simulation(x, y, theta):
    sim = Simulation(
        (x, y, theta),
        FOV_DISTANCE,
        FOV_ANGLE,
        LANDMARKS,
        odom_sd=0.1,
        camera_range_sd=0.08,
        camera_angle_sd=0.02,
    )
    pyvis = PyVis(FOV_DISTANCE, FOV_ANGLE, LANDMARKS)
    odom_pose = OdomPose(x, y, theta)

    pyvis.add_ground_truth_point(x, y, np.radians(theta))
    pyvis.add_odom_point(x, y, np.radians(theta))
    pyvis.add_slam_point(x, y, np.radians(theta))

    return sim, pyvis, odom_pose


def get_input():
    while True:
        s = input("[vx vy w dt]: ").split()

        if len(s) != 4 and len(s) != 3:
            print("Invalid input")
            continue
        elif s == "exit":
            return None

        vx = float(s[0])
        vy = float(s[1])
        w = np.radians(float(s[2]))
        dt = 1 if len(s) == 3 else float(s[3])

        return vx, vy, w, dt


def handle_ground_truth(sim, pyvis, vx, vy, w, dt):
    gt_x, gt_y, gt_theta = sim.compute_next_pose(vx, vy, w, dt)
    pyvis.add_ground_truth_point(gt_x, gt_y, gt_theta)
    print(f"gt_x: {gt_x}, gt_y: {gt_y}, gt_theta: {np.degrees(gt_theta)}")
    print("- - - - - - - - - - -")


def handle_odom(sim, pyvis, odom_pose, dt):
    odom_vx, odom_vy, odom_w = sim.get_odometry(dt)

    odom_pose.theta += odom_w * dt
    odom_pose.x += (
        odom_vx * np.cos(odom_pose.theta) + odom_vy * np.sin(odom_pose.theta)
    ) * dt
    odom_pose.y += (
        odom_vx * np.sin(odom_pose.theta) + odom_vy * np.cos(odom_pose.theta)
    ) * dt

    pyvis.add_odom_point(odom_pose.x, odom_pose.y, odom_pose.theta)


def sim_loop(sim, pyvis, odom_pose):
    while True:
        vx, vy, w, dt = get_input()

        if vx is None:
            return

        handle_ground_truth(sim, pyvis, vx, vy, w, dt)
        handle_odom(sim, pyvis, odom_pose, dt)


def main():
    sim, pyvis, odom_pose = start_simulation(1, 1, 0)
    sim_loop(sim, pyvis, odom_pose)
    pyvis.hold()


if __name__ == "__main__":
    main()
