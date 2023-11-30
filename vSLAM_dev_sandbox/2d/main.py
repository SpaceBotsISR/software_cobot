from utils.simulation import Simulation
from utils.pyvis import PyVis
from utils.odom_pose import OdomPose
from slam.gslam import GraphSlam

import numpy as np

STEP = 4
ROTATE_90_CW = [[0, 0, np.radians(-90)]]
ROTATE_90_CCW = [[0, 0, np.radians(90)]]
UP = [[0, STEP, 0]]
DOWN = [[0, -STEP, 0]]
LEFT = [[-STEP, 0, 0]]
RIGHT = [[STEP, 0, 0]]
MOVES = RIGHT + UP + (5 * RIGHT + 2 * UP + 5 * LEFT + 2 * UP) * 2


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


def start_simulation(
    x: float, y: float, theta: float
) -> tuple[Simulation, PyVis, OdomPose]:
    sim = Simulation(
        (x, y, theta),
        FOV_DISTANCE,
        FOV_ANGLE,
        LANDMARKS,
        odom_range_sd=0.55,
        odom_angle_sd=0.04,
        camera_range_sd=0.08,
        camera_angle_sd=0.02,
    )
    pyvis = PyVis(FOV_DISTANCE, FOV_ANGLE, LANDMARKS)
    odom_pose = OdomPose(x, y, theta)

    pyvis.add_ground_truth_point(x, y, np.radians(theta))
    pyvis.add_odom_point(x, y, np.radians(theta))
    pyvis.add_slam_point(x, y, np.radians(theta))

    return sim, pyvis, odom_pose


def get_input(moves: list = []) -> tuple[float, float, float, float]:
    if len(moves) != 0:
        return moves.pop(0)

    while True:
        s = input("[vx vy w]: ").split()

        print("s: ", s)
        if s[0] == "r":
            return None
        elif len(s) != 3:
            print("Invalid input")
            continue

        vx = float(s[0])
        vy = float(s[1])
        w = np.radians(float(s[2]))
        return vx, vy, w


def handle_ground_truth(
    sim: Simulation, pyvis: PyVis, vx: float, vy: float, w: float, dt: float
) -> None:
    gt_x, gt_y, gt_theta = sim.compute_next_pose(vx, vy, w, dt)
    pyvis.add_ground_truth_point(gt_x, gt_y, gt_theta)
    print(f"gt_x-> {gt_x}, gt_y: {gt_y}, gt_theta: {np.degrees(gt_theta)}")
    print("- - - - - - - - - - -")


def handle_odom(sim: Simulation, pyvis: PyVis, odom_pose: OdomPose, dt: float) -> None:
    odom_vx, odom_vy, odom_w = sim.get_odometry(dt)

    odom_pose.theta += odom_w * dt
    odom_pose.x += (
        odom_vx * np.cos(odom_pose.theta) + odom_vy * np.sin(odom_pose.theta)
    ) * dt
    odom_pose.y += (
        odom_vx * np.sin(odom_pose.theta) + odom_vy * np.cos(odom_pose.theta)
    ) * dt

    pyvis.add_odom_point(odom_pose.x, odom_pose.y, odom_pose.theta)


def handle_slam(pyvis: PyVis, slam_x, slam_y, slam_theta) -> None:
    pyvis.add_slam_point(slam_x, slam_y, slam_theta)


def sim_loop(sim: Simulation, pyvis: PyVis, odom_pose: OdomPose) -> None:
    moves = MOVES.copy()  # To insert manualy the moves set moves = []

    while True:
        input_val = get_input(moves)

        if input_val is None:
            return

        vx, vy, w = input_val

        handle_ground_truth(sim, pyvis, vx, vy, w, 1)
        handle_odom(sim, pyvis, odom_pose, 1)


def main() -> None:
    sim, pyvis, odom_pose = start_simulation(0, 0, 0)
    sim_loop(sim, pyvis, odom_pose)


if __name__ == "__main__":
    gs = GraphSlam()
    # main()
