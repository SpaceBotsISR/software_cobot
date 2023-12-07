from utils.simulation import Simulation
from utils.pyvis import PyVis
from utils.odom_pose import OdomPose
from slam.gslam import GraphSlam

import numpy as np

# Moves
STEP = 4
ROTATE_90_CW = [[0, 0, np.radians(-90)]]
ROTATE_90_CCW = [[0, 0, np.radians(90)]]
UP = [[0, STEP, 0]]
DOWN = [[0, -STEP, 0]]
LEFT = [[-STEP, 0, 0]]
RIGHT = [[STEP, 0, 0]]
MOVES = (
    RIGHT + UP + (5 * RIGHT + 2 * UP + 5 * LEFT + 2 * UP) * 2 + ROTATE_90_CCW + RIGHT
)

# Simulation params
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

prev_slam_yaw = 0.0


def start_simulation(
    x: float, y: float, theta: float
) -> tuple[Simulation, PyVis, OdomPose]:
    """
    Instanciate simulation and visualization objects

    :param x: Initial x position
    :param y: Initial y position
    :param theta: Initial theta

    :return: Simulation, PyVis, OdomPose
    """
    sim = Simulation(
        (x, y, theta),
        FOV_DISTANCE,
        FOV_ANGLE,
        LANDMARKS,
        odom_range_sd=0.6,
        odom_angle_sd=0.05,
        camera_range_sd=0.08,
        camera_angle_sd=0.02,
    )
    pyvis = PyVis(FOV_DISTANCE, FOV_ANGLE, LANDMARKS)
    odom_pose = OdomPose(x, y, theta)

    pyvis.add_ground_truth_point(x, y, np.radians(theta))
    pyvis.add_odom_point(x, y, np.radians(theta))
    pyvis.add_slam_point(x, y, np.radians(theta))

    return sim, pyvis, odom_pose


def get_gt_v(moves: list = []) -> tuple[float, float, float, float]:
    """
    Simulates IMU measurements

    :param moves: List of moves to simulate

    :return: vx, vy, w
    """
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
    """
    Simulates ground truth measurements

    :param sim: Simulation object
    :param pyvis: PyVis object
    :param vx: Linear velocity in x
    :param vy: Linear velocity in y
    :param w: Angular velocity
    :param dt: Time interval
    """
    gt_x, gt_y, gt_theta = sim.compute_next_pose(vx, vy, w, dt)
    pyvis.add_ground_truth_point(gt_x, gt_y, gt_theta)
    print(f"gt_x-> {gt_x}, gt_y: {gt_y}, gt_theta: {np.degrees(gt_theta)}")
    print("- - - - - - - - - - -")


def handle_odom(
    sim: Simulation,
    pyvis: PyVis,
    odom_pose: OdomPose,
    vx: float,
    vy: float,
    v_theta: float,
    dt: float,
) -> None:
    """
    Odometry integartion

    :param sim: Simulation object
    :param pyvis: PyVis object
    :param odom_pose: OdomPose object
    :param dt: Time interval
    """

    odom_pose.theta += v_theta * dt
    odom_pose.x += (vx * np.cos(odom_pose.theta) + vy * np.sin(odom_pose.theta)) * dt
    odom_pose.y += (vx * np.sin(odom_pose.theta) + vy * np.cos(odom_pose.theta)) * dt

    pyvis.add_odom_point(odom_pose.x, odom_pose.y, odom_pose.theta)


def handle_slam(
    pyvis: PyVis, graph_slam: GraphSlam, vx: float, vy: float, vtheta: float
) -> None:
    """
    Slam integration

    :param pyvis: PyVis object
    :param graph_slam: GraphSlam object
    :param vx: Linear velocity in x
    :param vy: Linear velocity in y
    :param vtheta: Angular velocity
    """
    global prev_slam_yaw
    yaw = prev_slam_yaw

    dx = vx * np.cos(yaw) + vy * np.sin(yaw)
    dy = vx * np.sin(yaw) + vy * np.cos(yaw)

    x, y, theta = graph_slam.add_imu_measurments(
        np.array([dx, dy, 0]), np.array([0, 0, vtheta]), 1
    )
    pyvis.add_slam_point(x, y, theta)
    prev_slam_yaw = theta


def sim_loop(
    sim: Simulation, pyvis: PyVis, odom_pose: OdomPose, use_move_list=True
) -> None:
    """
    Simulation loop

    :param sim: Simulation object
    :param pyvis: PyVis object
    :param odom_pose: OdomPose object
    :param use_move_list: If True, use the predefined move list
    """
    moves = MOVES.copy() if use_move_list else []
    graph_slam = GraphSlam()

    while True:
        gt_v = get_gt_v(moves)

        if gt_v is None:
            return

        vx, vy, w = gt_v
        handle_ground_truth(sim, pyvis, vx, vy, w, 1)
        vx, vy, w = sim.get_imu_measurments(1)

        handle_odom(sim, pyvis, odom_pose, vx, vy, w, 1)
        handle_slam(pyvis, graph_slam, vx, vy, w)


def main() -> None:
    sim, pyvis, odom_pose = start_simulation(0, 0, 0)
    sim_loop(sim, pyvis, odom_pose, use_move_list=True)


if __name__ == "__main__":
    main()
