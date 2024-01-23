import numpy as np
import numpy.linalg as la
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation


def S(v):
    """Skew-symmetric matrix"""
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


class ValidateSim:
    def __init__(self):
        self.A1M_sim = np.load("mats/sim_A1M.npy")
        self.A1M_est = np.load("mats/est_A1M.npy")

        m = 5.8
        r = 0.25  # m
        h = 0.15  # m
        J = np.diag(
            [m * (3 * r * r + h * h) / 12, m * (3 * r * r + h * h) / 12, m * r * r / 2]
        )  # this is JB
        Jinv = la.inv(J)
        self.L = np.linalg.cholesky(J / m)

        self.c = np.array([0.01, 0.02, 0.05])
        self.g = np.array([0, 0, -9.8])
        self.timestamp = []
        self.w = []
        self.R = []
        self.u = []

        self.read_file()
        self.peprocess_data()
        self.plot_erro()

    def parse_line(self, line):
        """
        Convert a line of numbers to a numpy array.

        Parameters:
            line: The line of numbers.

        Output:
            An array of numbers in the line.
        """
        line = line.replace("\n", "").replace(" ", ",")
        numbers = line.strip().split(",")

        return np.array([[float(n) for n in numbers if n != ""]])

    def read_file(self):
        """
        Read the input file and store the data in the class variables.

        Parameters:
            file_name: The name of the input file.
        """
        with open("data/sim.txt") as fp:
            contents = fp.read()
            measurements = contents.split("\n\n")

            for m in measurements:
                m = m.split("\n")

                if len(m) == 1 and m[0] == "":
                    return

                self.timestamp.append(float(m[0].strip()))
                self.w.append(self.parse_line(m[1]).T)
                self.R.append(self.get_R(m[2]))
                self.u.append(self.parse_line(m[3]).T)

    def get_R(self, line):
        """
        Convert a quaternion to a 3x3 rotation matrix.

        Parameters:
            quaternion: The quaternion in the format (w, x, y, z).

        Output:
            3x3 rotation matrix.
        """
        w, x, y, z = [float(x) for x in line.replace(" ", ",").split(",")]

        R = Rotation.from_quat([w, x, y, z]).as_matrix()

        return R

    def peprocess_data(self):
        """Pecumpute the data to be used in the cost function."""

        self.w_diff = [
            ((self.w[i + 1] - self.w[i]) / self.timestamp[i])
            for i in range(len(self.timestamp) - 1)
        ]
        self.w_avg = [(self.w[i + 1] + self.w[i]) / 2 for i in range(len(self.w) - 1)]
        self.u_avg = [(self.u[i + 1] + self.u[i]) / 2 for i in range(len(self.u) - 1)]

        R_svd = [np.linalg.svd(self.R[i + 1]) for i in range(len(self.R) - 1)]
        self.R_avg = [
            R_svd[i][0]
            @ np.diag([1, 1, np.linalg.det(R_svd[i][0]) * np.linalg.det(R_svd[i][2])])
            @ R_svd[i][2]
            for i in range(len(R_svd))
        ]

    def plot_erro(self):
        L = self.L
        error_sim = []
        error_est = []

        for i in range(len(self.timestamp) - 1):
            e1 = la.norm(
                L @ L.T @ self.w_diff[i]
                + S(self.w_avg[i].flatten()) @ L @ L.T @ self.w_avg[i]
                - S(self.c) @ self.R_avg[i].T @ self.g
                - self.A1M_sim @ self.u_avg[i]
            )

            e2 = la.norm(
                L @ L.T @ self.w_diff[i]
                + S(self.w_avg[i].flatten()) @ L @ L.T @ self.w_avg[i]
                - S(self.c) @ self.R_avg[i].T @ self.g
                - self.A1M_est @ self.u_avg[i]
            )

            error_sim.append(e1)
            error_est.append(e2)

        print("MSE: ", np.mean(np.array(error_sim) ** 2))

        plt.plot(self.timestamp[:-1], error_sim, label="Simulation")
        plt.plot(self.timestamp[:-1], error_est, label="Estimation")
        plt.legend()

        plt.xlabel("Timestamp")
        plt.ylabel("Error")
        plt.show()


if __name__ == "__main__":
    ValidateSim()
