import numpy as np
import scipy.optimize as sopt
from scipy.spatial.transform import Rotation
import sys


# Cobot parameters
g = np.array([[0], [0], [-9.81]], dtype=float)  # gravity acceleration
m = 3  # mass of Cobot
m_batt = 0.5  # mass of the battery
M = m + m_batt  # total mass
r = 0.25  # radius of Cobot
h = 0.10  # height of Cobot


class Estimator:
    def __init__(self, file_name="input_data.txt"):
        self.iter = 0
        self.cost = 0xFFFFFFFF
        self.timestamp = []
        self.w = []
        self.R = []
        self.u = []

        self.read_file(file_name)
        self.peprocess_data()

    def print(self):
        for i in range(len(self.timestamp)):
            print("\n - - - - - - - - - - - - - - - - - - - - - - ")
            print(f"t:\n {self.timestamp[i]}")
            print(f"w:\n{self.w[i]}")
            print(f"R:\n{self.R[i]}")
            print(f"u:\n{self.u[i]}")

    @property
    def L(self, mass=M):
        """
        Get lower triangular matrix 3x3 from the moment of inertia.

        Parameters:
        - mass: The total mass of Cobot.

        Output:
        - L: Lower triangular matrix 3x3 as list of values.
        """
        Ixx = Iyy = (1 / 12) * mass * (3 * r**2 + h**2)
        Izz = (1 / 2) * mass * r**2

        # Inertia matrix
        J_C = np.array([[Ixx, 0, 0], [0, Iyy, 0], [0, 0, Izz]])

        J1 = J_C / M

        # Cholesky decomposition
        L = np.linalg.cholesky(J1)

        return np.array([L[0, 0], L[1, 0], L[1, 1], L[2, 0], L[2, 1], L[2, 2]])

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

    def get_S(self, v):
        """
                |   0    -v3     v2 |
        S(v) =  |  v3      0    -v1 |
                | -v2     v1      0 |
        """
        v = v.flatten()
        return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

    @property
    def A1M(self):
        A = np.array(
            [
                [
                    -6.776660281601378126e-17,
                    -4.069248629204854639e-01,
                    2.905744659368497129e-01,
                    -1.897464878848385925e-15,
                    -3.334542096204833328e00,
                    -1.329837852169367229e00,
                ],
                [
                    -3.524072687206405985e-01,
                    2.034624314602428152e-01,
                    2.905744659368497129e-01,
                    2.887798165301998399e00,
                    -1.667271048102417330e00,
                    1.329837852169366563e00,
                ],
                [
                    3.524072687206407650e-01,
                    2.034624314602425932e-01,
                    2.905744659368496019e-01,
                    2.887798165302000175e00,
                    1.667271048102416220e00,
                    -1.329837852169366341e00,
                ],
                [
                    -1.107312451872372199e-16,
                    -4.069248629204853529e-01,
                    2.905744659368496574e-01,
                    4.429249807489488797e-16,
                    3.334542096204833328e00,
                    1.329837852169366563e00,
                ],
                [
                    -3.524072687206407095e-01,
                    2.034624314602426765e-01,
                    2.905744659368496019e-01,
                    -2.887798165301998843e00,
                    1.667271048102417108e00,
                    -1.329837852169367229e00,
                ],
                [
                    3.524072687206407650e-01,
                    2.034624314602426765e-01,
                    2.905744659368496019e-01,
                    -2.887798165301999287e00,
                    -1.667271048102416220e00,
                    1.329837852169366785e00,
                ],
            ]
        )
        A1 = A / M
        A1M = A1[3:, :]

        return A1M

    def get_initial_gesses(self):
        """
        Get the initial guesses for the parameters.

        Output:
            initial_params: The initial guesses for the parameters.
        """
        c = np.zeros((3, 1), dtype=float)
        initial_params_list = [self.L, c, self.A1M]
        initial_params = np.concatenate(
            [param.flatten() for param in initial_params_list]
        )

        L = self.L
        L = np.array(
            [
                [L[0], 0, 0],
                [L[1], L[2], 0],
                [L[3], L[4], L[5]],
            ]
        )

        print("Initial J1:\n", L @ L.T)
        print("\nInitial c:\n", c)
        print("\nInitial A1M:\n", self.A1M)
        print("- - - - - - - - - - - - - - - - - - - - - -")

        return initial_params

    def read_file(self, file_name):
        """
        Read the input file and store the data in the class variables.

        Parameters:
            file_name: The name of the input file.
        """
        with open(file_name) as fp:
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

    def peprocess_data(self):
        """Pecumpute the data to be used in the cost function."""

        self.w_diff = [
            ((self.w[i + 1] - self.w[i]) / (self.timestamp[i + 1] - self.timestamp[i]))
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

    def cost_function(self, params):
        """
        Cost function to minimize.

        Parameters:
            params: The parameters to optimize.

        Output:
            Cost
        """
        L_flat, c, A1M_flat = np.split(params, [6, 9])
        L = np.array(
            [
                [L_flat[0], 0, 0],
                [L_flat[1], L_flat[2], 0],
                [L_flat[3], L_flat[4], L_flat[5]],
            ]
        )

        A1M = A1M_flat.reshape((3, 6))

        cost = 0
        for i in range(len(self.timestamp) - 1):
            cost += (
                np.linalg.norm(
                    L @ L.T @ self.w_diff[i]
                    + self.get_S(self.w_avg[i]) @ L @ L.T @ self.w_avg[i]
                    - self.get_S(c) @ self.R_avg[i].T @ g
                    - A1M @ self.u_avg[i]
                )
                ** 2
            )

        self.cost = cost
        return cost

    def constraints(self, params):
        """
        Constraints function.

        Parameters:
            params: The parameters to optimize.

        Output:
            ||A1M||^2 = 1

        """
        _, _, A1M_flat = np.split(params, [6, 9])
        A1M = A1M_flat.reshape((3, 6))

        return np.linalg.norm(A1M) ** 2 - 1

    def solver_calback(self, _):
        """
        Callback function for the solver.

        Parameters:
            xk: The current solution.
        """
        self.iter += 1
        print(f"\t[Iteration {self.iter}]: Cost = {self.cost:.6e}")

    def solve(self):
        """Solve the optimization problem."""

        result = sopt.minimize(
            self.cost_function,
            self.get_initial_gesses(),
            method="SLSQP",
            constraints={"type": "eq", "fun": self.constraints},
            callback=self.solver_calback,
            options={"ftol": 1e-6, "disp": True},
        )

        optimized_L = result.x[:6]
        optimized_L = np.array(
            [
                [optimized_L[0], 0, 0],
                [optimized_L[1], optimized_L[2], 0],
                [optimized_L[3], optimized_L[4], optimized_L[5]],
            ]
        )
        optimized_c = result.x[6:9].reshape((3, 1))
        optimized_A1M = result.x[9:].reshape((3, 6))

        print("\n- - - - - - - - - - - - - - - - - - - - - -")
        print("Optimized J1:\n", optimized_L @ optimized_L.T)  # J = L @ L^T
        print("\nOptimized c:\n", optimized_c)
        print("\nOptimized A1M:\n", optimized_A1M)

        np.save("mats/est_J1.npy", optimized_L @ optimized_L.T)
        np.save("mats/est_A1M.npy", optimized_A1M)
        np.save("mats/est_c.npy", optimized_c)
        print("\nData saved to mats/est_*.npy")


def main(argv):
    if len(argv) != 2:
        print(f"Usage: python3 {argv[0]} <input_file>")
        return

    est = Estimator(file_name=argv[1])
    est.solve()


if __name__ == "__main__":
    main(sys.argv)
