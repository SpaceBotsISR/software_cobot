import numpy as np
import scipy.optimize as sopt
from scipy.spatial.transform import Rotation

"""
w(3x1) - angular velocity

R(3x3) - Atitute expressed as a rotation matrix
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

u(1x6) - actuation vector

samples = {(wk_dot, wk, Rk, uk)}

parameters = {(L, c, A1m)}


    minimize      sum(||f(x)||^2)  
s.t. L, c, A1M

f(x) = 1/dt * L @ L.T(w_k+1-w_k) + S(w_k,k+1) @ L @ L.T @ w_k,k+1) - S(c) R_k,k+1 * g - A_1M_(u_k,k+1) ||^2 

with

    w_k,k+1 = (w_k+1 + w_k)/2

    u_k,k+1 = (u_k+1 + u_k)/2

    R_k,k+1 = USV (USV is the SVD decomposition of (R_k,k+1) and S = diag(1,1,det(U)det(V))
"""

G = 9.81 # gravity acceleration
g = np.array([[0], [0], [-G]], dtype=float)
m = 3 # mass of Cobot
m_batt = 0.5 # mass of the battery
M = m + m_batt # total mass
r = 0.25 # radius of Cobot
h = 0.10 # height of Cobot

class Estimator:
    def __init__(self, file_name="input_data.txt"):
        self.timestamp = []
        self.w = []
        self.R = []
        self.u = []

        self.read_file(file_name)

    def print(self):
        for i in range(len(self.timestamp)):
            print("\n - - - - - - - - - - - - - - - - - - - - - - ")
            print(f"t:\n {self.timestamp[i]}")
            print(f"w:\n{self.w[i]}")
            print(f"R:\n{self.R[i]}")
            print(f"u:\n{self.u[i]}")

    def get_L(self, mass = M):
        """
        Get lower triangular matrix 3x3 from the moment of inertia.

        Parameters:
        - mass: The total mass of Cobot.

        Output:
        - L: Lower triangular matrix 3x3 as list of values.
        """
        Ixx = Iyy = (1/12) * mass * (3 * r**2 + h**2)
        Izz = (1/2) * mass * r**2

        # Inertia matrix
        J_C = np.array([
            [Ixx, 0, 0],
            [0, Iyy, 0],
            [0, 0, Izz]
        ])

        J1 = J_C / M

        # Cholesky decomposition
        L = np.linalg.cholesky(J1)

        return np.array([L[0,0], L[1,0], L[1,1], L[2,0], L[2,1] , L[2,2]])


    """
    v 0 0
    v v 0
    v v v
    """

    def get_R(self, line):
        """
        Convert a quaternion to a 3x3 rotation matrix.
    
        Parameters:
        - quaternion: The quaternion in the format (w, x, y, z).

        Output:
        - R: The 3x3 rotation matrix.
        """
        w, x, y, z = [float(x) for x in line.replace(" ", ",").split(",")]

        R = Rotation.from_quat([w, x, y, z]).as_matrix()

        return R

    def parse_line(self, line):
        line = line.replace("\n", "").replace(" ", ",")
        numbers = line.strip().split(",")

        return np.array([[float(n) for n in numbers if n != '']])

    def read_file(self, file_name):
        with open(file_name) as fp:
            contents = fp.read()
            measurements = contents.split("\n\n")

            for m in measurements:
                m = m.split("\n")
                self.timestamp.append(float(m[0].strip()))
                self.w.append(self.parse_line(m[1]).T)
                self.R.append(self.get_R(m[2]))
                self.u.append(self.parse_line(m[3]).T)

    def get_S(self, v):
        """
                |   0    -v3     v2 |
        S(v) =  |  v3      0    -v1 |
                | -v2     v1      0 |
        """
        v = v.flatten()
        return np.array([[0, -v[2], v[1]], 
                         [v[2], 0, -v[0]], 
                         [-v[1], v[0], 0]])

    def cost_function(self, params):
        residuals = []
        L_flat, c, A1M_flat = np.split(params, [6, 9])
        L = np.array([[0, 0, 0],
                     [L_flat[0], L_flat[1], 0],
                     [L_flat[3], L_flat[4], L_flat[5]]])

        A1M = A1M_flat.reshape((3, 6))


        for i in range(len(self.timestamp) - 1):
            w_diff = (self.w[i + 1] - self.w[i]) / self.timestamp[i]
            w_avg = (self.w[i + 1] + self.w[i]) / 2
            u_avg = (self.u[i + 1] + self.u[i]) / 2
            R_svd = np.linalg.svd(self.R[i + 1])
            R_avg = R_svd[0] @ np.diag([1, 1, np.linalg.det(R_svd[0]) * np.linalg.det(R_svd[2])]) @ R_svd[2]

            model = (L @ L.T @ w_diff +
                self.get_S(w_avg) @ L @ L.T @ w_avg -
                self.get_S(c) @ R_avg.T @ g -
                A1M @ u_avg)


            residuals.append(model.flatten())

        return np.concatenate(residuals)
    
def main():
    est = Estimator()

    # Initial guesses
    L = est.get_L()
    c = np.zeros((3, 1), dtype=float)
    A = np.array([
        [-6.776660281601378126e-17, -4.069248629204854639e-01, 2.905744659368497129e-01, -1.897464878848385925e-15, -3.334542096204833328e+00, -1.329837852169367229e+00],
        [-3.524072687206405985e-01, 2.034624314602428152e-01, 2.905744659368497129e-01, 2.887798165301998399e+00, -1.667271048102417330e+00, 1.329837852169366563e+00],
        [3.524072687206407650e-01, 2.034624314602425932e-01, 2.905744659368496019e-01, 2.887798165302000175e+00, 1.667271048102416220e+00, -1.329837852169366341e+00],
        [-1.107312451872372199e-16, -4.069248629204853529e-01, 2.905744659368496574e-01, 4.429249807489488797e-16, 3.334542096204833328e+00, 1.329837852169366563e+00],
        [-3.524072687206407095e-01, 2.034624314602426765e-01, 2.905744659368496019e-01, -2.887798165301998843e+00, 1.667271048102417108e+00, -1.329837852169367229e+00],
        [3.524072687206407650e-01, 2.034624314602426765e-01, 2.905744659368496019e-01, -2.887798165301999287e+00, -1.667271048102416220e+00, 1.329837852169366785e+00]
    ])
    A1 = A / M
    A1M = A1[3:, :]

    initial_params_list = [L, c, A1M]
    initial_params = np.concatenate([param.flatten() for param in initial_params_list])

    result = sopt.least_squares(est.cost_function, initial_params, verbose = 2)

    optimized_L = result.x[:9].reshape((3, 3))
    optimized_c = result.x[9:12].reshape((3, 1))
    optimized_A1M = result.x[12:].reshape((3, 6))

    print("Optimized L:")
    print(optimized_L)
    print("\nOptimized c:")
    print(optimized_c)
    print("\nOptimized A1M:")
    print(optimized_A1M)


if __name__ == "__main__":
    main()
