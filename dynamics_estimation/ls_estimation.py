import numpy as np
import scipy.optimize as sopt

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
@
with 
           | 0     -w3    w2 |
    S(w) = | w3     0    -w1 |
           |-w2     w1    0  |

    w_k,k+1 = (w_k+1 + w_k)/2

    u_k,k+1 = (u_k+1 + u_k)/2

    R_k,k+1 = USV (USV is the SVD decomposition of (R_k,k+1) and S = diag(1,1,det(U)det(V))
"""

G = 9.81
g = np.array([[0], [0], [-G]], dtype=float)


class Estimator:
    def __init__(self, file_name="data.txt"):
        self.timestamp = []
        self.w_dot = []
        self.w = []
        self.R = []
        self.u = []

        self.L = np.zeros((3, 3), dtype=float)
        self.c = np.zeros((3, 1), dtype=float)
        self.A1M = np.zeros((6, 6), dtype=float)

        self.read_file(file_name)

    def print(self):
        for i in range(len(self.timestamp)):
            print("\n - - - - - - - - - - - - - - - - - - - - - - ")
            print(f"t:\n {self.timestamp[i]}")
            print(f"w_dot:\n{self.w_dot[i]}")
            print(f"w:\n{self.w[i]}")
            print(f"R:\n{self.R[i]}")
            print(f"u:\n{self.u[i]}")

    def get_R(self, line):
        """
                                             | cos(alpha)cos(betha)    cos(alpha)sin(betha)sin(gamma) - sin(alpha)cos(gamma)    cos(alpha)sin(betha)cos(gamma) + sin(alpha)sin(gamma) |
        R = Rz(alpha).Ry(betha).Rx(gamma) =  | sin(alpha)cos(betha)    sin(alpha)sin(betha)sin(gamma) + cos(alpha)cos(gamma)    sin(alpha)sin(betha)cos(gamma) - cos(alpha)sin(gamma) |
                                             |       -sin(betha)                            cos(betha)sin(gamma)                                     cos(betha)cos(gamma)             |
        """
        alpha, betha, gamma = line.replace(" ", ",").split(",")
        cos = np.cos
        sin = np.sin
        R = np.array(
            [
                [
                    cos(alpha) * cos(betha),
                    cos(alpha) * sin(betha) * sin(gamma) - sin(alpha) * cos(gamma),
                    cos(alpha) * sin(betha) * cos(gamma) + sin(alpha) * sin(gamma),
                ],
                [
                    sin(alpha) * cos(betha),
                    sin(alpha) * sin(betha) * sin(gamma) + cos(alpha) * cos(gamma),
                    sin(alpha) * sin(betha) * cos(gamma) - cos(alpha) * sin(gamma),
                ],
                [-sin(betha), cos(betha) * sin(gamma), cos(betha) * cos(gamma)],
            ]
        )

    def read_file(self, file_name):
        with open(file_name) as fp:
            contents = fp.read()
            measurements = contents.split("\n\n")

            for m in measurements:
                m = m.split("\n")
                self.timestamp.append(float(m[0].strip()))
                self.w_dot.append(self.parse_line(m[1]).T)
                self.w.append(self.parse_line(m[2]).T)
                self.R.append(self.get_R(m[3]))
                self.u.append(self.parse_line(m[4]).T)

    def get_S(v):
        return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

    def calc_residuals(self):
        residuals = []

        for i in range(len(self.timestamp)):
            # TODO: check the R part of the model
            model = (
                1 / self.timestamp[i] * self.L @ self.L.T @ (self.w[i + 1] - self.w[i])
            )
            +self.calc_S((self.w[i + 1] + self.w[i]) / 2) @ self.L @ self.L.T @ (
                (self.w[i + 1] + self.w[i]) / 2
            )
            -self.calc_S(self.c) @ self.R[i].T @ g - self.A1M(
                (self.u[i + 1] + self.u[i]) / 2
            )


def main():
    est = Estimator()
    est.print()


if __name__ == "__main__":
    main()
