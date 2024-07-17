# MIT License
# 
# Copyright (c) 2024 Andre Rebelo Teixeira
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import casadi as ca
import spatial_casadi as sc
import numpy as np
import matplotlib.pyplot as plt

class RotationDynamics:
    """
    Class for simulating the rotational dynamics of a rigid body.
    """

    def __init__(self, J: np.array, c: np.array, A: np.array, N: int = 40, T: float = 1/50):
        """
        Initialize the RotationDynamics class.
        
        Parameters:
        J (np.array): Inertia matrix.
        c (np.array): Vector c.
        A (np.array): Actuation matrix.
        N (int): Number of time steps.
        T (float): Time step duration.
        """
        self.J_ = J
        self.c_ = c
        self.A_ = A

        self.N = N
        self.T = T

    def setup(self):
        """
        Set up the optimization problem for the rotational dynamics.
        """
        self.opti = ca.Opti()

        # Define optimization variables
        self.w = self.opti.variable(3, self.N)  # angular velocity
        self.w_dot = self.opti.variable(3, self.N)  # angular acceleration
        self.q = self.opti.variable(4, self.N)  # quaternions
        self.u = self.opti.variable(6, self.N)  # actuation

        # Define initial conditions as parameters
        self.w_init = self.opti.parameter(3, 1)  # initial angular velocity
        self.w_dot_init = self.opti.parameter(3, 1)  # initial angular acceleration
        self.q_init = self.opti.parameter(4, 1)  # initial orientation

        # Set initial values
        self.opti.set_value(self.w_init, np.zeros(3))
        self.opti.set_value(self.w_dot_init, np.zeros(3))
        self.opti.set_value(self.q_init, sc.Rotation.from_euler('xyz', [0, 0, 0], degrees=True).as_quat(seq='xyzw'))

        # Define the desired rotation matrix as a parameter
        self.desired_r = self.opti.parameter(3, 3)
        desired_r = sc.Rotation.from_euler('xyz', [0, 0, 0], degrees=True).as_matrix()
        self.opti.set_value(self.desired_r, desired_r)

        # Define other parameters
        self.J = self.opti.parameter(3, 3)  # inertia matrix
        self.g = self.opti.parameter(3, 1)  # gravity
        self.A = self.opti.parameter(3, 6)  # actuation matrix
        self.skew_c = self.opti.parameter(3, 3)  # skew matrix of c

        self.opti.set_value(self.J, self.J_)
        self.opti.set_value(self.g, np.array([0, 0, -9.81]))
        self.opti.set_value(self.A, self.A_)
        self.opti.set_value(self.skew_c, ca.skew(self.c_))

        # Set initial conditions constraints
        self.opti.subject_to(self.w[:, 0] == self.w_init)
        self.opti.subject_to(self.w_dot[:, 0] == self.w_dot_init)
        self.opti.subject_to(self.q[:, 0] == self.q_init)

        self.final_objective_iter = self.N - 1

        # Ensure final angular velocity is zero
        self.opti.subject_to(self.w[:, self.final_objective_iter] == np.zeros(3))

        # Set initial guesses and bounds for optimization variables
        for i in range(0, self.N):
            self.opti.set_initial(self.q[:, i], sc.Rotation.from_euler('xyz', [0, 0, 0], degrees=True).as_quat(seq='xyzw'))

            self.opti.subject_to(self.opti.bounded(-5, self.w[0, i], 5))
            for j in range(6):
                # newtons of thrust applied by each rotor
                self.opti.subject_to(self.opti.bounded(-10, self.u[j, i], 10))

        # Define the dynamics and constraints
        for i in range(0, self.N):
            c_r = sc.Rotation.from_quat(self.q[:, i], seq="xyzw").as_matrix()
            self.opti.subject_to(self.J @ self.w_dot[:, i] + ca.skew(self.w[:, i]) @ self.J @ self.w[:, i] - self.skew_c @ c_r.T @ self.g == self.A @ self.u[:, i])

        # Define the integration constraints for quaternions and angular velocity
        for i in range(0, self.N-1):
            self.opti.subject_to(self.q[:, i+1] == self.quaternion_integration(self.q[:, i], self.w[:, i], self.T))
            self.opti.subject_to(self.w[:, i+1] == self.w[:, i] + self.T * self.w_dot[:, i])

        # Define the cost function
        self.cost_function = 0
        for i in range(1, self.N):
            c_r = sc.Rotation.from_quat(self.q[:, i], seq='xyzw').as_matrix()
            self.cost_function += 0.01 * ca.sumsqr(self.u[:, i]) + 1000 * ca.trace(ca.MX.eye(3) - (self.desired_r @ c_r.T))

        self.opti.minimize(self.cost_function)


        # Solver options
        s_opts = {
            'print_time': 0,
            'verbose': False
        }

        p_opts = {'print_level': 0 , 
            'warm_start_bound_push': 1e-4,
            'warm_start_bound_frac': 1e-4,
            'warm_start_slack_bound_frac': 1e-4, 
            'warm_start_slack_bound_push': 1e-4,
            'warm_start_mult_bound_push': 1e-4, 
            'tol' : 1e-4, 
            'acceptable_iter' : 7, 

        }

        self.opti.solver('ipopt', s_opts, p_opts)

    def step(self, w, w_dot, q, desired_r: sc.Rotation = [0, 0, 0]):
        """
        Perform a step of the optimization with current states and desired rotation.

        Parameters:
        w: Current angular velocity.
        w_dot: Current angular acceleration.
        q: Current quaternion.
        desired_r: Desired rotation.

        Returns:
        Solution of the optimization problem.
        """
        desired_rotation_matrix = sc.Rotation.from_euler('xyz', desired_r, degrees=True).as_matrix()
        self.opti.set_value(self.w_init, w)
        self.opti.set_value(self.w_dot_init, w_dot)
        self.opti.set_value(self.q_init, q)
        self.opti.set_value(self.desired_r, desired_rotation_matrix)
        
        if hasattr(self, 'sol'):
            self.opti.set_initial(self.q, self.sol.value(self.q))
            self.opti.set_initial(self.w, self.sol.value(self.w))
            self.opti.set_initial(self.w_dot, self.sol.value(self.w_dot))
            self.opti.set_initial(self.u, self.sol.value(self.u))
        
        self.sol = self.opti.solve()
        return self.sol 

    def quaternion_integration(self, q, w, t):
        """
        Integrate quaternion q using angular velocity w over time t.

        Parameters:
        q: Quaternion.
        w: Angular velocity.
        t: Time step.

        Returns:
        Integrated quaternion.
        """
        epsilon = 1e-20  # to avoid division by zero
        w_ = w + epsilon
        w_norm = ca.norm_2(w_)

        p = ca.vertcat(
            w / (w_norm + epsilon) * ca.sin((w_norm + epsilon) * t / 2),
            ca.cos(w_norm * t / 2)
        )

        q_ = ca.vertcat(
            ca.horzcat(q[3], q[2], -q[1], q[0]),
            ca.horzcat(-q[2], q[3], q[0], q[1]),
            ca.horzcat(q[1], -q[0], q[3], q[2]),
            ca.horzcat(-q[0], -q[1], -q[2], q[3])
        )

        return q_ @ p

