#!/usr/bin/env python3

import sys
import io
import numpy as np
import numpy.linalg as la
import scipy.integrate as intgr
import scipy.spatial.transform as trf
import matplotlib.pyplot as plt


def S(v):
    """Skew-symmetric matrix"""
    return np.array([[0,    -v[2], v[1]],
                     [v[2],  0,   -v[0]],
                     [-v[1], v[0], 0]])

class Simulator:
    ## Synthetic parameters of Space CoBot
    m = 5.8 # Kg
    # simplification: consider a cylinder
    r = 0.25 # m
    h = 0.15 # m
    J = np.diag([m*(3*r*r + h*h)/12,
                 m*(3*r*r + h*h)/12,
                 m*r*r/2])
    Jinv = la.inv(J)
    c = np.array([0.01, 0.02, 0.05])  # center of mass
    g = np.array([0, 0, -9.8])  # gravity vector
    mu = 0.01  # friction coefficient
    # Actuation matrix
    Atxt = """
0.000000000000000000e+00 -7.094064799162224100e-01 7.094064799162225210e-01 -1.003171929053524623e-16 -7.094064799162221879e-01 7.094064799162224100e-01
-8.191520442889916875e-01 4.095760221444959548e-01 4.095760221444956772e-01 -8.191520442889916875e-01 4.095760221444962323e-01 4.095760221444958438e-01
5.735764363510461594e-01 5.735764363510461594e-01 5.735764363510461594e-01 5.735764363510461594e-01 5.735764363510461594e-01 5.735764363510461594e-01
0.000000000000000000e+00 8.657114718190686564e-02 8.657114718190689340e-02 1.224202867855199215e-17 -8.657114718190685176e-02 -8.657114718190686564e-02
-9.996375025905729350e-02 -4.998187512952866063e-02 4.998187512952862593e-02 9.996375025905729350e-02 4.998187512952868838e-02 -4.998187512952864675e-02
-1.253285627227282151e-01 1.253285627227282151e-01 -1.253285627227282151e-01 1.253285627227282151e-01 -1.253285627227282151e-01 1.253285627227282151e-01
"""
    A = np.loadtxt(io.StringIO(Atxt))

    def dyn(self, state, u):
        """Continuous time dynamics with explicit rotation matrix"""
        (x, v, R, w) = state
        F = self.A[0:3,:]@u
        M = self.A[3:6,:]@u
        x_dot = v
        v_dot = R@F
        R_dot = R@S(w)
        w_dot = self.Jinv@(M + S(self.c)@R.T@self.g - S(w)@self.J@w - self.mu*w)
        return (x_dot, v_dot, R_dot, w_dot)

    def ode(self, s0, u, delta_t):
        def flat(s):
            (x, v, R, w) = s
            return np.hstack([x, v, R.flatten(), w])
        def unflat(y):
            x = y[0:3]
            v = y[3:6]
            R = y[6:15].reshape(3,3)
            w = y[15:18]
            return (x, v, R, w)
        def f(t, y):
            s = unflat(y)
            s_dot = self.dyn(s, u)
            return flat(s_dot)
        y0 = flat(s0)
        ret = intgr.solve_ivp(f, (0,delta_t), y0)
        return unflat(ret.y[:,-1])
        



def test1():
    """Single step ODE test"""
    sim = Simulator()
    initial = (np.zeros(3), np.zeros(3), np.eye(3), np.zeros(3))
    u = np.zeros(6)
    print(sim.dyn(initial, u))
    final = sim.ode(initial, u, 1.0)
    print(final)

def test2(steps=200):
    """Trajectory simulated with zero actuation"""
    sim = Simulator()
    initial = (np.zeros(3), np.zeros(3), np.eye(3), np.zeros(3))
    #u = np.zeros(6)
    #u[0] = 0.01
    u = la.inv(sim.A)@np.array([0,0,0, 0,0,0])
    traj = [initial]
    s = initial
    for i in range(steps):
        s = sim.ode(s, u, 0.1)
        traj.append(s)
    xs = np.vstack([s[0] for s in traj])
    vs = np.vstack([s[1] for s in traj])
    Rs = np.array([s[2] for s in traj])
    es = trf.Rotation.from_matrix(Rs).as_euler("ZYX")
    ws = np.vstack([s[3] for s in traj])
    plt.subplot(411)
    plt.plot(xs[:,0], label="x")
    plt.plot(xs[:,1], label="y")
    plt.plot(xs[:,2], label="z")
    plt.legend()
    plt.subplot(412)
    plt.plot(vs[:,0], label="vx")
    plt.plot(vs[:,1], label="vy")
    plt.plot(vs[:,2], label="vz")
    plt.legend()
    plt.subplot(413)
    plt.plot(es[:,0], label="Yaw")
    plt.plot(es[:,1], label="Pitch")
    plt.plot(es[:,2], label="Roll")
    plt.legend()
    plt.subplot(414)
    plt.plot(ws[:,0], label="wx")
    plt.plot(ws[:,1], label="wy")
    plt.plot(ws[:,2], label="wz")
    plt.legend()
    plt.show()


def test3():
    """Trajectory simulated with random step-wise actuation"""
    sim = Simulator()
    initial = (np.zeros(3), np.zeros(3), np.eye(3), np.zeros(3))
    steps = np.hstack([np.zeros((10,3)), 0.1*np.random.randn(10,3)])
    M = np.vstack([np.tile(steps[i,:], (20,1)) for i in range(len(steps))]).T
    u = la.inv(sim.A)@M
    traj = [initial]
    s = initial
    for i in range(u.shape[1]):
        s = sim.ode(s, u[:,i], 0.1)
        traj.append(s)
    xs = np.vstack([s[0] for s in traj])
    vs = np.vstack([s[1] for s in traj])
    Rs = np.array([s[2] for s in traj])
    es = trf.Rotation.from_matrix(Rs).as_euler("ZYX")
    ws = np.vstack([s[3] for s in traj])
    plt.subplot(511)
    plt.plot(xs[:,0], label="x")
    plt.plot(xs[:,1], label="y")
    plt.plot(xs[:,2], label="z")
    plt.legend()
    plt.subplot(512)
    plt.plot(vs[:,0], label="vx")
    plt.plot(vs[:,1], label="vy")
    plt.plot(vs[:,2], label="vz")
    plt.legend()
    plt.subplot(513)
    plt.plot(es[:,0], label="Yaw")
    plt.plot(es[:,1], label="Pitch")
    plt.plot(es[:,2], label="Roll")
    plt.legend()
    plt.subplot(514)
    plt.plot(ws[:,0], label="wx")
    plt.plot(ws[:,1], label="wy")
    plt.plot(ws[:,2], label="wz")
    plt.legend()
    plt.subplot(515)
    plt.plot(u[0,:], label="u1")
    plt.plot(u[1,:], label="u2")
    plt.plot(u[2,:], label="u3")
    plt.plot(u[3,:], label="u4")
    plt.plot(u[4,:], label="u5")
    plt.plot(u[5,:], label="u6")
    plt.show()













    
def main(argv):
    g = globals()
    if len(argv)>1:
        f = g[argv[1]]
        f()
    else:
        print(f"Usage: {argv[0]} <test>\nwhere <test> is one of:")
        for fn in g:
            if fn.startswith("test"):
                print(f"  {fn}")


if __name__=='__main__':
    main(sys.argv)


# EOF