
import numpy as np
import numpy.linalg as la
import numpy.random as rnd
import matplotlib.pyplot as plt

I = np.eye(3)

if False:
    J  = np.array([[0.04533008, 0,          0],
                   [0,          0.04166828, 0],
                   [0,          0,          0.05198795]])
else:
    J  = np.array([[ 0.04533008,   0.000710774, -0.001934362],
                   [ 0.000710774,  0.04166828,  -0.003146458],
                   [-0.001934362, -0.003146458,  0.05198795]])

Ji = la.inv(J)

m = 6.047

A = np.array([
[1.783331653052994098e-17, 7.139032682815533992e-02, 5.097797648014908145e-02, -4.755551074807984466e-17, -5.982942815153184093e-04, 3.351770482861808897e-01],
[6.182583661765626848e-02, -3.569516341407769078e-02, 5.097797648014910921e-02, 1.817854216429663217e-01, -3.150657132792836213e-01, 1.143561957336149126e-01],
[-6.182583661765628236e-02, -3.569516341407764221e-02, 5.097797648014903288e-02, 1.817854216429664327e-01, 3.150657132792836213e-01, -1.143561957336148849e-01],
[1.833979422398007340e-17, 7.139032682815532604e-02, 5.097797648014906757e-02, -2.683783748816093577e-17, 5.982942815153671984e-04, -3.351770482861808897e-01],
[6.182583661765623378e-02, -3.569516341407767690e-02, 5.097797648014905370e-02, -1.817854216429662384e-01, 3.150657132792837323e-01, -1.143561957336149265e-01],
[-6.182583661765624766e-02, -3.569516341407766302e-02, 5.097797648014906757e-02, -1.817854216429664604e-01, -3.150657132792837323e-01, 1.143561957336149126e-01]])

Ai = la.inv(A)

kx = 0.1
kv = 2*np.sqrt(kx)  # for double pole
kR = 0.1
kO = np.sqrt(np.diag(J).max()*kR/2)
T = 0.01

# # experiment: scaling down gains
# scale = 0.5
# kx *= scale**2
# kv *= scale
# kR *= scale**2
# kO *= scale

def skew(x):
    return np.array([[  0,  -x[2],  x[1]],
                     [ x[2],  0,   -x[0]],
                     [-x[1], x[0],   0]])

def unskew(X):
    return np.array([X[2,1], X[0,2], X[1,0]])

def rodrigues(k, theta):
    K = skew(k)
    return I + np.sin(theta)*K + (1-np.cos(theta))*np.dot(K, K)

def expr(omega):
    n = la.norm(omega)
    return rodrigues(omega/n, n) if n!=0 else np.eye(3)

def logr(R):
    phi = np.arccos(np.clip((R.trace() - 1)/2, -1 , 1))
    return unskew((R-R.T)/(2*np.sinc(phi/np.pi)))


def step(x, v, R, O, xd, vd, Rd, Od):
    # Error signals
    ex = x - xd
    ev = v - vd
    eR = 1/(2*np.sqrt(1+np.trace(np.dot(Rd.T, R)))) * unskew(np.dot(Rd.T, R) - np.dot(R.T, Rd))
    eO = O - reduce(np.dot, (R.T, Rd, Od))
    # Controller
    p = -kx*ex - kv*ev
    F = m*np.dot(R.T, p)
    M = -kR*eR - kO*eO  # MISSING: some terms from Lee12 eq (23)
    # Actuation limits -- DISABLED
    # u = np.dot(Ai, concatenate((F, M)))
    # l = min(1, np.min(10/np.abs(u)))
    # print "l =", l
    # F *= l
    # M *= l
    # System
    x = x + T*v
    v = v + (T/m)*np.dot(R, F)
    #R = np.dot(R, rodrigues(O/la.norm(O), la.norm(T*O)))
    R = np.dot(R, I+T*skew(O))
    O = O - T*np.dot(Ji, np.cross(O, np.dot(J, O))) + T*np.dot(Ji, M)
    #print "R sanity:", la.det(R), la.norm(np.dot(R.T, R)-I)
    return (x, v, R, O)

def sim(Tf, x0, v0, R0, O0, xd, vd, Rd, Od):
    t = 0
    x = x0
    v = v0
    R = R0
    O = O0
    data = []
    for t in np.arange(0, Tf+T/2, T):
        (x, v, R, O) = step(x, v, R, O, xd, vd, Rd, Od)
        ex = x - xd
        ev = v - vd
        eR = 1/(2*np.sqrt(1+np.trace(np.dot(Rd.T, R)))) * unskew(np.dot(Rd.T, R) - np.dot(R.T, Rd))
        eO = O - reduce(np.dot, (R.T, Rd, Od))
        data.append(np.concatenate(([t], ex, ev, eR, eO)))
    print "reality check:", la.det(R), la.norm(np.dot(R.T, R)-I)
    return np.array(data)

def plot_sim(data):
    plt.figure(figsize=(14,6))
    plt.subplot(221)
    plt.title("ex")
    plt.plot(data[:,0], data[:,1:4])
    plt.subplot(223)
    plt.title("ev")
    plt.plot(data[:,0], data[:,4:7])
    plt.subplot(222)
    plt.title("eR")
    plt.plot(data[:,0], data[:,7:10])
    plt.subplot(224)
    plt.title("eO")
    plt.plot(data[:,0], data[:,10:13])

def random_R():
    k = rnd.uniform(-1, 1, 3)
    k = k / la.norm(k)
    a = rnd.uniform(0, 2*np.pi)
    return rodrigues(k, a)
    
def random_run():
    xd = rnd.uniform(-1, 1, 3)
    vd = np.zeros(3)
    Rd = random_R()
    Od = np.zeros(3)
    x0 = rnd.uniform(-1, 1, 3)
    v0 = 0*rnd.randn(3)
    R0 = random_R()
    O0 = 0*rnd.randn(3)
    data = sim(200, x0, v0, R0, O0, xd, vd, Rd, Od)
    plot_sim(data)



       #  x  y  z   Ox Oy Oz
PATH1 = [(0, 0, 0,  0, 0, 0),
         (0, 0, 1,  0, 0, 1),
         (1, 0, 1,  0, 0, 1),
         (1, 1, 1,  1, 0, 0),
         (1, 1, 0,  0, 0, 0)]

VREF = 0.05  # translation speed
OREF = 0.02  # rotation speed
    
def gen_from_path(path=PATH1):
    out = []
    for i in xrange(len(path)-1):
        pi = np.array(path[i])
        pf = np.array(path[i+1])
        xi, oi = pi[0:3], pi[3:6]
        xf, of = pf[0:3], pf[3:6]
        d = la.norm(xf-xi)
        r = la.norm(of-oi)
        tt = max(d/VREF, r/OREF)
        for t in np.arange(0, tt, T):
            xd = xi + (xf-xi)*t/tt
            vd = (xf-xi)/tt
            Rd = expr(oi + (of-oi)*t/tt)
            Od = (of-oi)/tt
            out.append((xd, vd, Rd, Od))
    return out

        #  t   x  y  z   Ox Oy Oz
TRAJ1 = [( 0,  0, 0, 0,  0, 0, 0),
         (20,  0, 0, 1,  0, 0, 1),
         (40,  1, 0, 1,  0, 0, 1),
         (60,  1, 1, 1,  1, 0, 0),
         (80,  1, 1, 0,  0, 0, 0)]

def gen_from_traj(traj=TRAJ1):
    out = []
    for i in xrange(len(traj)-1):
        ti, pi = traj[i][0],   np.array(traj[i][1:])
        tf, pf = traj[i+1][0], np.array(traj[i+1][1:])
        xi, oi = pi[0:3], pi[3:6]
        xf, of = pf[0:3], pf[3:6]
        tt = float(tf - ti)
        #print tt, xi, xf, (xf-xi)/tt
        for t in np.arange(0, tt, T):
            xd = xi + (xf-xi)*t/tt
            vd = (xf-xi)/tt
            Rd = expr(oi + (of-oi)*t/tt)
            Od = (of-oi)/tt
            out.append((xd, vd, Rd, Od))
    return out

def sim_traj(traj):
    refs = []
    data = []
    (x, v, R, O) = traj[0]
    for i in xrange(1, len(traj)):
        t = i*T
        (xd, vd, Rd, Od) = traj[i]
        (x, v, R, O) = step(x, v, R, O, xd, vd, Rd, Od)
        ex = x - xd
        ev = v - vd
        eR = 1/(2*np.sqrt(1+np.trace(np.dot(Rd.T, R)))) * unskew(np.dot(Rd.T, R) - np.dot(R.T, Rd))
        eO = O - reduce(np.dot, (R.T, Rd, Od))
        #                            0   1  4  7           16  19  22  25  28 (31)
        data.append(np.concatenate(([t], x, v, R.flatten(), O, ex, ev, eR, eO)))
        #                            0   1   4   7             16 (19)
        refs.append(np.concatenate(([t], xd, vd, Rd.flatten(), Od)))
    print "reality check:", la.det(R), la.norm(np.dot(R.T, R)-I)
    return (np.array(data), np.array(refs))
    
def plot_sim_traj(data, refs):
    plt.figure(figsize=(14,6))
    plt.subplot(221)
    plt.title("ex")
    plt.plot(data[:,0], data[:,19:22])
    plt.subplot(223)
    plt.title("ev")
    plt.plot(data[:,0], data[:,22:25])
    plt.subplot(222)
    plt.title("eR")
    plt.plot(data[:,0], data[:,25:28])
    plt.subplot(224)
    plt.title("eO")
    plt.plot(data[:,0], data[:,28:31])
    #
    plt.figure(figsize=(14,6))
    plt.subplot(221)
    plt.title("x")
    plt.plot(refs[:,0], refs[:,1:4], '--')
    plt.plot(data[:,0], data[:,1:4], '-')
    plt.subplot(223)
    plt.title("v")
    plt.plot(refs[:,0], refs[:,4:7], '--')
    #plt.plot(data[:,0], data[:,4:7], '-')
    plt.subplot(222)
    plt.title("log(R)")
    refs_logR = np.array([logr(rf.reshape((3,3))) for rf in refs[:,7:16]])
    data_logR = np.array([logr(rf.reshape((3,3))) for rf in data[:,7:16]])
    plt.plot(refs[:,0], refs_logR, '--')
    plt.plot(data[:,0], data_logR, '-')
    plt.subplot(224)
    plt.title("O")
    plt.plot(refs[:,0], refs[:,16:19], '--')
    plt.plot(data[:,0], data[:,16:19], '-')

def path_run():
    traj = gen_from_path()
    (data, refs) = sim_traj(traj)
    plot_sim_traj(data, refs)

def traj_run():
    traj = gen_from_traj()
    (data, refs) = sim_traj(traj)
    plot_sim_traj(data, refs)



# EOF
