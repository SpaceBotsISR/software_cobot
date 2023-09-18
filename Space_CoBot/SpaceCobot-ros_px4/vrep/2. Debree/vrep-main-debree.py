import vrep
import numpy as np
import time
import math
import sys
from sim1 import *
import transforms3d.euler as t3d

from util import *
from dataIO import *
from numpy.linalg import norm
from numpy.random import randn
from numpy.random import normal
import scipy.linalg as la
import argparse as ap

try:
    parser = ap.ArgumentParser(description='Receive vehicle ID for each instance.')
    parser.add_argument('VehicleID', type=int, help='Vehicle number in simulation.')
    args = parser.parse_args()
    VehicleID = args.VehicleID
except:
    sys.exit()


# Initial setups
np.set_printoptions(precision=8)

# Connect to Server and set synchronous operation
vrep.simxFinish(-1)
clientID=vrep.simxStart('127.0.0.1',19999+VehicleID,True,True,5000,5) 
while (clientID ) == -1:
   print "Could not start remote server. Is simulation running?"
   clientID=vrep.simxStart('127.0.0.1',19999+VehicleID,True,True,5000,5)
vrep.simxSynchronous(clientID,True)
vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot)

# Initial data read
n, n, n, n, n, n,begin_control_time,dt= readData(clientID,vrep.simx_opmode_streaming,VehicleID)
prevST= begin_control_time

# j=j+1
# if norm(posError) < ErrorThreshold and norm(rotError) < ErrorRThreshold and j > 20 and pospointer <= TrajCount and t > reftime[pospointer]:
#     traj = gen_traj(np.concatenate((reftrajectory[h:h+1],reforientation[h:h+1]),1))
#     h=h+1
#     pospointer=pospointer+1
#     j=0
#Trajectory for reference, 12pts, scene 1
#reftrajectory = [[0,0,0.85], [0,-1,0.85], [0,-1,0.3],[0,1,0.3],[1,1,1.5],[1,2,1.5],[1,3.5,1.5],[4,3.5,1],[4,0,1],[3,0,1],[2,0,1],[0,0,0.85]]
#reforientation = [[np.deg2rad(0),np.deg2rad(0),np.deg2rad(0)],[np.deg2rad(0),np.deg2rad(0),np.deg2rad(0)],
#                    [np.deg2rad(0),np.deg2rad(0),np.deg2rad(0)],[np.deg2rad(0),np.deg2rad(0),np.deg2rad(90)],
#                    [np.deg2rad(0),np.deg2rad(70),np.deg2rad(0)],[np.deg2rad(0),np.deg2rad(140),np.deg2rad(0)],
#                    [np.deg2rad(0),np.deg2rad(140),np.deg2rad(0)],[np.deg2rad(90),np.deg2rad(0),np.deg2rad(0)],
#                    [np.deg2rad(90),np.deg2rad(0),np.deg2rad(0)],[np.deg2rad(90),np.deg2rad(0),np.deg2rad(0)],
#                    [np.deg2rad(0),np.deg2rad(0),np.deg2rad(0)],[np.deg2rad(0),np.deg2rad(0),np.deg2rad(0)]]

#Trajectory for reference, 4pts, scene 2
#reftrajectory = [[0,0,1], [0,0,2], [0,1,2]]
#reforientation = [[np.deg2rad(0),np.deg2rad(0),np.deg2rad(0)],[np.deg2rad(0),np.deg2rad(0),np.deg2rad(0)],
#                    [np.deg2rad(0),np.deg2rad(0),np.deg2rad(0)]]

# Trajectory for reference, 1st run
#WaypointTime = [0,20,40,60,80,100,120]
#reftrajectory = [[0,0,0.1], [0,0,1], [1,0,1],[1,0,1],[2,0,1], [1,1,1],[1,1,0.1]]
#reforientation = [[0,0,0], [0,0,1], [np.deg2rad(90),0,0],[np.deg2rad(90),0,0], 
#                    [0,np.deg2rad(90),0], [0,0,np.deg2rad(90)],[0,0,0]]

# Solo holonomic          t  x y z  - a b g
# time_traj=5
# waypoints = [(time_traj*0, 0,0,0.1, 0,0,0), 
#              (time_traj*1, 0,0,1, 0,0,0),
#              (time_traj*2, 1,0,1, np.deg2rad(0),0,0),
#              (time_traj*3, 1,1,1, np.deg2rad(0),0,0),
#              (time_traj*4, 1,0,1, np.deg2rad(90),0,0),
#              (time_traj*5, 2,0,1, 0,np.deg2rad(90),0),
#              (time_traj*6, 1,1,1, 0,0,np.deg2rad(90)),
#              (time_traj*7, 1,1,0.5, 0,0,0),
#              (time_traj*8, 1,1,0.1, 0,0,0)]

# Solo narrow passage   t  x y z  - a b g
# time_traj=2.5
# waypoints = [(time_traj*0, 0,0,0.1, 0,0,0), 
#              (time_traj*1, 0,0,0.5, 0,0,0),
#              (time_traj*2, 0,0.5,0.5, 0,np.deg2rad(90),0),
#              (time_traj*3, 0,1,0.5, 0,np.deg2rad(90),0),
#              (time_traj*4, 0,2.15,0.25, np.deg2rad(90),0,0),
#              (time_traj*5, 0.1,2.15,0.25, np.deg2rad(90),np.deg2rad(0),0),
#              (time_traj*6, 1,2.15,0.25, np.deg2rad(90),np.deg2rad(0),0),
#              (time_traj*7, 1,2.15,2.15, np.deg2rad(0),np.deg2rad(0),0),
#              (time_traj*8, 0.5,2.15,2.15, np.deg2rad(0),np.deg2rad(0),0),
#              (time_traj*9, 0,2.15,2.2, np.deg2rad(0),np.deg2rad(0),0),
#              (time_traj*0, -0.1,2.2,2.2, np.deg2rad(0),np.deg2rad(0),0)]

# Solo cargo   t  x y z  - a b g
# time_traj=2.5
# waypoints = [(time_traj*0, 0,0,0.5, 0,0,0),
#              (time_traj*1, 0,0,1, 0,0,0),
#              (10, 1,0,1, 0,np.deg2rad(0),0)]
             #(time_traj*3, 0,1,0.5, 0,np.deg2rad(90),0),
             #(time_traj*4, 0,1.4,1.2, np.deg2rad(0),0,0),
             #(time_traj*5, 0.3,1.5,1.2, 0,np.deg2rad(0),0),
             #(time_traj*6, 0.6,2,1.2, 0,0,np.deg2rad(0)),
             #(time_traj*7, 1,2,1.3, 0,0,0),
             #(time_traj*8, 1,2,1.7, 0,0,0)]

if VehicleID == 1:
    print "First trajectory"
    time_traj=2.5
    stop_time=10
    waypoints = [(5, 0,1,0.5, 0,0,0),
                 (10, 0,0.5,0.5, 0,0,0),
                 (15, 0,0,0.5, 0,0,0),
                 (20, 0,0,0.25, 0,0,0),
                 (30, 0,0,0.14, 0,0,0),
                 (40, 0,0,0.14, 0,0,0),
                 (50, 0,0,0.14, 0,0,0),
                 (55, 0,0,0.5, 0,0,0),
                 (60, 0,0.5,0.5, 0,0,0),
                 (65, 0,1,0.5, 0,0,0)]

if VehicleID == 2:
    print "Second trajectory"
    time_traj=2.5
    stop_time=5
    waypoints = [(5, 0,-1,1, 0,0,0),
                 (10, 0,-0.5,1, 0,0,0),
                 (15, 0,0,1, 0,0,0),
                 (20, 0,0,0.5, 0,0,0),
                 (30, 0,0,0.34, 0,0,0),
                 (43, 0,0,0.34, 0,0,0),
                 (45, 0,0,1, 0,0,0),
                 (55, 0,0,1, 0,0,0),
                 (60, 0,0,0.5, 0,0,0),
                 (70, 0,0,0.25, 0,0,0),
                 (75, 0,0,0.14, 0,0,0),
                 (80, 0,0,0.14, 0,0,0),
                 (85, 0,0,0.14, 0,0,0)]




traj = gen_from_traj(waypoints)

# Noise Params
PosD = 0.01
VelD = 0.01
OmeD = np.deg2rad(0.1)
RotD = np.deg2rad(0.1)

# Constants
Ftop = 10000
Mtop = 10000
mass=6.047
Ai = np.loadtxt("Ai_m5.txt")
A = np.loadtxt("A_m5.txt")

# Ref
v,targetObj= vrep.simxGetObjectHandle(clientID,("Ref%d"%VehicleID),vrep.simx_opmode_oneshot_wait)
d = 0.4
refvel = np.array([0,0.03,0])

# Variables VehicleID
#refvel = np.zeros(3)
OmegaDes = np.zeros(3)
prevRot = np.eye(3)

# Gains for position
KXp=20
KXd=22

# Gains for attitude
KRp=0.1
KRd=0.1510

# Logs - Attitude
pltRError1 = [0]
pltRError2 = [0]
pltRError3= [0]

pltOmError1 = [0]
pltOmError2 = [0]
pltOmError3 = [0]

pltM1 = [0]
pltM2 = [0]
pltM3 = [0]

# Logs - Trajectory
pltPos1 = [0]
pltPos2 = [0]
pltPos3 = [0]

pltRPos1 = [0]
pltRPos2 = [0]
pltRPos3 = [0]

# Logs - Position
pltFx = [0]
pltFy = [0]
pltFz = [0]

pltEPos1 = [0]
pltEPos2 = [0]
pltEPos3 = [0]

pltEVel1 = [0]
pltEVel2 = [0]
pltEVel3 = [0]

pltRotx1 = [0]
pltRotx2 = [0]
pltRotx3 = [0]
pltRotz1 = [0]
pltRotz2 = [0]
pltRotz3 = [0]

pltPhi = [0]
pltSNR = [0]
prevRot = np.eye(3)
prevRefpos= np.zeros(2)

pltInput1 = [0]
pltInput2 = [0]
pltInput3 = [0]
pltInput4 = [0]
pltInput5 = [0]
pltInput6 = [0]

simTime = [0]
# Others
i=0 
j=0

# Read data before loop:
n, n, n, n, n, n,start_time,dt= readData(clientID,vrep.simx_opmode_buffer,VehicleID)
prevST= 0
step = 0.01

behindTime = 0
behindTimeRef = 0
ref1 = [0,0,0.14]
ref2 = [0,0,0.34]

removed = 0

getRef(clientID,1,vrep.simx_opmode_streaming)
# -------- MAIN LOOP --------
while vrep.simxGetConnectionId(clientID)!=-1:
    vrep.simxSynchronousTrigger(clientID);

    hexapos, hexavel, Rmat, notUsed, refpos, Rdes,simtime,dt = readData(clientID,vrep.simx_opmode_buffer,VehicleID)
    # Fetch Data
    t = simtime-start_time
    i = int(round(t/step))

    #if 0<=i<=len(traj)-1:
    #    refpos,refvel,refori,OmegaDes = traj[i]
    #else:
    #    refpos,refvel,refori,OmegaDes = traj[-1]
    #    OmegaDes = np.zeros(3)
    #    refvel = np.zeros(3)

    #reforiE = t3d.mat2euler(refori, axes='sxyz')
    #vrep.simxSetObjectPosition(clientID,targetObj,-1,refpos,vrep.simx_opmode_oneshot_wait)
    #vrep.simxSetObjectOrientation(clientID,targetObj,-1,reforiE,vrep.simx_opmode_oneshot_wait)
   
    # Angular Velocity
    omegaSkew = logr(np.dot(prevRot.T, Rmat))/(simtime-prevST)
    hexaomega = np.array([omegaSkew[2][1], omegaSkew[0][2], omegaSkew[1][0] ]) 

    #if refpos[2] > 0.45:
    #    hexapos, hexavel, hexaomega, Rmat = addNoise(PosD,VelD,OmeD,RotD,hexapos,hexavel,hexaomega,Rmat)

    if(removed == 0):
        # Position controller
        getRef(clientID,1,vrep.simx_opmode_streaming)

        normU = np.subtract(hexapos,refpos)-np.dot(np.cross( np.subtract(hexapos,refpos), refvel/norm(refvel) ) ,refvel/norm(refvel))
        normU = normU/norm(normU)

        if(t>7):
            d=d-t*0.0004
            if(d < 0.1):
                d=0.1

        Nrefpos=refpos+d*normU
        NposError = np.subtract(Nrefpos,hexapos)
        posError = np.subtract(refpos,hexapos)
        velError = np.subtract(refvel,hexavel)


        a = normU.T 
        c = np.cross( refvel/norm(refvel) , normU.T )
        b = refvel/norm(refvel) 

        Rdes = np.array([ refvel/norm(refvel) , np.cross(  normU.T , refvel/norm(refvel) ), normU.T]).T

        # Attitude Controller
        skew = np.dot(Rdes.T,Rmat)-np.dot(Rmat.T,Rdes)
        rotError = (1/(2*np.sqrt(1.001+np.trace(np.dot(Rdes.T,Rmat)))))*np.array([skew[2][1],skew[0][2],skew[1][0]])
        omeError = hexaomega.T - reduce(np.dot, (Rmat.T, Rdes, OmegaDes))


        # Controller
        F = np.dot(Rmat.T,KXp*NposError+KXd*velError)
        M = -KRp*rotError-KRd*omeError

        print "SimTime: ", t, "\nPE: ", norm(posError) ,"\nRE:",norm(rotError), "\nR:",removed, "\n---------------"   

        #F = np.zeros(3)
        #M = np.zeros(3)

        T = np.concatenate((F,M),0)
        u = np.dot(Ai,T)

        if(norm(posError)<0.00005):
            getRef(clientID,0,vrep.simx_opmode_buffer)
            vrep.simxRemoveObject(clientID,targetObj,vrep.simx_opmode_oneshot)
            removed = 1

    elif(removed==1):

        getRef(clientID,0,vrep.simx_opmode_streaming)
        refpos=prevRefpos
        #simxSetFloatingParameter(number clientID,number paramIdentifier,number paramValue,number operationMode)

        # Position controller
        posError = np.subtract(refpos,hexapos)
        velError = np.subtract(refvel,hexavel)

        # Attitude Controller
        skew = np.dot(Rdes.T,Rmat)-np.dot(Rmat.T,Rdes)
        rotError = (1/(2*np.sqrt(1.001+np.trace(np.dot(Rdes.T,Rmat)))))*np.array([skew[2][1],skew[0][2],skew[1][0]])
        omeError = hexaomega.T - reduce(np.dot, (Rmat.T, Rdes, OmegaDes))

        # Controller
        F = np.dot(Rmat.T,KXp*posError+KXd*velError)
        M = -KRp*rotError-KRd*omeError

        print "SimTime: ", t, "\nPE: ",refpos,"\nRE:",norm(rotError), "\nR:",removed, "\n---------------"   

        #F = np.zeros(3)
        #M = np.zeros(3)

        T = np.concatenate((F,M),0)
        u = np.dot(Ai,T)


    
    status = sendCommand(clientID,u.tolist(),vrep.simx_opmode_oneshot,VehicleID)

    prevRot = Rmat
    prevST = simtime
    if(removed==0):
        prevRefpos = refpos
    #dataLogger(rotError,omeError,M,hexapos,refpos,F,posError,velError,Rmat,simtime)
    pltInput1.append(u.tolist()[0])
    pltInput2.append(u.tolist()[1])
    pltInput3.append(u.tolist()[2])
    pltInput4.append(u.tolist()[3])
    pltInput5.append(u.tolist()[4])
    pltInput6.append(u.tolist()[5])

    simTime.append(simtime)
    pltRError1.append(rotError[0])
    pltRError2.append(rotError[1])
    pltRError3.append(rotError[2])

    pltOmError1.append(omeError[0])
    pltOmError2.append(omeError[1])
    pltOmError3.append(omeError[2])

    pltM1.append(M[0])
    pltM2.append(M[1])
    pltM3.append(M[2])

    pltPos1.append(hexapos[0])
    pltPos2.append(hexapos[1])
    pltPos3.append(hexapos[2])

    pltRPos1.append(refpos[0])
    pltRPos2.append(refpos[1])
    pltRPos3.append(refpos[2])

    pltFx.append(F[0])
    pltFy.append(F[1])
    pltFz.append(F[2])

    pltEPos1.append(posError[0])
    pltEPos2.append(posError[1])
    pltEPos3.append(posError[2])

    pltEVel1.append(velError[0])
    pltEVel2.append(velError[1])
    pltEVel3.append(velError[2])

    pltRotx1.append(Rmat[0][0])
    pltRotx2.append(Rmat[1][0])
    pltRotx3.append(Rmat[2][0])
    pltRotz1.append(Rmat[0][1])
    pltRotz2.append(Rmat[1][1])
    pltRotz3.append(Rmat[2][1])

    


saveMat(VehicleID,simTime,pltM1,pltM2,pltM3,pltFx,pltFy,pltFz,pltRError1,pltRError2,pltRError3,pltOmError1,pltOmError2,pltOmError3,
    pltEPos1,pltEPos2,pltEPos3,pltEVel1,pltEVel2,pltEVel3,pltRotx1,pltRotx2,pltRotx3,pltRotz1,pltRotz2,pltRotz3,
    pltPos1,pltPos2,pltPos3,pltInput1,pltInput2,pltInput3,pltInput4,pltInput5,pltInput6)


vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)
