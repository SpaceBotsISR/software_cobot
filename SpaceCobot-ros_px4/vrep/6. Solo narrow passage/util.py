# -*- coding: utf-8 -*-
"""
Created on Mon Jan 25 15:57:02 2016

@author: Pedro Roque
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from numpy.random import normal
import scipy.io as sio
from numpy.random import randn
import scipy.linalg as la


def plotAttErrors(simTime,pltRError1,pltRError2,pltRError3, pltOmError1, pltOmError2, pltOmError3,pltM1,pltM2,pltM3):
    FIGSIZE = (5,5)
    labels = ['Attitude Error','Angular Velocity Error','Torque',None]
    style = ['-','--','-.','-']
    color = ['#000000', '#000000', '#FF0000','#939696']

    y1 = [pltRError1[2:] , pltOmError1[2:] , pltM1[2:], np.zeros(len(simTime) - 2) ]
    y2 = [pltRError2[2:] , pltOmError2[2:] , pltM2[2:], np.zeros(len(simTime) - 2) ]
    y3 = [pltRError3[2:] , pltOmError3s[2:] , pltM3[2:], np.zeros(len(simTime) - 2) ]    
    
    plt.figure(figsize=FIGSIZE)
    for y_arr, label, style,color in zip(y1,labels,style,color):        
       plt.plot(simTime[2:], y_arr, style, label=label, color=color)
    plt.legend()
    plt.ylabel("Error: -[rad]  --[rad/s]")
    plt.xlabel("Time [s]")
    plt.title("Error Over Time - X Axis")
    plt.show()
    
    plt.figure(figsize=FIGSIZE)
    for y_arr, label, style,color in zip(y2,labels,style,color):        
       plt.plot(simTime[2:], y_arr, style, label=label, color=color)
    plt.legend()    
    plt.ylabel("Error: -[rad]  --[rad/s]")
    plt.xlabel("Time [s]")
    plt.title("Error Over Time - Y Axis")
    plt.show()
    
    plt.figure(figsize=FIGSIZE)
    for y_arr, label, style,color in zip(y3,labels,style,color):        
       plt.plot(simTime[2:], y_arr, style, label=label, color=color)
    plt.legend()    
    plt.ylabel("Error: -[rad]  --[rad/s]")
    plt.xlabel("Time [s]")
    plt.title("Error Over Time - Z Axis")
    plt.show()
    

    #plt.axes([simTime[2:] simTime[-1] 0 ])
    #plt.xlim((0,7))
    #plt.ylim((-0.7,0.2))
    #plt.savefig('rz-error.png', bbox_inches='tight')
    #plt.show()

def plotPosErrors(simTime,pltEPos1,pltEPos2,pltEPos3, pltEVel1, pltEVel2, pltEVel3,pltFx,pltFy,pltFz):
    FIGSIZE = (5,5)
    labels = ['Position Error','Velocity Error',None]
    style = ['-','--','-']
    color = ['#000000', '#000000', '#939696']

    y1 = [pltEPos1[2:] , pltEVel1[2:] , np.zeros(len(simTime) - 2) ]
    y2 = [pltEPos2[2:] , pltEVel2[2:] , np.zeros(len(simTime) - 2) ]
    y3 = [pltEPos3[2:] , pltEVel3[2:] , np.zeros(len(simTime) - 2) ]    
    
    plt.figure(figsize=FIGSIZE)
    for y_arr, label, style,color in zip(y1,labels,style,color):        
       plt.plot(simTime[2:], y_arr, style, label=label, color=color)
    plt.legend()    
    plt.ylabel("Error: -[m]  --[m/s]")
    plt.xlabel("Time [s]")
    plt.title("Error Over Time - X Axis")
    plt.xlim((0,3))
    plt.ylim((-2.2,1.5))
    plt.savefig('px-error.png', bbox_inches='tight')
    plt.show()

    # plt.figure(figsize=FIGSIZE)
    # for y_arr, label, style,color in zip(y2,labels,style,color):        
    #     plt.plot(simTime[2:], y_arr, style, label=label, color=color)
    # plt.legend()    
    # plt.ylabel("Error: -[m]  --[m/s]")
    # plt.xlabel("Time [s]")
    # plt.title("Error Over Time - Y Axis")
    # plt.xlim((0,3))
    # plt.ylim((-2.2,1.5))
    # plt.savefig('py-error.png', bbox_inches='tight')
    # plt.show()
 
    # plt.figure(figsize=FIGSIZE)
    # for y_arr, label, style,color in zip(y3,labels,style,color):        
    #     plt.plot(simTime[2:], y_arr, style, label=label, color=color)
    # plt.legend()
    # plt.ylabel("Error: -[m]  --[m/s]")
    # plt.xlabel("Time [s]")
    # plt.title("Error Over Time - Z Axis")
    # plt.xlim((0,3))
    # plt.ylim((-2.2,1.5))
    # plt.savefig('pz-error.png', bbox_inches='tight')
    # plt.show()

    # #plt.axes([simTime[2:] simTime[-1] 0 ])
    # plt.show()
    return 0




def plot3D(simTime,pltPos1,pltPos2,pltPos3,pltRPos1,pltRPos2,pltRPos3,pltRotx1,pltRotx2,pltRotx3,pltRotz1,pltRotz2,pltRotz3):
    FIGSIZE = (14,7)
    labels = ['X Position [m]','Y Position [m]','Z Position [m]']

    sampling = 10
    maskrot = np.zeros(len(pltRotx1))
    
    for i in range(len(maskrot)):
        if i%sampling == 0:
            maskrot[i] = 1

    maskrot[0] = 1

    pltRotx1 = pltRotx1*maskrot
    pltRotx2 = pltRotx2*maskrot
    pltRotx3 = pltRotx3*maskrot
    pltRotz1 = pltRotz1*maskrot
    pltRotz2 = pltRotz2*maskrot
    pltRotz3 = pltRotz3*maskrot

    scale = 0.1

    fig = plt.figure(figsize=FIGSIZE)
    ax = fig.add_subplot(111, projection='3d')
    #ax = Axes3D(plt.gcf())
    #ax.plot(pltPos1,pltPos2,zs=pltPos3,color='#757575')
    #ax.plot(pltRPos1,pltRPos2,zs=pltRPos3,color='#FF0000')
    for i in range(len(pltRotx1)):
        ax.plot([pltPos1[i] ,pltPos1[i]+pltRotx1[i]*scale],[pltPos2[i],pltPos2[i]+pltRotx2[i]*scale],zs=[pltPos3[i],pltPos3[i]+pltRotx3[i]*scale], color='0.3')
        ax.plot([pltPos1[i] ,pltPos1[i]+pltRotz1[i]*scale],[pltPos2[i],pltPos2[i]+pltRotz2[i]*scale],zs=[pltPos3[i],pltPos3[i]+pltRotz3[i]*scale], color='0.3')
        #ax.plot(pltRPos1[i],pltRPos2[i],zs=pltRPos3[i],color='#ee0000',)
    #ax.legend()
    #plt.axis('equal')
    ax.set_xlabel(labels[0])
    ax.set_ylabel(labels[1])
    ax.set_zlabel(labels[2])
    plt.show()
    #Axes3D.plot()

    return 0

def plotTrajErrorsAtt(simTime,pltRError1,pltRError2,pltRError3, pltOmError1, pltOmError2, pltOmError3,pltM1,pltM2,pltM3):
    FIGSIZE = (13,10)
    labels = ['Attitude Error on X','Attitude Error on Y','Attitude Error on Z',None]
    labels2 = ['Angular Velocity Error on X','Angular Velocity Error on Y','Angular Velocity Error on Z',None]
    style = ['-','--',':','-']
    color = ['#000000', '#000000', '#000000', '#939696']

    y1 = [pltRError1[2:] , pltRError2[2:] , pltRError3[2:], np.zeros(len(simTime) - 2) ]
    y2 = [pltOmError1[2:] , pltOmError2[2:] , pltOmError3[2:] ,np.zeros(len(simTime) - 2) ]
   
    # plt.figure(figsize=FIGSIZE)
    # for y_arr, label, style,color in zip(y1,labels,style,color):        
    #    plt.plot(simTime[2:], y_arr, style, label=label, color=color)
    # plt.legend(loc='upper center',ncol=3, bbox_to_anchor=(0.5,-0.06))    
    # plt.ylabel("Error [rad]",fontsize=22)
    # plt.xlabel("Time [s]",fontsize=22)
    # #plt.title("Error Over Time - Attide Error on Trajectory")
    # plt.xlim((0,simTime[-1]))
    # #plt.ylim((-0.7,0.2))
    # plt.savefig('traj-r-error.png', bbox_inches='tight')
    # plt.show()

    # plt.figure(figsize=FIGSIZE)
    # for y_arr, label, style,color in zip(y2,labels2,style,color):        
    #    plt.plot(simTime[2:], y_arr, style, label=label, color=color)
    # plt.legend(loc='upper center',ncol=3, bbox_to_anchor=(0.5,-0.06))    
    # plt.ylabel("Error [rad/s]",fontsize=18)
    # plt.xlabel("Time [s]",fontsize=18)
    # #plt.title("Error Over Time - Attide Error on Trajectory")
    # plt.xlim((0,simTime[-1]))
    # #plt.ylim((-0.7,0.2))
    # plt.savefig('traj-om-error.png', bbox_inches='tight')
    # plt.show()

def plotTrajErrorsPos(simTime,pltEPos1,pltEPos2,pltEPos3, pltEVel1, pltEVel2, pltEVel3,pltFx,pltFy,pltFz):
    FIGSIZE = (13,10)
    labels = ['Position Error on X','Position Error on Y','Position Error on Z',None]
    labels2 = ['Linear Velocity Error on X','Linear Velocity Error on Y','Linear Velocity Error on Z',None]
    style = ['-','--',':','-']
    color = ['#000000', '#000000', '#000000', '#939696']

    y1 = [pltEPos1[2:] , pltEPos2[2:], pltEPos3[2:] , np.zeros(len(simTime) - 2) ]
    y2 = [pltEVel1[2:] , pltEVel2[2:] , pltEVel3[2:], np.zeros(len(simTime) - 2) ]

    # plt.figure(figsize=FIGSIZE)
    # for y_arr, label, style,color in zip(y1,labels,style,color):        
    #    plt.plot(simTime[2:], y_arr, style, label=label, color=color)
    # plt.legend(loc='upper center',ncol=3, bbox_to_anchor=(0.5,-0.06))    
    # plt.ylabel("Error [m]",fontsize=18)
    # plt.xlabel("Time [s]",fontsize=18)
    # #plt.title("Error Over Time - Attide Error on Trajectory")
    # plt.xlim((0,simTime[-1]))
    # #plt.ylim((-0.7,0.2))
    # plt.savefig('traj-p-error.png', bbox_inches='tight')
    # plt.show()

    plt.figure(figsize=FIGSIZE)
    for y_arr, label, style,color in zip(y2,labels2,style,color):        
       plt.plot(simTime[2:], y_arr, style, label=label, color=color)
    plt.legend(loc='upper center',ncol=3, bbox_to_anchor=(0.5,-0.06))    
    plt.ylabel("Error [m/s]",fontsize=18)
    plt.xlabel("Time [s]",fontsize=18)
    #plt.title("Error Over Time - Attide Error on Trajectory")
    plt.xlim((0,simTime[-1]))
    #plt.ylim((-0.7,0.2))
    plt.savefig('traj-v-error.png', bbox_inches='tight')
    plt.show()


def logr(R):
    phi = np.arccos(np.clip((R.trace() - 1)/2, -1 , 1))
    return (R-R.T)/(2*np.sinc(phi/np.pi))
    
#def PrintData(A):
#    for x in A:
#        print (x)
#        for y in A[x]:
#            print (y,':',A[x][y])
#        
#    print "\n ------ "

def noise(v):
    a = np.array([[0,-v[3],v[2]],[v[3],0,[-v[1]]],[-v[2],v[1],0]])
    return la.expm(a)

def dataLogger(rotError,omeError,M,hexapos,refpos,F,posError,velError,Rmat,simTime):
    if simtime != 0 and simtime != simTime[-1]:
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
        return 0
    else:
        return -1

def saveMat(VehicleID,simTime,pltM1,pltM2,pltM3,pltFx,pltFy,pltFz,pltRError1,pltRError2,pltRError3,pltOmError1,pltOmError2,pltOmError3,
    pltEPos1,pltEPos2,pltEPos3,pltEVel1,pltEVel2,pltEVel3,pltRotx1,pltRotx2,pltRotx3,pltRotz1,pltRotz2,pltRotz3,
    pltPos1,pltPos2,pltPos3,pltInput1,pltInput2,pltInput3,pltInput4,pltInput5,pltInput6):
    try:
        sio.savemat(('python_data_%d.mat'%VehicleID), {'PySimTime':simTime,'M1':pltM1,'M2':pltM2,'M3':pltM3,
             'PyRError1':pltRError1,'PyRError2':pltRError2,'PyRError3':pltRError3,
             'PyOmeError1':pltOmError1,'PyOmeError2':pltOmError2,'PyOmeError3':pltOmError3, 
             'PyPosError1':pltEPos1,'PyPosError2':pltEPos2,'PyPosError3':pltEPos3,
             'PyVelError1':pltEVel1, 'PyVelError2':pltEVel2, 'PyVelError3':pltEVel3, 'pltRotx1':pltRotx1,
             'pltRotx2':pltRotx2,'pltRotx3':pltRotx3,'pltRotz1':pltRotz1,'pltRotz2':pltRotz2,'pltRotz3':pltRotz3,
             'pltPos1':pltPos1,'pltPos2':pltPos2,'pltPos3':pltPos3,'pltFx':pltFx,'pltFy':pltFy,'pltFz':pltFz,
             'pltInput1':pltInput1,'pltInput2':pltInput2,'pltInput3':pltInput3,'pltInput4':pltInput4,'pltInput5':pltInput5,
             'pltInput16':pltInput6})
        return 0
    except:
        return -1

def addNoise(PosD,VelD,OmeD,RotD,hexapos,hexavel,hexaomega,Rmat):
    hexapos = hexapos+PosD*randn(3)
    hexavel = hexavel+VelD*randn(3)
    hexaomega = hexaomega+OmeD*randn(3)
    v = RotD*randn(3)
    a = np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
    Rmat=np.dot(Rmat,la.expm(a))
    return hexapos,hexavel,hexaomega,Rmat