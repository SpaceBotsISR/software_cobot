# -*- coding: utf-8 -*-
"""
Created on Wed Jan 20 20:43:32 2016

@author: Pedro Roque
"""
import vrep
import numpy as np


# Globals
threshold = 0.00001

def readData(clientID,opmode,VehicleID=''):
    # Opmodes:
    # simx_opmode_buffer
    # simx_opmode_streaming

    simtime = 0
    dt = 0

    hexavel = [0,0,0]
    hexaomega = [0,0,0]

    hexaPos = [0,0,0]
    hexaQ = [0,0,0,0]
    
    refPos = [0,0,0]
    refQ = [0,0,0,0]
    
    hexarot = np.zeros((3,3))
    refrot = np.zeros((3,3))
    
    v, hexapos1 = vrep.simxGetFloatSignal(clientID,("hexaPos1%s"%VehicleID),opmode)
    if v==0:
        hexaPos[0] = hexapos1

    v, hexapos2 = vrep.simxGetFloatSignal(clientID,("hexaPos2%s"%VehicleID),opmode)
    if v==0:
        hexaPos[1] = hexapos2   
    
    v, hexapos3 = vrep.simxGetFloatSignal(clientID,("hexaPos3%s"%VehicleID),opmode)
    if v==0:
        hexaPos[2] = hexapos3
        
    v, hexavel1=vrep.simxGetFloatSignal(clientID,("hexaVel1%s"%VehicleID),opmode)
    if v == 0 and abs(hexavel1) > threshold:
        hexavel[0] = hexavel1
        
    v, hexavel2=vrep.simxGetFloatSignal(clientID,("hexaVel2%s"%VehicleID),opmode)
    if v == 0 and abs(hexavel2) > threshold:
        hexavel[1] = hexavel2
        
    v, hexavel3=vrep.simxGetFloatSignal(clientID,("hexaVel3%s"%VehicleID),opmode)
    if v == 0 and abs(hexavel3) > threshold:
        hexavel[2] = hexavel3
        
    v, hexaomega1=vrep.simxGetFloatSignal(clientID,("hexaOmega1%s"%VehicleID),opmode)
    if v == 0 and abs(hexaomega1) > threshold:
        hexaomega[0] = hexaomega1
        
    v, hexaomega2=vrep.simxGetFloatSignal(clientID,("hexaOmega2%s"%VehicleID),opmode)
    if v == 0 and abs(hexaomega2) > threshold:
        hexaomega[1] = hexaomega2
        
    v, hexaomega3=vrep.simxGetFloatSignal(clientID,("hexaOmega3%s"%VehicleID),opmode)
    if v == 0 and abs(hexaomega3) > threshold:
        hexaomega[2] = hexaomega3
     
     
    # Reference         
    v, refpos1 = vrep.simxGetFloatSignal(clientID,("refPos1%s"%VehicleID),opmode)
    if v==0:
        refPos[0] = refpos1
    
    v, refpos2 = vrep.simxGetFloatSignal(clientID,("refPos2%s"%VehicleID),opmode)
    if v==0:
        refPos[1] = refpos2   
    
    v, refpos3 = vrep.simxGetFloatSignal(clientID,("refPos3%s"%VehicleID),opmode)
    if v==0:
        refPos[2] = refpos3
    
    v, H1 = vrep.simxGetFloatSignal(clientID,("hexaData1%s"%VehicleID),opmode)
    if v == 0 :
        hexarot[0][0] = H1
    
    v, H2 = vrep.simxGetFloatSignal(clientID,("hexaData2%s"%VehicleID),opmode)
    if v == 0 :
        hexarot[0][1] = H2
    
    v, H3 = vrep.simxGetFloatSignal(clientID,("hexaData3%s"%VehicleID),opmode)
    if v == 0 :
        hexarot[0][2] = H3
    
    v, H4 = vrep.simxGetFloatSignal(clientID,("hexaData5%s"%VehicleID),opmode)
    if v == 0 :
        hexarot[1][0] = H4
    
    v, H5 = vrep.simxGetFloatSignal(clientID,("hexaData6%s"%VehicleID),opmode)
    if v == 0 :
        hexarot[1][1] = H5
    
    v, H6 = vrep.simxGetFloatSignal(clientID,("hexaData7%s"%VehicleID),opmode)
    if v == 0 :
        hexarot[1][2] = H6

    v, H7 = vrep.simxGetFloatSignal(clientID,("hexaData9%s"%VehicleID),opmode)
    if v == 0 :
        hexarot[2][0] = H7
    
    v, H8 = vrep.simxGetFloatSignal(clientID,("hexaData10%s"%VehicleID),opmode)
    if v == 0 :
        hexarot[2][1] = H8
    
    v, H9 = vrep.simxGetFloatSignal(clientID,("hexaData11%s"%VehicleID),opmode)
    if v == 0 :
        hexarot[2][2] = H9
        
        
    v, R1 = vrep.simxGetFloatSignal(clientID,("refData1%s"%VehicleID),opmode)
    if v == 0 :
        refrot[0][0] = R1
    
    v, R2 = vrep.simxGetFloatSignal(clientID,("refData2%s"%VehicleID),opmode)
    if v == 0 :
        refrot[0][1] = R2
    
    v, R3 = vrep.simxGetFloatSignal(clientID,("refData3%s"%VehicleID),opmode)
    if v == 0 :
        refrot[0][2] = R3
    
    v, R4 = vrep.simxGetFloatSignal(clientID,("refData5%s"%VehicleID),opmode)
    if v == 0 :
        refrot[1][0] = R4
    
    v, R5 = vrep.simxGetFloatSignal(clientID,("refData6%s"%VehicleID),opmode)
    if v == 0 :
        refrot[1][1] = R5
    
    v, R6 = vrep.simxGetFloatSignal(clientID,("refData7%s"%VehicleID),opmode)
    if v == 0 :
        refrot[1][2] = R6

    v, R7 = vrep.simxGetFloatSignal(clientID,("refData9%s"%VehicleID),opmode)
    if v == 0 :
        refrot[2][0] = R7
    
    v, R8 = vrep.simxGetFloatSignal(clientID,("refData10%s"%VehicleID),opmode)
    if v == 0 :
        refrot[2][1] = R8
    
    v, R9 = vrep.simxGetFloatSignal(clientID,("refData11%s"%VehicleID),opmode)
    if v == 0 :
        refrot[2][2] = R9

    v, simTime = vrep.simxGetFloatSignal(clientID,"simtime",opmode)
    if v == 0 and simTime > 0:
        simtime = simTime
        
    v, Dt = vrep.simxGetFloatSignal(clientID,"dt",opmode)
    if v== 0 and Dt > 0:
        dt = Dt
        
    return hexaPos, np.asarray(hexavel), hexarot, np.asarray(hexaomega), refPos, refrot, simtime, dt

def sendCommand(clientID,u,opmode,VehicleID=''):
    vrep.simxPauseCommunication(clientID,True)
    returnCode1 = vrep.simxSetFloatSignal(clientID,("control1%s"%VehicleID),u[0],opmode)
    returnCode2 = vrep.simxSetFloatSignal(clientID,("control2%s"%VehicleID),u[1],opmode)
    returnCode3 = vrep.simxSetFloatSignal(clientID,("control3%s"%VehicleID),u[2],opmode)
    returnCode4 = vrep.simxSetFloatSignal(clientID,("control4%s"%VehicleID),u[3],opmode)
    returnCode5 = vrep.simxSetFloatSignal(clientID,("control5%s"%VehicleID),u[4],opmode)
    returnCode6 = vrep.simxSetFloatSignal(clientID,("control6%s"%VehicleID),u[5],opmode)
    vrep.simxPauseCommunication(clientID,False)

    a = [returnCode1,returnCode2,returnCode3,returnCode4,returnCode5,returnCode6]    
    b = [0,0,0,0,0,0]

    if ((a or b)==b):
        return 0
    else:
        return -1

def getRef(clientID,refR,opmode):
    vrep.simxSetFloatSignal(clientID,"refR",refR,opmode)

# def sendForceAndTorque(clientID,Force,Torque,opmode):
#     vrep.simxPauseCommunication(clientID,True)
#     returnCode1 = vrep.simxSetFloatSignal(clientID,"Force1",Force[0],opmode)
#     returnCode2 = vrep.simxSetFloatSignal(clientID,"Force2",Force[1],opmode)
#     returnCode3 = vrep.simxSetFloatSignal(clientID,"Force3",Force[2],opmode)
#     returnCode4 = vrep.simxSetFloatSignal(clientID,"Torque1",Torque[0],opmode)
#     returnCode5 = vrep.simxSetFloatSignal(clientID,"Torque2",Torque[1],opmode)
#     returnCode6 = vrep.simxSetFloatSignal(clientID,"Torque3",Torque[2],opmode)
#     vrep.simxPauseCommunication(clientID,False)
    
#     a = [returnCode1,returnCode2,returnCode3,returnCode4,returnCode5,returnCode6]    
#     b = [0,0,0,0,0,0]
    
#     if ((a or b)==b):
#         return 0
#     else:
#         return -1