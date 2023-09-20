
-- DO NOT WRITE CODE OUTSIDE OF THE if-then-end SECTIONS BELOW!! (unless the code is a function definition)

if (sim_call_type==sim_childscriptcall_initialization) then
    VehicleID=1
    -- Put some initialization code here
    v=simGetInt32Parameter(sim_intparam_program_version)
    if (v<20413) then
        simDisplayDialog('Warning','The propeller model is only fully supported from V-REP version 2.4.13 and above.&&nThis simulation will not run as expected!',sim_dlgstyle_ok,false,'',nil,{0.8,0,0,0,0,0})
    end
    hexacopter = simGetObjectHandle('Cubo'..VehicleID)
    result=simExtRemoteApiStart(19999+VehicleID)
    print("API Start: ", result)

    -- Detatch the manipulation sphere:
    targetObj=simGetObjectHandle('Ref'..VehicleID)
    Force={0,0,0}
    Torque={0,0,0}
    --simSetShapeMassAndInertia(hexacopter, 6.047, {0.04533008, 0.000710774, -0.001934362, 0.000710774, 0.04166828, -0.003146458, -0.001934362, -0.003146458, 0.05198795},simGetObjectPosition(hexacopter,-1))
    simSetShapeMassAndInertia(hexacopter, 6.047, {0.04533008, 0.0, 0.0, 0.0, 0.04166828, 0.0, 0.0, 0.0, 0.05198795},simGetObjectPosition(hexacopter,-1))
    
    -- Main initialization
    propellerRespondable = {-1,-1,-1,-1,-1,-1}
    propellerMatrix={-1,-1,-1,-1,-1,-1}
    force = {-1,-1,-1,-1,-1,-1}
    torque = {-1,-1,-1,-1,-1,-1}
    u = {{0},{0},{0},{0},{0},{0}}
 

    for i=1,6,1 do
        propellerRespondable[i]=simGetObjectHandle('Propeller'..i..VehicleID)
    end
    for i=1,6,1 do
        propellerMatrix[i] = simGetObjectMatrix(propellerRespondable[i],-1)
        propellerMatrix[i][4] = 0
        propellerMatrix[i][8] = 0
        propellerMatrix[i][12] = 0
    end

    for i=1,6,1 do
        force[i] = {0,0,0}
    end

    for i=1,6,1 do
        torque[i] = {0,0,0}
    end
end


if (sim_call_type==sim_childscriptcall_actuation) then
    --Force={0,0,0}
    --Torque={0,0,0}    
    K1 = 1
    K2 = 0.01
    
    -- SIMÉTRICO AO ARTIGO!
    w = {1,-1,1,-1,1,-1}
    --w = {-1,1,-1,1,-1,1}

    refData = simGetObjectMatrix(targetObj,-1)
    simSetFloatSignal("refData1"..VehicleID, refData[1])
    simSetFloatSignal("refData2"..VehicleID, refData[2])
    simSetFloatSignal("refData3"..VehicleID, refData[3])
    simSetFloatSignal("refPos1"..VehicleID, refData[4])
    simSetFloatSignal("refData5"..VehicleID, refData[5])
    simSetFloatSignal("refData6"..VehicleID, refData[6])
    simSetFloatSignal("refData7"..VehicleID, refData[7])
    simSetFloatSignal("refPos2"..VehicleID, refData[8])
    simSetFloatSignal("refData9"..VehicleID, refData[9])
    simSetFloatSignal("refData10"..VehicleID, refData[10])
    simSetFloatSignal("refData11"..VehicleID, refData[11])
    simSetFloatSignal("refPos3"..VehicleID, refData[12])    

    hexaData = simGetObjectMatrix(hexacopter,-1)
    simSetFloatSignal("hexaData1"..VehicleID, hexaData[1])
    simSetFloatSignal("hexaData2"..VehicleID, hexaData[2])
    simSetFloatSignal("hexaData3"..VehicleID, hexaData[3])
    simSetFloatSignal("hexaPos1"..VehicleID, hexaData[4])
    simSetFloatSignal("hexaData5"..VehicleID, hexaData[5])
    simSetFloatSignal("hexaData6"..VehicleID, hexaData[6])
    simSetFloatSignal("hexaData7"..VehicleID, hexaData[7])
    simSetFloatSignal("hexaPos2"..VehicleID, hexaData[8])
    simSetFloatSignal("hexaData9"..VehicleID, hexaData[9])
    simSetFloatSignal("hexaData10"..VehicleID, hexaData[10])
    simSetFloatSignal("hexaData11"..VehicleID, hexaData[11])
    simSetFloatSignal("hexaPos3"..VehicleID, hexaData[12])

    hexavel, hexaomega = simGetObjectVelocity(hexacopter,-1)
    simSetFloatSignal("hexaVel1"..VehicleID,hexavel[1])
    simSetFloatSignal("hexaVel2"..VehicleID,hexavel[2])
    simSetFloatSignal("hexaVel3"..VehicleID,hexavel[3])
    simSetFloatSignal("hexaOmega1"..VehicleID,hexaomega[1])
    simSetFloatSignal("hexaOmega2"..VehicleID,hexaomega[2])
    simSetFloatSignal("hexaOmega3"..VehicleID,hexaomega[3])
    

    simtime = simGetSimulationTime()
    simSetFloatSignal("simtime",simtime)
    print("Simtime:",simtime)

    dt = simGetSimulationTimeStep()
    simSetFloatSignal("dt",dt)

    data1 = simGetFloatSignal("control1"..VehicleID)
    if data1 then
        u[1][1] = data1
    end
    data2 = simGetFloatSignal("control2"..VehicleID)
    if data2 then
        u[2][1] = data2
    end
    data3 = simGetFloatSignal("control3"..VehicleID)
    if data3 then
        u[3][1] = data3
    end
    data4 = simGetFloatSignal("control4"..VehicleID)
    if data4 then
        u[4][1] = data4
    end
    data5 = simGetFloatSignal("control5"..VehicleID)
    if data5 then
        u[5][1] = data5
    end
    data6 = simGetFloatSignal("control6"..VehicleID)
    if data6 then
        u[6][1] = data6
    end

    --simAddForceAndTorque(hexacopter,Force,Torque)
    for i=1,6,1 do
        print("U"..i..VehicleID..": ",u[i][1])
        force[i] = {0,0,K1*u[i][1]}
        --force[i] = simMultiplyVector(simGetInvertedMatrix(propellerMatrix[i]),force[i])
        simAddForce(propellerRespondable[i],{0,0,0},force[i])

        torque[i] = {0,0,w[i]*K2*u[i][1]}
        torque[i] = simMultiplyVector(propellerMatrix[i],torque[i])
    end

    -- Send the desired motor velocities to the 6 rotors:
    for i=1,6,1 do
        --print("Propeller:"..i, force[i][1],force[i][2],force[i][3], torque[i][1],torque[i][2],torque[i][3])
       simAddForceAndTorque(propellerRespondable[i],{0,0,0},torque[i])
    end


end


if (sim_call_type==sim_childscriptcall_sensing) then

    -- Put your main SENSING code here

end


if (sim_call_type==sim_childscriptcall_cleanup) then

    result=simExtRemoteApiStop(19999+VehicleID)
    print("API Stop: ", result)

end