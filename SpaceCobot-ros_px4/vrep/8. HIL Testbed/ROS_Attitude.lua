
-- DO NOT WRITE CODE OUTSIDE OF THE if-then-end SECTIONS BELOW!! (unless the code is a function definition)
function setSCobotAttitude(msg)
    att = msg.orientation
    print('q=[w:',att.w,',x:',att.x,',y:',att.y,',z:',att.z,']')
    --print('y:',att.y)
    --print('z:',att.x)
    --print('w:',att.w)

    -- Attitude subscriber callback
    simSetObjectQuaternion(hexacopter,-1,{att.x,att.y,att.z,att.w})

end
function getTransformStamped(objHandle,name,relTo,relToName)
    t=simGetSystemTime()
    p=simGetObjectPosition(objHandle,relTo)
    o=simGetObjectQuaternion(objHandle,relTo)
    return {
        header={
            stamp=t,
            frame_id=relToName
        },
        child_frame_id=name,
        transform={
            translation={x=p[1],y=p[2],z=p[3]},
            rotation={x=o[1],y=o[2],z=o[3],w=o[4]}
        }
    }
end

if (sim_call_type==sim_childscriptcall_initialization) then
    -- ROS Stuff --
    hexacopter = simGetObjectHandle('Cubo1')
    --robotHandle=simGetObjectAssociatedWithScript(sim_handle_self)
    
    -- Check if the required ROS plugin is there:
    moduleName=0
    moduleVersion=0
    index=0
    pluginNotFound=true
    while moduleName do
        moduleName,moduleVersion=simGetModuleName(index)
        if (moduleName=='RosInterface') then
            pluginNotFound=false
        end
        index=index+1
    end

    -- Add a banner:
    if (pluginNotFound) then
        bannerText="I cannot run! (I couldn't find my RosInterface plugin)"
    else
        bannerText="I am controlled via a ROS node!"
    end
    black={0,0,0,0,0,0,0,0,0,0,0,0}
    red={0,0,0,0,0,0,0,0,0,1,0.2,0.2}
    simAddBanner(bannerText,0,sim_banner_bitmapfont+sim_banner_overlay,nil,simGetObjectAssociatedWithScript(sim_handle_self),black,red)

    print('Seting up ROS publishers and subscribers')

    -- Ok now launch the ROS client application:
    if (not pluginNotFound) then
        local sysTime=simGetSystemTimeInMs(-1) 
        local simulationTimeTopicName='simTime'..sysTime -- we add a random component so that we can have several instances of this robot running

        -- Prepare the sensor publisher and the motor speed subscribers:
        --sensorPub=simExtRosInterface_advertise('/'..sensorTopicName,'std_msgs/Bool')
        simTimePub=simExtRosInterface_advertise('/SpaceCoBotSimTime','std_msgs/Float32')
        
        -- Uncomment line below if running default PX4 or APM firmware
        --SpaceCoBotAttitude=simExtRosInterface_subscribe('/mavros/imu/data','sensor_msgs/Imu','setSCobotAttitude')
        
        -- Uncomment line below if subscribit to custom topic
        SpaceCoBotAttitude=simExtRosInterface_subscribe('/scobot/imu','sensor_msgs/Imu','setSCobotAttitude')

        -- Now we start the client application: (USE THIS TO LAUNCH EXECUTABLES ON VREP_ROOT AND PASSA PARAMETERS)
        --result=simLaunchExecutable('rosBubbleRob2',leftMotorTopicName.." "..rightMotorTopicName.." "..sensorTopicName.." "..simulationTimeTopicName,0)
    end
end


if (sim_call_type==sim_childscriptcall_actuation) then
    -- Send an updated sensor and simulation time message, and send the transform of the robot:
    if not pluginNotFound then
        simExtRosInterface_publish(simTimePub,{data=simGetSimulationTime()})
        -- Send the robot's transform:
        --simExtRosInterface_sendTransform(getTransformStamped(hexacopter,'SpaceCoBot',-1,'world'))
        -- To send several transforms at once, use simExtRosInterface_sendTransforms instead
    end
end


if (sim_call_type==sim_childscriptcall_sensing) then

    -- Put your main SENSING code here

end


if (sim_call_type==sim_childscriptcall_cleanup) then

   

end