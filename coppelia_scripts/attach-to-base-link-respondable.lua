--[[
===============================================================================
  CoppeliaSim ROS 2 Control Lua Script
  -------------------------------------------------------------------------------
  Package:    coppelia_ros2_control
  Author:     Julian Duda (@dudajulian)
  Description:
    Lua script for real-time joint control and ROS 2 integration in CoppeliaSim.
    Exposes joint velocities as topics and publishes joint states, robot pose,
    and simulation time for ros3_control compatibility.

  -------------------------------------------------------------------------------
  Dependencies:
    - CoppeliaSim API (sim, simROS2)
    - ROS 2 topics: /topic_based_joint_commands, /topic_based_joint_states,
      /robot_pose, /clock

  -------------------------------------------------------------------------------
  Callbacks:
    - sysCall_init():      Initializes handles, subscribers, and publishers.
    - sysCall_actuation(): Publishes simulation time, robot pose, and joint states.
    - sysCall_cleanup():   Shuts down ROS 2 publishers/subscriptions.
    - setJointCmd_cb():    Processes incoming joint velocity commands.

  -------------------------------------------------------------------------------
  Usage:
    Attach this script to the `base_link_respondable` object in CoppeliaSim.
    Ensure ROS 2 topics and joint names match the configuration in your URDF.
===============================================================================
--]]

sim = require('sim')
simROS2 = require('simROS2')


function sysCall_init()
    -- Get handle and alias of the object the script is attached to (should be base_link_respondable) 
    objectHandle = sim.getObject('..')
    objectAlias = sim.getObjectAlias(objectHandle, 3)

    -- Initialize table of the controlled joint handles and aliases
    jointHandles = {} 
    jointAliases = {}
    -- automatically populated by setJointCmd_cb
    -- you can also pre-populate here if joint names are known at startup
    
    --Subscribers
    subJointCmd=simROS2.createSubscription('/topic_based_joint_commands','sensor_msgs/msg/JointState','setJointCmd_cb')
    
    -- Publishers
    pubSimTime = simROS2.createPublisher('/clock', 'rosgraph_msgs/msg/Clock')
    pubRobotPose = simROS2.createPublisher('/robot_pose', 'geometry_msgs/msg/TransformStamped')
    pubJointState = simROS2.createPublisher('/topic_based_joint_states', 'sensor_msgs/msg/JointState')
end

function sysCall_actuation()
    -- Send an updated simulation time message, and send the transform of the object this script is attached to:
    simROS2.publish(pubSimTime, {clock = simROS2.getSimulationTime()})
    simROS2.publish(pubRobotPose, getTransformStamped(objectHandle, objectAlias, sim.handle_world, 'map'))
    simROS2.publish(pubJointState, getWheelJointState("base_link"))
    -- To send several transforms at once, use simROS2.sendTransforms instead
end

function sysCall_cleanup()
    -- Following not really needed in a simulation script (i.e. automatically shut down at simulation end):
    simROS2.shutdownPublisher(pubJointState)
    simROS2.shutdownPublisher(pubRobotPose)
    simROS2.shutdownPublisher(pubSimTime)
    -- simROS2.shutdownSubscription(subLeftMotor)
    -- simROS2.shutdownSubscription(subRightMotor)
    simROS2.shutdownSubscription(subJointCmd)
end

function setJointCmd_cb(msg)
    -- populate joinHandles if empty
    if #jointHandles == 0 then
        for i, jointName in ipairs(msg.name) do
            jointAliases[i] = jointName
            jointHandles[i] = sim.getObject("../" .. jointName)
        end
    end
    -- set joint target velocities
    for i, jh in ipairs(jointHandles) do
        sim.setJointTargetVelocity(jh, msg.velocity[i])
    end
end


function getTransformStamped(objHandle, objName, refHandle, refName)
    -- This function retrieves the stamped transform for a specific object
    local t = simROS2.getSimulationTime()
    local p = sim.getObjectPosition(objHandle,refHandle)
    local o = sim.getObjectQuaternion(objHandle,refHandle)
    return {
        header = {
            stamp = t,
            frame_id = refName
        },
        child_frame_id = objName,
        transform = {
            translation = {x = p[1], y = p[2], z = p[3]},
            rotation = {x = o[1], y = o[2], z = o[3], w = o[4]}
        }
    }
end

function getWheelJointState(refFrameId)
    local names = {}
    local positions = {}
    local velocities = {}
    local efforts = {}
    for i, jh in ipairs(jointHandles) do
        names[i] = jointAliases[i]
        positions[i] = sim.getJointPosition(jh)
        velocities[i] = sim.getJointVelocity(jh)
        efforts[i] = sim.getJointForce(jh)
    end
    
    local msg = {
        header = {
            stamp = simROS2.getSimulationTime(),
            frame_id = refFrameId
        },
        name = names,
        position = positions,
        velocity = velocities,
        effort = efforts
    }
    return msg
end