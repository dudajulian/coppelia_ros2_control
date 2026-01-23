-- // -- TODO: create header files
-- Modify these parameters to your liking
--- LEFT OFF: MAKE THIS A LIST AS IN CONTROLLER.YAML and make it work with Husky.
leftWheelJointName = "left_wheel_joint" -- should match the diff_drive_controller.yaml
rightWheelJointName = "right_wheel_joint" -- should match the diff_drive_controller.yaml
robotPoseTopicName = "/robot_pose"
-- // --

sim = require('sim')
simROS2 = require('simROS2')

function findIndex(t, value)
    for i, v in ipairs(t) do
        if v == value then
            return i
        end
    end
    return nil -- if not found
end

function setLeftMotorVelocity_cb(msg)
    -- Left motor speed subscriber callback
    sim.setJointTargetVelocity(leftMotor,msg.data)
end

function setRightMotorVelocity_cb(msg)
    -- Right motor speed subscriber callback
    sim.setJointTargetVelocity(rightMotor,msg.data)
end

function setCmd_cb(msg)
    lwi = findIndex(msg.name, leftWheelJointName)
    rwi = findIndex(msg.name, rightWheelJointName)
    sim.setJointTargetVelocity(leftMotor,msg.velocity[lwi])
    sim.setJointTargetVelocity(rightMotor,msg.velocity[rwi])
end

function getTransformStamped(objHandle, name, relTo, relToName)
    -- This function retrieves the stamped transform for a specific object
    local t = simROS2.getSimulationTime()
    local p = sim.getObjectPosition(objHandle,relTo)
    local o = sim.getObjectQuaternion(objHandle,relTo)
    return {
        header = {
            stamp = t,
            frame_id = relToName
        },
        child_frame_id = name,
        transform = {
            translation = {x = p[1], y = p[2], z = p[3]},
            rotation = {x = o[1], y = o[2], z = o[3], w = o[4]}
        }
    }
end

function getWheelJointState(refFrameId)
    local msg = {
        header = {
            stamp = simROS2.getSimulationTime(),
            frame_id = refFrameId
        },
        name = {leftWheelJointName, rightWheelJointName},
        position = {sim.getJointPosition(leftMotor), sim.getJointPosition(rightMotor)},
        velocity = {sim.getJointVelocity(leftMotor), sim.getJointVelocity(rightMotor)},
        effort = {sim.getJointForce(leftMotor), sim.getJointForce(rightMotor)}
    }
    return msg
end

function sysCall_init()
    -- The simulation script initialization
    objectHandle = sim.getObject('..')
    objectAlias = sim.getObjectAlias(objectHandle, 3)
    
    -- Get the handles for the two motors
    leftMotor=sim.getObject("../" .. leftWheelJointName) -- Handle of the left motor
    rightMotor=sim.getObject("../" .. rightWheelJointName) -- Handle of the right motor
    
    --Subscribers
    -- subLeftMotor=simROS2.createSubscription('/left_motor_speed','std_msgs/msg/Float32','setLeftMotorVelocity_cb')
    -- subRightMotor=simROS2.createSubscription('/right_motor_speed','std_msgs/msg/Float32','setRightMotorVelocity_cb')
    subCmd=simROS2.createSubscription('/topic_based_joint_commands','sensor_msgs/msg/JointState','setCmd_cb')
    
    -- Publishers
    pubSimTime = simROS2.createPublisher('/clock', 'rosgraph_msgs/msg/Clock')
    pubRobotPose = simROS2.createPublisher(robotPoseTopicName, 'geometry_msgs/msg/TransformStamped')
    pubJointState = simROS2.createPublisher('/topic_based_joint_states', 'sensor_msgs/msg/JointState')
end

function sysCall_actuation()
    -- Send an updated simulation time message, and send the transform of the object this script is attached to:
    simROS2.publish(pubSimTime, {clock = simROS2.getSimulationTime()})
    simROS2.publish(pubRobotPose, getTransformStamped(objectHandle, objectAlias, sim.handle_world, 'map'))
    simROS2.publish(pubJointState, getWheelJointState("base_link"))
    -- To send several transforms at once, use simROS2.sendTransforms instead
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- Following not really needed in a simulation script (i.e. automatically shut down at simulation end):
    simROS2.shutdownPublisher(pubJointState)
    simROS2.shutdownPublisher(pubRobotPose)
    simROS2.shutdownPublisher(pubSimTime)
    -- simROS2.shutdownSubscription(subLeftMotor)
    -- simROS2.shutdownSubscription(subRightMotor)
    simROS2.shutdownSubscription(subCmd)
end
