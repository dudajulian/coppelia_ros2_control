--[[
===============================================================================
  Vision Sensor ROS 2 Interface for CoppeliaSim
  -------------------------------------------------------------------------------
  Package:    coppelia_ros2_control (Pen Limit)
  Author:     Julian Duda (Pen Limit)
  Description:
    Generic Lua script for vision sensors in CoppeliaSim.
    Publishes RGB images, depth images, and point clouds as ROS 2 topics.
    Attach this script to any vision sensor in CoppeliaSim.

  -------------------------------------------------------------------------------
  Topics:
    - Publishes:
      - RGB Image:          Configurable (default: /rgb/color/rect/image)
      - Depth Image:        Configurable (default: zed/zed_node/depth/depth_registered)
      - Point Cloud:        Configurable (default: zed/zed_node/point_cloud/cloud_registered)

  -------------------------------------------------------------------------------
  Dependencies:
    - CoppeliaSim API (sim, simROS2, simVision)
    - ROS 2 (sensor_msgs/msg/Image, sensor_msgs/msg/PointCloud2)

  -------------------------------------------------------------------------------
  Callbacks:
    - sysCall_init():      Initializes sensor handles and ROS 2 publishers.
    - sysCall_vision():    Processes vision data and publishes point clouds.
    - sysCall_sensing():   Publishes RGB and depth images.
    - sysCall_cleanup():   Shuts down ROS 2 publishers.
===============================================================================
--]]

-- Modify these parameters to your liking
imgTopicName='/rgb/color/rect/image'
depthImgTopicName='zed/zed_node/depth/depth_registered'
pointCloudTopicName='zed/zed_node/point_cloud/cloud_registered'
-- -- --

sim=require'sim'
simROS2=require'simROS2'
simVision=require'simVision'

-- ZED2 simulation: attach this script to a vision sensor

-- Helper to pack r,g,b (0-255) into a float32 as expected by ROS PointCloud2
function packRGBToFloat(r, g, b)
    local rgb = r * 2^16 + g * 2^8 + b
    -- Now reinterpret the int as a float32 with sim.packUInt32Table/unpackFloatTable
    return sim.unpackFloatTable(sim.packUInt32Table({rgb}))[1]
end

-- pts: {resX, resY, x1, y1, z1, d1, x2, y2, z2, d2, ...}
-- cols: {r1, g1, b1, r2, g2, b2, ...}
function makePackedPointCloud2(pts, cols)
    local packed = {}
    local resX, resY = pts[1], pts[2]
    local num_points = (#pts - 2) // 4
    for i=1,num_points do
        local idx = 2 + (i-1)*4
        local x = pts[idx+1]
        local y = pts[idx+2]
        local z = pts[idx+3]
        -- local d = pts[idx+4] -- not used for pointcloud
        local r = cols[(i-1)*3+1]
        local g = cols[(i-1)*3+2]
        local b = cols[(i-1)*3+3]
        local rgb_float = packRGBToFloat(r, g, b)
        table.insert(packed, x)
        table.insert(packed, y)
        table.insert(packed, z)
        table.insert(packed, rgb_float)
    end
    return packed -- This is {x1, y1, z1, rgb1, x2, y2, z2, rgb2, ...}
end

function sysCall_init()

    -- Get sensor handle
    activeVisionSensor=sim.getObject('..')
    
    resX=sim.getObjectInt32Param(activeVisionSensor,sim.visionintparam_resolution_x)
    resY=sim.getObjectInt32Param(activeVisionSensor,sim.visionintparam_resolution_y)

    -- Enable an image publisher and subscriber:
    pubImage=simROS2.createPublisher(imgTopicName, 'sensor_msgs/msg/Image')
    simROS2.publisherTreatUInt8ArrayAsString(pubImage) 
    -- treat uint8 arrays as strings (much faster, tables/arrays are kind of slow in Lua)
    
    pubDepth=simROS2.createPublisher(depthImgTopicName, 'sensor_msgs/msg/Image')
    simROS2.publisherTreatUInt8ArrayAsString(pubDepth) 
    -- treat uint8 arrays as strings (much faster, tables/arrays are kind of slow in Lua)
    
    pubPointCloud2=simROS2.createPublisher(pointCloudTopicName, 'sensor_msgs/msg/PointCloud2')
    simROS2.publisherTreatUInt8ArrayAsString(pubPointCloud2) 
    -- treat uint8 arrays as strings (much faster, tables/arrays are kind of slow in Lua)

end

function sysCall_vision(inData)
    
    simVision.sensorImgToWorkImg(inData.handle)
    simVision.workImgToBuffer1(inData.handle)
    simVision.sensorDepthMapToWorkImg(inData.handle)
    local _,pts,cols=simVision.coordinatesFromWorkImg(inData.handle|sim.handleflag_abscoords,{resX,resY},false,true)
    
    pts=sim.unpackFloatTable(pts)
    cols=sim.unpackUInt8Table(cols)
    
    local msg_data = sim.packFloatTable(makePackedPointCloud2(pts, cols))
    
    d={}
    d.header={stamp=simROS2.getSimulationTime(), frame_id="map"}
    d.height=pts[2] --resY
    d.width=pts[1] --resX
    d.fields={
        {name="x", offset=0, datatype=7, count=1},
        {name="y", offset=4, datatype=7, count=1},
        {name="z", offset=8, datatype=7, count=1},
        {name="rgb", offset=12, datatype=7, count=1}
    }
    d.is_bigendian=false
    d.point_step=4*3+4
    d.row_step=pts[1]*d.point_step
    --sim.transformImage(data,{d.width,d.height},4)
    d.data=msg_data
    simROS2.publish(pubPointCloud2,d)
    
    return {}
end

function sysCall_sensing()
    -- Publish the image of the active vision sensor:
    local data,resolution=sim.getVisionSensorImg(activeVisionSensor)
    
    d={}
    d.header={stamp=simROS2.getSimulationTime(), frame_id='a'}
    d.height=resolution[2]
    d.width=resolution[1]
    d.encoding='rgb8'
    d.is_bigendian=0
    d.step=resolution[1]*3
    sim.transformImage(data,{d.width,d.height},4)
    d.data=data
    simROS2.publish(pubImage,d)
    
    -- Publish the depth image of the active vision sensor:
    local data,resolution=sim.getVisionSensorDepth(activeVisionSensor)
    
    local floatTable = sim.unpackFloatTable(data)
    
    d={}
    d.header={stamp=simROS2.getSimulationTime(), frame_id='a'}
    d.height=resolution[2]
    d.width=resolution[1]
    d.encoding='32FC1'
    d.is_bigendian=0
    d.step=resolution[1]*4
    sim.transformImage(data,{d.width,d.height},4)
    
    
    d.data=data
    simROS2.publish(pubDepth,d)
end

function sysCall_cleanup()
    -- Shut down publisher and subscriber. Not really needed from a simulation script (automatic shutdown)
    simROS2.shutdownPublisher(pubImage)
    simROS2.shutdownPublisher(pubDepth)
    simROS2.shutdownPublisher(pubPointCloud2)
end