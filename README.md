# Coppelia Adapter for Ros2 Control
This package enables to use ros2_control with CoppeliaSim using the topic_based_ros2_control by XXX. With this not every robot needs it's own custom interface and kinematics programmed in CoppeliaSim. The hardware interface simply exposes the joint_velocities derived from the robot's URDF as topics, which CoppeliaSim can subscribe to. Further the hardware interface can also subscribe to the joint_states broadcasted from coppelia, which enable's the controller to publish odometry. Again this removes the need to implement the robot kinematics in CoppeliaSim.

## Setup Coppelia
The `sim_ros2_interface` for CoppeliaSim does not work with topics of type `sensor_msgs/msg/JointState` out of the box. All used topic types have to be added to the list in `meta/inerfaces.txt` and the package has to be recompiled following the instructions from the ROS2 Tutorial of the [CoppeliaSim Manual](https://manual.coppeliarobotics.com/).
> TIPP: If the Coppelia directory is inside our ROS2 workspace, it can be ignored at compilation by adding an empty file named `COLCON_IGNORE` to it. This avoids conflicts of duplicated packages.

## Launch Create 2 Example
1. Install all dependencies. Note that some of the dependencies are not available as binary and must be built manually. These are listed in the `.repos` file and can be pulled using vcstool.
2. Build the workspace.
3. Launch CoppeliaSim and open the scene `scenes/create_2_terrain.ttt`. Then start the simulation.
4. From a new terminal launch the `create_2_control.launch.py`. Run 
```bash
ros2 launch coppelia_ros2_control create_2_control.launch 
```
1. You can now teleoperate the robot (e.g. via teleop_twist_keyboard). The robot should now move in CoppeliaSim and also in RViz. RViz will show /odom and /map frame.

## Use with custom robot
Assumption: URDF/Xacro-File for Differential drive robot
1. Compile your URDF.xacro to a urdf file since coppelia cannot parse xacro.
2. Import the URDF into Coppelia Sim using the `Modules>Importers>URDF importer...`. In the import dialogue paste the correct path as a replacement for `package://` (usually the absolute path of the robot package)
3. Now attach the `scripts/attach-to-base-link.lua` to the `base_link_respondable`. (Feel free to make use of the other scripts in this folder.) (If you would like to attach the script to another link than base_link_respondable you need to adapt this in the map_odom_broadcaster.cpp as well.)
4. 