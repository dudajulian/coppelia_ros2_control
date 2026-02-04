# CoppeliaSim Adapter for ROS2 Control
This package connects **ros2_control** with **CoppeliaSim** via a hardware interface that exposes joint velocities as topics.

It removes the need to manually program robot interfaces and kinematics in CoppeliaSim. Instead, joint velocities are published as topics for CoppeliaSim to use. The hardware interface also listens to joint states from CoppeliaSim, allowing odometry to be published. With this setup, you can directly use all the standard **ros2_control** controllers for controlling the robot in CoppeliaSim.
*(This package was tested for ROS2-humble. Please let me know if you have tested it sucessfully with other versions.)*

---

## Quickstart

### 1. Source-Based Third-Party Dependencies
This package requires the following third-party repositories to be built from source (no binary packages are provided):
- [`topic_based_ros2_control` by PickNik Robotics](https://github.com/PickNikRobotics/topic_based_ros2_control)
- [`twist_stamper` by Josh Newans](https://github.com/joshnewans/twist_stamper)

To fetch these repositories into the `src/` directory of your workspace, run the following command:
```bash
vcs import src < src/coppelia_ros2_control/.repos
```

### 2. Setup CoppeliaSim

The `sim_ros2_interface` for CoppeliaSim does not natively support all ROS2 message types needed. To make them available the package must be compiled manually:

1. Locate the package folder at `CoppeliaSim/programming/ros2_packages/sim_ros2_interface` and copy it to `src/` of your workspace.

2. Add all required message types to `meta/interfaces.txt` inside the copied package:
```txt
sensor_msgs/msg/JointState
rosgraph_msgs/msg/Clock
```

3. Recompile the package following the [ROS2 Tutorial in the CoppeliaSim Manual](https://manual.coppeliarobotics.com/).


### 3. Build your workspace

After pulling the dependencies, build your workspace. Using `--symlink-install` is recommended to avoid unnecessary rebuilds when making file changes during development.

```bash
rosdep install --from-paths src --ignore-src -y && colcon build --symlink-install
```

### 4. Launch the Example

#### 1. Start the Simulation
Open the provided `scenes/example.ttt` file in **CoppeliaSim** and start the simulation by clicking the **play button**.

#### 2. Launch ROS2 Control
Run the following command:
```bash
ros2 launch coppelia_ros2_control coppelia_control.launch.py
```
This command launches:
- A **controller** that interacts with the topic-based Hardware Interface (HWI) and all required nodes (e.g., `joint_state_broadcaster`).
- A **`twist_stamper` node**, which accepts unstamped twist messages on the `/cmd_vel` topic for simplified teleoperation integration.
- A **`map_odom_broadcaster` node**, which listens to the `/robot_pose` topic (a transform from `map` to `base_link`) and publishes a **REP 105-compliant** transform from `map` to `odom` on the `/tf` topic.

#### 3. Teleoperate the Robot
Publish command velocities to the `/cmd_vel` topic (e.g., using `teleop_twist_keyboard`) to control the robot in **CoppeliaSim**. Verify the **TF tree** in **RViz** to ensure everything is functioning correctly.

---

## Use with a Custom Robot

### Prerequisites
- **URDF/Xacro** file for your differential drive robot (e.g. `robot.urdf.xacro`)
- **controller configuration** file file (e.g. `control.yaml`)
   
### 1. Modify the <ros2_control> Tag 
Follow the **[topic_based_ros2_control instructions](https://github.com/PickNikRobotics/topic_based_ros2_control/blob/main/doc/user.md)** to edit your **URDF/Xacro** file. Typically this means:

1. Locate the `<ros2_control>` section in your description files.
2. Replace the content inside the `<hardware>` tag with:
   ```xml
   <plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
   <param name="joint_commands_topic">topic_based_joint_commands</param>
   <param name="joint_states_topic">topic_based_joint_states</param>
   <param name="sum_wrapped_joint_states">false</param>
   <param name="trigger_joint_command_threshold">0.001</param>
   ```
Refer to `description/urdf/example.urdf` for guidance.

> **Note:** If your robot's URDF/Xacro already includes **separate `<hardware>` sections** for simulation and real-world use (e.g., controlled by a condition like `is_sim`), only modify the **simulation-specific section**. Leave the real-world configuration unchanged.

### 1.5 Compile Xacro to URDF:
CoppeliaSim does not support Xacro directly. If your robot's description is a XACRO convert it to URDF: 
```bash
xacro robot.urdf.xacro > robot.urdf
```
> **Note:** For **Clearpath robots**, use their [live editor](https://docs.clearpathrobotics.com/docs/ros/config/live/) to generate the `robot.urdf.xacro`.

### 2. Import URDF into CoppeliaSim:
   - Open CoppeliaSim and navigate to `Modules > Importers > URDF importer...`.
   - When prompted, provide the absolute path to your robot’s package directory to resolve package:// references. Ensure the path ends with a / for correct mesh importing.
Your robot should now appear in the simulation.

### 3. Configure Dynamic Properties:
In CoppeliaSim:
   - Ensure the `base_link_respondable` object and all wheel joint respondables are set to **dynamic**.
   - Configure all controlled joints to use **velocity control** and set their target value to `0.0` to prevent unintended movement before sending commands.

### **4. Attach Lua Script**
1. Attach a new script to the **`base_link_respondable`** object. (`Add > Script > simulation script > Non threaded > Lua`)
2. Copy the contents of **`scripts/attach-to-base-link.lua`** into the script.
   - If attaching to a different object, update **`map_odom_broadcaster.cpp`** accordingly.

### **5. Set Joint Names**
Update **`leftWheelJointNames`** and **`rightWheelJointNames`** in the script to match your robot’s joint names.

### **6. Update Control Configuration**
Add the following line to your **`control.yaml`** file:
```yaml
cmd_vel_topic: "/cmd_vel"
```
Refer to `config/example_control.yaml` for guidance.

### **7. Start the Simulation**
- Press **Play** in CoppeliaSim to begin the simulation.

### **8. Launch ROS2 Control**
Run the following command, replacing placeholders with your specific parameters:
```bash
ros2 launch coppelia_ros2_control coppelia_control.launch.py controller_name:=<controller_node_name> controller_config_path:="/path/to/control.yaml/" robot_description_path:="path/to/robot.urdf.xacro"
```
- **`controller_name`**: Name of the controller node from `control.yaml`.
- **`controller_config_path`**: Path to your `control.yaml` file.
- **`robot_description_path`**: Path to your robot’s URDF/Xacro file.

---

### **Credits**
This project was developed and maintained by Julian Duda

For questions, suggestions, or contributions, feel free to reach out:
- **GitHub:** [@dudajulian](https://github.com/dudajulian)
- **LinkedIn:** [@julianduda](https://www.linkedin.com/in/julianduda)
