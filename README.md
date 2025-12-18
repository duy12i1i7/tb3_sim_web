# Multi-TurtleBot3 Simulation running on any OS based on ROS 2 Jazzy & Gazebo Harmonic
This repository provides a scalable ROS 2-based framework to simulate multiple TurtleBot3 robots in Gazebo with Navigation2 (Nav2) support. Each robot runs within its own namespace, enabling clean separation and interaction-free operation.

## Prerequisites
- **Docker**: only Docker on any OS

### Install Dependencies
- If you are running on ARM architecture(like Mac M chip):
```
git clone https://github.com/duy12i1i7/tb3_sim_web.git
docker build --platform linux/arm64 -t tb3_multi_robot:jazzy-novnc .
```
- If you are running on AMD architecture:
```
git clone https://github.com/duy12i1i7/tb3_sim_web.git
docker build --platform linux/amd64 -t tb3_multi_robot:jazzy-novnc .
```

### Run Container + View result
```
docker run --rm -it \
  -p 6080:6080 -p 5900:5900 \
  --name tb3sim \
  tb3_multi_robot:jazzy-novnc
```
Afterthat, open Browser with URL:
```
http://localhost:6080/vnc.html
```

Open Terminal and run this command:
```
ros2 launch tb3_multi_robot tb3_world.launch.py
```
<img width="1840" height="1004" alt="image" src="https://github.com/user-attachments/assets/68d08e6a-8ab6-4f3d-98b3-504102b96312" />

After the simulation is launched, the system can either proceed with the Nav2 stack for autonomous navigation or use driving nodes for manual or scripted control.
A Python-based turtlebot3_drive script is included, replicating the original C++ node functionality while addressing compatibility issues with ROS 2 Jazzy and Gazebo Harmonic.

## 🚗 Launch Driving Nodes (Optional)
The original turtlebot3_drive application is not fully compatible with ROS 2 Jazzy and Gazebo Harmonic due to message type differences (e.g., use of TwistStamped for the /cmd_vel topic).
To address this, a Python-based equivalent is provided using compatible message types.

To launch the driving node for each robot, use the command below.
While a drive.launch.py file is included for automated multi-robot support, it is still under development and may require manual execution for each robot.

```
$ ./install/tb3_multi_robot/bin/turtlebot3_drive --ros-args -r __ns:=/tb1
```

Replace /tb1 with the appropriate robot namespace (/tb2, /tb3, etc.) as defined in robot configuration.

## 🧭 Launch Navigation2 Stack

With the robots running in Gazebo (via tb3_world.launch.py), the Navigation2 (Nav2) stack can be launched from a separate terminal.

```
$ ros2 launch  tb3_multi_robot tb3_nav2.launch.py
```
This will launch Nav2 nodes for all enabled robots using their respective namespaces.

<img width="2172" height="1721" alt="image" src="https://github.com/user-attachments/assets/d4c9e2ff-9721-4711-8e89-c25acbb3b207" />

The RViz2 panel title has been updated to include the corresponding robot name, making it easier to match each RViz instance with its respective robot in the Gazebo simulation.

### 🧭 Set Initial Pose
With RViz running, the robot’s initial position and orientation can be set using the 2D Pose Estimate button, aligned to its simulated placement and heading.

<img width="2180" height="1676" alt="image" src="https://github.com/user-attachments/assets/6a0c1bb0-32b1-4cf2-ad74-2da1ffba88e7" />

Alternatively, the initial pose can be set programmatically via the command line when the robot’s position and orientation are known from the simulation.

To retrieve the live pose of robots from Gazebo Harmonic, run:

```
$ gz topic -e -t /world/default/pose/info
```
This command lists the poses of all simulated entities. Identify the target robot by its name, such as tb1_waffle or tb1_burger.

Update the sample commands below with the position and orientation values obtained. The covariance typically remains unchanged.
Below commands are given for included robots.  User will need to update them for any custom robots. 

```
# TB1
$ ros2 topic pub --once /tb1/initialpose geometry_msgs/msg/PoseWithCovarianceStamped "header:
  frame_id: 'map'
pose:
  pose:
    position: {x: -1.500653720729433, y: -0.5000000060919606, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: -1.9562638e-06, w: 0.99999595}
  covariance: [0.25, 0, 0, 0, 0, 0,
               0, 0.25, 0, 0, 0, 0,
               0, 0, 0.0001, 0, 0, 0,
               0, 0, 0, 0.0001, 0, 0,
               0, 0, 0, 0, 0.0001, 0,
               0, 0, 0, 0, 0, 0.06853892]"
               
# TB2
$ ros2 topic pub --once /tb2/initialpose geometry_msgs/msg/PoseWithCovarianceStamped "header:
  frame_id: 'map'
pose:
  pose:
    position: {x: -1.500653720729433, y: 0.49999999390803895, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: -1.9562638e-06, w: 0.99999595}
  covariance: [0.25, 0, 0, 0, 0, 0,
               0, 0.25, 0, 0, 0, 0,
               0, 0, 0.0001, 0, 0, 0,
               0, 0, 0, 0.0001, 0, 0,
               0, 0, 0, 0, 0.0001, 0,
               0, 0, 0, 0, 0, 0.06853892]"
   
# TB3
$ ros2 topic pub --once /tb3/initialpose geometry_msgs/msg/PoseWithCovarianceStamped "header:
  frame_id: 'map'
pose:
  pose:
    position: {x: 1.499346279270567, y: -0.50000000609196049, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: -1.9562638e-06, w: 0.99999595}
  covariance: [0.25, 0, 0, 0, 0, 0,
               0, 0.25, 0, 0, 0, 0,
               0, 0, 0.0001, 0, 0, 0,
               0, 0, 0, 0.0001, 0, 0,
               0, 0, 0, 0, 0.0001, 0,
               0, 0, 0, 0, 0, 0.06853892]"
  
# TB4
$ ros2 topic pub --once /tb4/initialpose geometry_msgs/msg/PoseWithCovarianceStamped "header:
  frame_id: 'map'
pose:
  pose:
    position: {x: 1.499346279270567, y: 0.499999993908039, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: -1.9562638e-06, w: 0.99999595}
  covariance: [0.25, 0, 0, 0, 0, 0,
               0, 0.25, 0, 0, 0, 0,
               0, 0, 0.0001, 0, 0, 0,
               0, 0, 0, 0.0001, 0, 0,
               0, 0, 0, 0, 0.0001, 0,
               0, 0, 0, 0, 0, 0.06853892]"
```

### 🎯 Set Navigation Goal

A navigation goal may be sent using the 'Nav2 Goal' button in RViz after launching the Nav2 stack.

<img width="1091" height="842" alt="image" src="https://github.com/user-attachments/assets/99e0ebc3-09e8-47e6-be67-5743ca5e6d15" />

Alternatively, goals can be sent programmatically via the command line using ROS 2 action interface.

Below is an example command for setting a goal for tb1:

```
$ ros2 action send_goal /tb1/navigate_to_pose nav2_msgs/action/NavigateToPose \
'{
  pose: {
    header: {
      frame_id: "map"
    },
    pose: {
      position: {
        x: 2.0,
        y: 0.0,
        z: 0.0
      },
      orientation: {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0
      }
    }
  }
}'
```

Replace /tb1 with the appropriate robot namespace (e.g., /tb2, /tb3, etc.) and modify the x, y, and orientation fields to specify the desired goal pose.

## RQT Usage
rqt is a versatile tool for inspecting various ROS 2 data. In a multi-robot setup where each robot operates within its own namespace, specific configurations are required for correct usage.
Ensure that the /tf and /tf_static topics are mapped using absolute names (prefixed with /). Additionally, launch rqt within the desired robot's namespace to correctly visualize and interact with its respective topics.

```
rqt --ros-args -r __ns:=/tb1 -r /tf:=tf -r /tf_static:=tf_static
```

<img width="1630" height="1572" alt="image" src="https://github.com/user-attachments/assets/a8f4221b-705e-4b52-ab04-03916a60de08" />

## Robot Configuration

The placement and activation of individual robots are defined in the config/robots.yaml file. Each robot is assigned a unique name and initial position. Set the enabled flag to true to include a robot in the simulation or false to exclude it.

Below is an example configuration used in the Nav2 simulation:

```
 robots:
  - name: tb1
    x_pose: "-1.5"
    y_pose: "-0.5"
    z_pose: 0.01
    enabled: true
  - name: tb2
    x_pose: "-1.5"
    y_pose: "0.5"
    z_pose: 0.01
    enabled: false
  - name: tb3
    x_pose: "1.5"
    y_pose: "-0.5"
    z_pose: 0.01
    enabled: true
  - name: tb4
    x_pose: "1.5"
    y_pose: "0.5"
    z_pose: 0.01
    enabled: false
```
💡 Robots can be enabled or disabled by updating the enabled field, and their starting poses can be configured by modifying the position values accordingly.

## Improving performance

Simulating multiple robots (especially 4 or more) can be demanding on system resources. Below are some suggestions to help optimize performance:

### 1. Limit Robot Count
Running fewer robots significantly reduces CPU and memory load. Consider limiting the simulation to 2 robots if performance is a concern.

### 2. Lower Simulation Update Rates
Reduce the update frequency in the Gazebo .world file to ease the physics computation load. In worlds/tb3_world.world, modify the <physics> block as follows:

```
<physics type="ode">
      <real_time_update_rate>100.0</real_time_update_rate>
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1</real_time_factor>
```

🔧 Lower real_time_update_rate and higher max_step_size lead to fewer simulation steps per second, improving runtime performance.

### 3. Reduce Topic Frequency
Consider modifying the robot model or relevant plugins to reduce the frequency of published topics (e.g., /odom, /tf, /scan) if they are not critical at high rates.

# FAQ
**Why are /tf and /tf_static explicitly remapped in RViz and other nodes, even when a namespace is applied?**

Although nodes like rviz2 can be launched with a specific namespace, the TransformListener in tf2_ros subscribes to the global topics /tf and /tf_static by default. This behavior stems from how the listener is implemented internally — the topic names are hardcoded with a leading slash, making them absolute paths (refer to /opt/ros/jazzy/include/tf2_ros/transform_listener.h for reference).

This can be quickly verified using the following command:

```
rviz2 --ros-args -r __ns:=/tb1
```

Despite the namespace, the resulting RViz instance still subscribes to /tf and /tf_static globally.

In a multi-robot configuration, where each robot is designed to operate with an isolated TF tree, remapping /tf to tf (a relative topic) ensures proper namespacing. This approach prevents conflicts and guarantees that TF messages remain within the intended robot scope.

Such remapping is also necessary in other components (e.g., Nav2) that instantiate their own transform listeners. Applying consistent remapping avoids unintended cross-robot data mixing and supports clean separation of transform data across all robot instances.

# 📎 Note on Included Files

Some of configuration and model files (e.g., from turtlebot3 and nav2) have been directly copied into this repository. These were modified to better suit the multi-robot simulation and to ensure long-term consistency and reproducibility—even if the original upstream repositories evolve or change in the future. All original credit for these files remains with their respective authors and maintainers.


