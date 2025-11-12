# Robotics Lab 2025 - Homework 2

**Arena Giuseppe - Ruggiero Nicholas**

## Description

This repository contains the solution for Homework 2 of the Robotics Lab 2025 course. The project focuses on developing and implementing kinematic and vision-based controllers for a KUKA iiwa manipulator arm within the ROS 2 simulation environment. It utilizes the Kinematics and Dynamics Library (KDL) for kinematic control and the `aruco_ros` package for vision-based setup.

The implemented solutions cover points 1A, 1B, 1C, 2A, 2B, and 2C from the official homework assignment.

## Prerequisites

Before building, ensure you have a working ROS 2 Humble Hawksbill environment installed. The project relies on the following key components and packages:

* **ROS 2 Humble** (and its core dependencies)
* **Gazebo** (for simulation)
* **ros2_control** (for hardware/sim interfacing)
* **MoveIt 2** (for planning components)
* **KDL** (via `orocos_kdl_vendor` or similar ROS 2 packages)
* **Aruco_ROS** (included in this workspace)

## Build Instructions

1.  Place all provided packages (`ros2_iiwa-main`, `ros2_kdl_package-main`, `aruco_ros-humble-devel`) into the `src` folder of your ROS 2 workspace.
2.  Navigate to the root of your workspace and build the packages:

    ```bash
    cd /path/to/your/ros2_ws
    colcon build
    ```

3.  Source the workspace to make the nodes and launch files available in your environment:

    ```bash
    source install/setup.bash
    ```

---

## Execution Instructions (How to Run)

This section details the specific commands to launch and test the solution for each completed point of the assignment.

### Point 1A: ROS 2 Parameters and Launch Files

**Objective:** This task involved modifying the `ros2_kdl_node` to accept key variables as ROS 2 parameters (e.g., `traj_duration`, `acc_duration`, `Kp`, `end_position`). These parameters are loaded from a `.yaml` configuration file using a dedicated launch file.

**Commands:**

* **Terminal 1:** Launch the robot interface.
    ```bash
    ros2 launch iiwa_bringup iiwa.launch.py
    ```

* **Terminal 2:** Launch the KDL node, which will load the parameters from the config file.
    ```bash
    ros2 launch ros2_kdl_package kdl_node.launch.py
    ```
    *You can verify in the terminal output that the parameters from the `.yaml` file have been correctly loaded.*

---

### Point 1B: Velocity Control with Null-Space Optimization

**Objective:** This task implements a new velocity controller, `velocity_ctrl_null`, which utilizes the Jacobian pseudoinverse and a null-space component to manage redundancy. The control law is $\dot{q}=J^{\dagger}K_{p}e_{p}+(I-J^{\dagger}J)\dot{q}_{0}$.

The following commands allow for testing both the standard controller (`velocity_ctrl`) and the new null-space controller (`velocity_ctrl_null`) for comparison.

**Commands:**

* **Terminal 1 (Common):** Launch the robot interface, specifying the `velocity` command interface.
    ```bash
    ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"
    ```

* **Terminal 2 (Case 1: Standard Control):** Launch the KDL node using the standard `velocity_ctrl`.
    ```bash
    ros2 launch ros2_kdl_package kdl_node.launch.py cmd_interface:=velocity ctrl:=velocity_ctrl
    ```

* **Terminal 2 (Case 2: Null-Space Control):** Launch the KDL node using the new `velocity_ctrl_null`.
    ```bash
    ros2 launch ros2_kdl_package kdl_node.launch.py cmd_interface:=velocity ctrl:=velocity_ctrl_null
    ```

---

### Point 1C: ROS 2 Action Client

**Objective:** This task demonstrates the execution of a trajectory using a ROS 2 Action client. The `ros2_kdl_node` (configured for null-space control) acts as the Action server, waiting for a goal from the client to begin the trajectory.

**Commands:**

* **Terminal 1:** Launch the robot interface with the velocity controller.
    ```bash
    ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"
    ```

* **Terminal 2:** Launch the KDL node, which will start the Action server and wait for a goal.
    ```bash
    ros2 launch ros2_kdl_package kdl_node.launch.py cmd_interface:=velocity ctrl:=velocity_ctrl_null
    ```

* **Terminal 3:** Run the Action client to send the goal and trigger the trajectory execution.
    ```bash
    ros2 run ros2_kdl_package ros2_kdl_action_client
    ```

---

### Point 2A: Gazebo Simulation with Aruco Marker

**Objective:** This task involves launching the Gazebo simulation environment. The world contains the KUKA iiwa robot (equipped with a camera) and an Aruco marker. These commands are used to verify that the simulation is running, the camera topics are active, and the `aruco_ros` node is correctly identifying and publishing the marker's pose.

**Commands:**

* **Terminal 1:** Launch the Gazebo simulation with the camera-equipped robot and Aruco tag.
    ```bash
    ros2 launch iiwa_bringup iiwa.launch.py use_sim:=true
    ```

* **Verification (Optional):** To verify that the camera topics are active, in a new terminal:
    ```bash
    ros2 topic list
    ```
    *(Look for topics like `/camera/image_raw` and `/camera/camera_info`)*

* **Verification (Optional):** To verify that the Aruco marker is being detected, in a new terminal:
    ```bash
    ros2 topic echo /aruco_single/pose
    ```
    *(This will show the pose of the detected marker relative to the camera.)*

---

### Point 2B: Vision-Based Control

**Objective:** This task implements vision-based control (visual servoing). The KDL node (`ctrl:=vision`) uses the Aruco marker's pose (detected in point 2A) to guide the robot's end-effector towards the target.

**Commands:**

* **Terminal 1:** Launch the robot interface in simulation with the velocity controller.
    ```bash
    ros2 launch iiwa_bringup iiwa.launch.py use_sim:=true command_interface:=velocity robot_controller:=velocity_controller
    ```

* **Terminal 2:** Launch the KDL node using the vision-based controller.
    ```bash
    ros2 launch ros2_kdl_package kdl_node.launch.py cmd_interface:=velocity ctrl:=vision
    ```

---

### Point 2C: Moving the Aruco Marker

**Objective:** This command demonstrates how to dynamically move the Aruco marker in the Gazebo simulation environment using a ROS 2 service call. This is useful for testing the visual controller's ability to react to a moving target.

**Commands:**

* **Terminal (New):** Execute the following service call to move the Aruco marker to a new pose (e.g., `x: 0.7, y: 0.2, z: 1.1`).
    ```bash
    ros2 service call /set_marker_pose ros_gz_interfaces/srv/SetEntityPose '{
      entity: {
        name: "arucotag",
        id: 0,
        type: 2
      },
      pose: {
        position: {x: 0.7, y: 0.2, z: 1.1},
        orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
      }
    }'
    ```
