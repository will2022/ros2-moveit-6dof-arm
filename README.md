# ROS 2 MoveIt 6-DOF Arm

A complete MoveIt 2 setup for a 6-DOF robot arm, targeting **ROS 2 Jazzy Jalisco**.

## Packages

- **`my_robot_description`**: URDF/Xacro description, visual/collision geometry, and ros2_control integration.
- **`my_robot_moveit_config`**: MoveIt 2 configuration (SRDF, kinematics, OMPL, controllers) and launch files.
- **`my_robot_bringup`**: System integration, controller manager configuration, and main bringup launch.

## Prerequisites

### Option 1: Docker (Recommended for macOS/Windows)
- **Docker Desktop** installed ([Download](https://www.docker.com/products/docker-desktop))

### Option 2: Native Installation (Linux)
- **ROS 2 Jazzy** installed
- **MoveIt 2** installed (`sudo apt install ros-jazzy-moveit`)
- **ros2_control** installed (`sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers`)

## Build Instructions

### Using Docker

1.  **Clone the repository**:
    ```bash
    git clone https://github.com/will2022/ros2-moveit-6dof-arm.git
    cd ros2-moveit-6dof-arm
    ```

2.  **Build the Docker image**:
    ```bash
    docker-compose build
    ```

3.  **Build the workspace**:
    ```bash
    ./scripts/docker_build.sh
    ```

4.  **Start an interactive shell**:
    ```bash
    ./scripts/docker_shell.sh
    ```

### Native Installation (Linux)

1.  **Create a workspace and clone**:
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone https://github.com/will2022/ros2-moveit-6dof-arm.git .
    cd ~/ros2_ws
    ```

2.  **Source ROS 2 Jazzy**:
    ```bash
    source /opt/ros/jazzy/setup.bash
    ```

3.  **Install dependencies**:
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```

4.  **Build the workspace**:
    ```bash
    colcon build
    ```

5.  **Source the workspace**:
    ```bash
    source install/setup.bash
    ```

## Running the Demo

### 1. Full Bringup (Robot + MoveIt + RViz)

This is the standard workflow for simulation.

**Terminal 1: Bringup**
Starts the mock hardware, controller manager, and spawns controllers.
```bash
ros2 launch my_robot_bringup bringup.launch.py
```

**Terminal 2: MoveIt**
Starts the MoveIt planning pipeline and RViz.
```bash
ros2 launch my_robot_moveit_config moveit.launch.py
```

### 2. Operator Workflow

1.  In RViz, you should see the robot model.
2.  Use the **MotionPlanning** plugin in RViz.
3.  Drag the interactive marker (ball/arrows) at the end effector to a new pose.
4.  Click **Plan & Execute** in the MotionPlanning panel.
5.  The robot should move to the target pose.

### 3. Python Example

Run the example node to perform automated planning and execution via the MoveItPy API.

```bash
ros2 run my_robot_bringup planning_example
```

## Configuration Details

### Kinematics
Default solver: **KDL**.
To switch to **TracIK** (recommended for higher success rates):
1.  Install: `sudo apt install ros-jazzy-trac-ik-kinematics-plugin`
2.  Edit `src/my_robot_moveit_config/config/kinematics.yaml`:
    ```yaml
    manipulator:
      kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
    ```

### Controllers
Configured in `src/my_robot_bringup/config/my_robot_controllers.yaml`.
- `joint_state_broadcaster`: Publishes `/joint_states`
- `joint_trajectory_controller`: Follows `FollowJointTrajectory` action

## Troubleshooting (Jazzy)

-   **Plugin Names**: Ensure `ompl_interface/OMPLPlanner` is used in `ompl_planning.yaml`.
-   **Middleware**: If you experience communication issues, try setting `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.
-   **Simulation Time**: If running with Gazebo (future extension), ensure `use_sim_time:=true` is passed to launch files.

## Directory Structure

```
my_robot_workspace/
└── src/
    ├── my_robot_description/   # Robot model
    ├── my_robot_moveit_config/ # MoveIt config
    └── my_robot_bringup/       # Integration
```
