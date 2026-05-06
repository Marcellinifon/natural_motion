# Motion Shapes Design

## Overview

`motion_shapes_design.py` is a Python script for generating Cartesian drawing shapes and executing them on a robot using joint-space trajectory planning.

The script:
- connects to a robot over RPC,
- reads the current TCP pose,
- generates a Cartesian path for a selected shape,
- solves inverse kinematics for each pose,
- creates a smooth 6-DOF trajectory with Ruckig,
- streams the resulting joint commands to the robot using `ServoJ`.

## Supported Shapes

The script supports the following shapes:
- `circle`
- `ellipse`
- `square`
- `rectangle`
- `triangle`
- `arc`
- `s_curve`
- `wave`
- `star`
- `moon`
- `heart`
- `spiral`
- `infinity`
- `pentagon`
- `hexagon`
- `diamond`
- `cross`
- `arrow`

## Requirements

The script depends on the following Python packages and robot SDK modules:
- `fairino` (Robot RPC interface)
- `numpy`
- `ruckig`

The script is written for Python 3.13 and a robotics environment where:
- the robot controller supports `Robot.RPC(...)`,
- `GetInverseKin` is available,
- `MoveJ`, `ServoMoveStart`, `ServoJ`, `ServoMoveEnd`, and `CloseRPC` are available.

## Configuration

### Robot connection

- `ROBOT_IP` is set to `192.168.58.4`
- `dt` is set to `0.008` seconds (125 Hz control cycle)

### Ruckig limits

These motion limits are defined in joint-space units:
- `V_MAX` = [100, 100, 120, 150, 150, 180] deg/s
- `A_MAX` = [250, 250, 300, 400, 400, 400] deg/s²
- `J_MAX` = [1500, 1500, 2000, 3000, 3000, 3000] deg/s³

### Blend velocity

- `BLEND_VEL_DEG = 20.0`
- This controls intermediate waypoint blending in joint-space.
- `0.0` means the robot stops at every waypoint.
- A nonzero value causes smoother transitions through intermediate points.

## DH Parameters

The robot is described with the following Denavit-Hartenberg parameters for each joint:

| Joint | θ (rad) | a (mm) | d (mm) | α (rad) | Link | Mass (kg) | Center of mass (mm) |
|------:|--------:|-------:|-------:|--------:|:-----|----------:|:-------------------:|
| 1 | 0 | 0 | 152 | π/2 | Link 1 | 4.64 | [-0.19, -18.28, 2.26] |
| 2 | 0 | -425 | 0 | 0 | Link 2 | 10.08 | [212.47, 0, 121.2] |
| 3 | 0 | -395 | 0 | 0 | Link 3 | 2.71 | [122.62, 0.17, 12.59] |
| 4 | 0 | 0 | 102 | π/2 | Link 4 | 1.56 | [0.05, -2.33, 14.68] |
| 5 | 0 | 0 | 102 | -π/2 | Link 5 | 1.56 | [-0.05, 2.33, 14.68] |
| 6 | 0 | 0 | 100 | 0 | Link 6 | 0.36 | [0.93, 0.81, -20.05] |

> Note: These values describe the kinematic chain and center-of-mass data. The script itself does not compute dynamics from these values, but the DH table is useful for modeling and simulation.

## How the Script Works

### 1. Read current pose

- The script connects to the robot via RPC.
- It calls `GetActualTCPPose()` to read the current tool pose.
- The first six values of the returned pose are used as the shape center and orientation.

### 2. Ask user for shape and size

- The available shapes are displayed.
- The user inputs a shape name.
- For `ellipse`, the script asks for semi-major and semi-minor axes.
- For other shapes, it asks for a size value.

### 3. Generate Cartesian path

- The selected shape is generated in the XY plane around the current TCP pose.
- Orientation `rx`, `ry`, and `rz` remain fixed from the current pose.
- Cartesian points are returned as `[x, y, z, rx, ry, rz]`.

### 4. Inverse kinematics

- The function `cartesian_path_to_joint_path()` calls `robot.GetInverseKin(type=0, desc_pos=pose, config=-1)` for each waypoint.
- Generated joint values are converted from degrees to radians.
- Joint angles are unwrapped to avoid unnecessary revolutions using `unwrap_joints()`.

### 5. Trajectory generation

- `generate_ruckig_trajectory()` builds a continuous trajectory over the joint waypoints.
- It sets current and target positions, velocities, accelerations, and jerk limits.
- Intermediate waypoints can be blended using a nonzero `BLEND_VEL_DEG`.
- The final waypoint is enforced with zero target velocity.

### 6. Execution

- The script moves the robot to the first waypoint with a blocking `MoveJ()`.
- It then starts servo streaming with `ServoMoveStart()`.
- Each joint sample is sent with `ServoJ()` at a fixed 125 Hz timing.
- After the trajectory, `ServoMoveEnd()` stops servo streaming.
- The robot returns to the start configuration at the end.

## Running the Script

Run the script from the workspace directory:

```bash
python motion_shapes_design.py
```

During execution, you will be prompted to:
- select a shape,
- enter a size,
- optionally enter ellipse axes.

## Safety and Notes

- The script uses fixed real-time servo timing; ensure the robot is ready before running.
- Verify the current TCP pose and workspace clearance before execution.
- The path generation assumes planar shapes in the XY plane at constant height.
- If inverse kinematics fails for a generated pose, execution stops and reports the failing waypoint.

## File Summary

- `motion_shapes_design.py`: main script that generates shapes, solves IK, and executes a Ruckig trajectory on the robot.
- `README.md`: documentation for script usage, configuration, and robot kinematics.

## Recommended Improvements

If you extend this project, consider:
- adding a configuration file for robot IP, speeds, and limits,
- adding collision checking before execution,
- supporting variable orientation by rotating the shape plane,
- adding a visualizer for Cartesian and joint paths.
