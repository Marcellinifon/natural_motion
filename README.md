<div align="center">

# Natural Manipulator Motion: An Approach Based on Principles of Biomechanics and Animation*

<p align="center"><strong>Wakam Tchinda Henoch · Guangwei Li · Suanon Ifon Felix Marcellin · Haojun Qu · Jinping Li</strong></p>


</div>

---

## Abstract

<p align="justify">Natural, human-like motion is not only elegant but also essential for the safe integration of robots into human spaces. While learning-driven approaches such as imitation learning and reinforcement learning have shown promising results, optimizing them for human-like behavior remains challenging due to high data requirements and the difficulty of formalizing 'naturalness' within reward functions. In contrast, principle-driven methods based on biomechanics or animation offer more interpretable solutions that extend to new tasks without retraining, but often focus on isolated aspects of motion, neglecting the structural coupling between physical efficiency and perceptual quality. This paper introduces a unified, principle-driven framework for generating natural manipulator motion that operates independently of costly data collection or blind reward engineering. We formulate a definition of natural motion based on three fundamental pillars: expressiveness, efficiency, and smoothness. These are mapped into a three-tier architecture in which (i) expressive intent is encoded using animation principles in structured motion primitives, (ii) efficient configurations are obtained through constrained inverse kinematics that minimizes unnecessary joint motion, and (iii) smooth execution is achieved via jerk-limited trajectory generation. The proposed method is validated on a 6 degree of freedom collaborative robot (Fairino FR5) across a range of interactive tasks. Results show that the generated motions exhibit life-like kinematic and dynamic properties, while operating within the 250 µs control cycle of the target platform. A user study further indicates that 72% of participants perceived the proposed motions as more natural than standard industrial motion profiles.</p>

---
## Demo
<video src="C:\project\fr_motion_shapes_design\assets\demo.mp4" width="100%" controls></video>

## 🚀 Overview

`motion_shapes_design.py` is a Python script for generating Cartesian drawing shapes and executing them on a robot using joint-space trajectory planning.

The script:
- connects to a robot over RPC,
- reads the current TCP pose,
- generates a Cartesian path for a selected shape,
- solves inverse kinematics for each pose,
- creates a smooth 6-DOF trajectory with Ruckig,
- streams the resulting joint commands to the robot using `ServoJ`.

## 🧩 Drawing shapes for investigating RUCKIG parameters

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

## ⚙️ Requirements

The script depends on the following Python packages and robot SDK modules:
- `fairino` (Robot RPC interface — refer to the Fairino manual/documentation for installation and usage details)
- `numpy`
- `ruckig`

The script is written for Python 3.13 and a robotics environment where:
- the robot controller supports `Robot.RPC(...)`,
- `GetInverseKin` is available,
- `MoveJ`, `ServoMoveStart`, `ServoJ`, `ServoMoveEnd`, and `CloseRPC` are available.

## ⚙️ Configuration

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

## 📐 DH Parameters

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

## 🧠 How the Script Works

### 1. Read current pose 🧭

- The script connects to the robot via RPC.
- It calls `GetActualTCPPose()` to read the current tool pose.
- The first six values of the returned pose are used as the shape center and orientation.

### 2. Ask user for shape and size 🎯

- The available shapes are displayed.
- The user inputs a shape name.
- For `ellipse`, the script asks for semi-major and semi-minor axes.
- For other shapes, it asks for a size value.

### 3. Generate Cartesian path 📍

- The selected shape is generated in the XY plane around the current TCP pose.
- Orientation `rx`, `ry`, and `rz` remain fixed from the current pose.
- Cartesian points are returned as `[x, y, z, rx, ry, rz]`.

### 4. Inverse kinematics 🔄

- The function `cartesian_path_to_joint_path()` calls `robot.GetInverseKin(type=0, desc_pos=pose, config=-1)` for each waypoint.
- Generated joint values are converted from degrees to radians.
- Joint angles are unwrapped to avoid unnecessary revolutions using `unwrap_joints()`.

### 5. Trajectory generation 🛣️

- `generate_ruckig_trajectory()` builds a continuous trajectory over the joint waypoints.
- It sets current and target positions, velocities, accelerations, and jerk limits.
- Intermediate waypoints can be blended using a nonzero `BLEND_VEL_DEG`.
- The final waypoint is enforced with zero target velocity.

### 6. Execution ▶️

- The script moves the robot to the first waypoint with a blocking `MoveJ()`.
- It then starts servo streaming with `ServoMoveStart()`.
- Each joint sample is sent with `ServoJ()` at a fixed 125 Hz timing.
- After the trajectory, `ServoMoveEnd()` stops servo streaming.
- The robot returns to the start configuration at the end.

## ▶️ Running the Script

Run the script from the workspace directory:

```bash
python motion_shapes_design.py
```

During execution, you will be prompted to:
- select a shape,
- enter a size,
- optionally enter ellipse axes.

## ⚠️ Safety and Notes

- The script uses fixed real-time servo timing; ensure the robot is ready before running.
- Verify the current TCP pose and workspace clearance before execution.
- The path generation assumes planar shapes in the XY plane at constant height.
- If inverse kinematics fails for a generated pose, execution stops and reports the failing waypoint.

## 📦 Natural robot motion blocks 

### `nod_yes.py`

- Purpose: run a ``yes`` nod gesture by moving the robot through a predefined head/upper-body motion sequence.
- Motion sequence: ``ALERT`` → ``NOD_BASE`` → ``NOD_DOWN_FWD`` → ``NOD_BASE`` → ``NOD_DOWN_FWD`` → ``NOD_BASE`` → ``ALERT``.
- Pose definitions are expressed as Cartesian ``[x, y, z, rx, ry, rz]`` values with the option to switch to direct joint-space waypoints.
- The script uses a custom F5 forward-kinematics model and SciPy optimization to solve inverse kinematics for each Cartesian waypoint.
- After IK, it builds a Ruckig-controlled trajectory and streams joint updates with ``ServoJ`` at 125 Hz.

**Example usage:**

```bash
python nod_yes.py
```

**What happens:**
- the robot first moves to the ``ALERT`` start posture using ``MoveJ``,
- then the script executes the nod sequence over a smooth servo trajectory,
- finally it holds the ``ALERT`` pose.

### `refuse_no.py`

- Purpose: execute a ``no`` refusal gesture using a side-to-side motion.
- Motion sequence: ``ALERT`` → ``REFUSE_BASE`` → ``REFUSE_LEFT`` → ``REFUSE_BASE`` → ``REFUSE_RIGHT`` → ``REFUSE_BASE`` → ``REFUSE_LEFT`` → ``REFUSE_BASE`` → ``ALERT``.
- The script uses the same FK/IK solver as ``nod_yes.py`` to ensure the Cartesian pose definitions map cleanly to feasible joint configurations.
- It generates one continuous Ruckig trajectory for the whole gesture and streams it with ``ServoJ``.
- A direct joint-space fallback is available by uncommenting the joint pose definitions and bypassing the IK solver.

**Example usage:**

```bash
python refuse_no.py
```

**What happens:**
- the robot moves to the initial ``ALERT`` pose,
- performs the refusal head-shake gesture,
- returns to ``ALERT`` and holds.

### `Use_Case1_Rehabilitation.py`

- Purpose: demonstrate a rehabilitation-themed medical inspection behavior with branching outcome logic.
- Main behavior sequences:
  - ``LEAN_SEQ``: move from the alert posture into a closer inspection pose,
  - ``INSPECT_SEQ``: sweep left and right to inspect an object or patient area,
  - ``WITHDRAW_SEQ``: return to the alert posture after inspection,
  - ``VERDICT_SEQ``: final acceptance or refusal motion depending on ``VERDICT``.
- The global variable ``VERDICT`` selects the final behavior: ``1`` for accept, ``0`` for refuse.
- Each sequence is converted from Cartesian poses to joint waypoints via the custom IK solver.
- Separate Ruckig trajectories are generated for lean, inspect, withdraw, and final verdict segments, then executed in order.

**Example usage:**

```bash
python Use_Case1_Rehabilitation.py
```

To switch the final outcome:
- edit ``VERDICT = 1`` for the accept behavior,
- edit ``VERDICT = 0`` for the refuse behavior.

**What happens:**
- the robot leans in for inspection,
- performs a left/right inspect sweep,
- withdraws to the alert posture,
- then executes either an accept or refuse response.

### `wake_up.py`

- Purpose: perform a wake-up / get-up transition from a docked start pose to an alert pose.
- Motion sequence: ``DOCKED`` → ``WAKEUP`` → ``ARC_MID`` → ``ALERT``.
- The script uses a three-segment Ruckig trajectory with velocity blending so the robot moves smoothly through the intermediate ``WAKEUP`` and ``ARC_MID`` poses.
- This script emphasizes controlled motion rather than stopping fully at every waypoint, making it suitable for a gentle wake-up sequence.

**Example usage:**

```bash
python wake_up.py
```

**What happens:**
- the robot moves to a docked start configuration,
- executes the wake-up transition over a blended trajectory,
- ends holding the alert posture.

## 📁 File Summary

- `motion_shapes_design.py`: main script that generates shapes, solves IK, and executes a Ruckig trajectory on the robot.
- `nod_yes.py`: executes a ``yes`` nod gesture using Cartesian IK and a multi-waypoint Ruckig servo trajectory.
- `refuse_no.py`: executes a ``no`` head-shake gesture using Cartesian pose waypoints and Ruckig control.
- `Use_Case1_Rehabilitation.py`: medical inspection behavior with lean/inspect/withdraw sequences and configurable accept/refuse final action.
- `wake_up.py`: wake-up/get-up transition sequence with blended Ruckig motion through intermediate poses.
- `README.md`: documentation for script usage, configuration, and robot kinematics.

## 💡 Recommended Improvements

If you extend this project, consider:
- adding a configuration file for robot IP, speeds, and limits,
- adding collision checking before execution,
- supporting variable orientation by rotating the shape plane,
- adding a visualizer for Cartesian and joint paths.
