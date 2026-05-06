from fairino import Robot
import math
import time
import numpy as np
from ruckig import Ruckig, InputParameter, OutputParameter, Result, Synchronization

# -------------------------------------------------------------------
# CONFIG
# -------------------------------------------------------------------
ROBOT_IP = "192.168.58.4"
dt = 0.008  # 125 Hz control cycle

# Fixed path resolution — not user-configurable
DEFAULT_POINTS = 40

# Ruckig limits (rad/s, rad/s^2, rad/s^3)
V_MAX = np.deg2rad(np.array([100, 100, 120, 150, 150, 180]))
A_MAX = np.deg2rad(np.array([250, 250, 300, 400, 400, 400]))
J_MAX = np.deg2rad(np.array([1500, 1500, 2000, 3000, 3000, 3000]))

# Intermediate waypoint blend velocity (deg/s in joint space).
# 0.0 = full stop at every waypoint.
# 15-30 = flows through waypoints for smoother drawing.
BLEND_VEL_DEG = 20.0

SHAPES = [
    "circle", "ellipse", "square", "rectangle", "triangle",
    "arc", "s_curve", "wave",
    "star", "moon", "heart", "spiral", "infinity",
    "pentagon", "hexagon", "diamond", "cross", "arrow"
]


# -------------------------------------------------------------------
# UTILS
# -------------------------------------------------------------------
def rounded_pose(pose):
    return [round(float(v), 3) for v in pose[:6]]

def rounded_joints(q):
    return [round(float(np.rad2deg(v)), 3) for v in q[:6]]

def is_valid_pose(pose):
    if not isinstance(pose, (list, tuple)) or len(pose) < 6:
        return False
    for v in pose[:6]:
        if not isinstance(v, (int, float)):
            return False
        if math.isnan(v) or math.isinf(v):
            return False
    return True

def ask_float(prompt, default_value):
    raw = input(prompt).strip()
    if raw == "":
        return default_value
    try:
        return float(raw)
    except ValueError:
        return default_value


# -------------------------------------------------------------------
# SHAPE GENERATION (Cartesian path — fixed resolution)
# -------------------------------------------------------------------
def generate_shape(shape, center, size=30.0, size_b=None, points=DEFAULT_POINTS):
    path = []
    cx, cy, cz, rx, ry, rz = center[:6]
    if points < 3:
        points = 3

    if shape == "circle":
        for i in range(points + 1):
            angle = 2 * math.pi * i / points
            x = cx + size * math.cos(angle)
            y = cy + size * math.sin(angle)
            path.append([x, y, cz, rx, ry, rz])

    elif shape == "ellipse":
        a = size
        b = size_b if size_b is not None else size / 2.0
        for i in range(points + 1):
            angle = 2 * math.pi * i / points
            x = cx + a * math.cos(angle)
            y = cy + b * math.sin(angle)
            path.append([x, y, cz, rx, ry, rz])

    elif shape == "square":
        s = size
        corners = [
            [cx - s / 2, cy - s / 2],
            [cx + s / 2, cy - s / 2],
            [cx + s / 2, cy + s / 2],
            [cx - s / 2, cy + s / 2],
            [cx - s / 2, cy - s / 2],
        ]
        for x, y in corners:
            path.append([x, y, cz, rx, ry, rz])

    elif shape == "rectangle":
        w = size
        h = size / 2.0
        corners = [
            [cx - w / 2, cy - h / 2],
            [cx + w / 2, cy - h / 2],
            [cx + w / 2, cy + h / 2],
            [cx - w / 2, cy + h / 2],
            [cx - w / 2, cy - h / 2],
        ]
        for x, y in corners:
            path.append([x, y, cz, rx, ry, rz])

    elif shape == "triangle":
        s = size
        vertices = [
            [cx,         cy - s / 2],
            [cx + s / 2, cy + s / 2],
            [cx - s / 2, cy + s / 2],
            [cx,         cy - s / 2],
        ]
        for x, y in vertices:
            path.append([x, y, cz, rx, ry, rz])

    elif shape == "arc":
        start_angle = 0.0
        end_angle = math.pi
        for i in range(points + 1):
            angle = start_angle + (end_angle - start_angle) * i / points
            x = cx + size * math.cos(angle)
            y = cy + size * math.sin(angle)
            path.append([x, y, cz, rx, ry, rz])

    elif shape == "s_curve":
        for i in range(points + 1):
            t = i / points
            x = cx - size + 2 * size * t
            y = cy + (size / 2.0) * math.sin(2 * math.pi * t)
            path.append([x, y, cz, rx, ry, rz])

    elif shape == "wave":
        waves = 3
        amplitude = size / 3.0
        for i in range(points + 1):
            t = i / points
            x = cx - size + 2 * size * t
            y = cy + amplitude * math.sin(waves * 2 * math.pi * t)
            path.append([x, y, cz, rx, ry, rz])

    elif shape == "star":
        outer = size
        inner = size * 0.4
        spikes = 5
        for i in range(spikes * 2 + 1):
            angle = math.pi / spikes * i - math.pi / 2
            r = outer if i % 2 == 0 else inner
            x = cx + r * math.cos(angle)
            y = cy + r * math.sin(angle)
            path.append([x, y, cz, rx, ry, rz])

    elif shape == "moon":
        offset = size * 0.35
        for i in range(points + 1):
            angle = 2 * math.pi * i / points
            x = cx + size * math.cos(angle)
            y = cy + size * math.sin(angle)
            path.append([x, y, cz, rx, ry, rz])
        for i in range(points + 1):
            angle = 2 * math.pi * (points - i) / points
            x = cx + offset + size * math.cos(angle)
            y = cy + size * math.sin(angle)
            path.append([x, y, cz, rx, ry, rz])

    elif shape == "heart":
        scale = size / 17.0
        for i in range(points + 1):
            t = 2 * math.pi * i / points
            x = cx + scale * 16 * (math.sin(t) ** 3)
            y = cy - scale * (
                13 * math.cos(t)
                - 5 * math.cos(2 * t)
                - 2 * math.cos(3 * t)
                - math.cos(4 * t)
            )
            path.append([x, y, cz, rx, ry, rz])

    elif shape == "spiral":
        turns = 3
        for i in range(points + 1):
            t = turns * 2 * math.pi * i / points
            r = size * i / points
            x = cx + r * math.cos(t)
            y = cy + r * math.sin(t)
            path.append([x, y, cz, rx, ry, rz])

    elif shape == "infinity":
        a = size
        for i in range(points + 1):
            t = 2 * math.pi * i / points
            denom = 1 + math.sin(t) ** 2
            x = cx + a * math.cos(t) / denom
            y = cy + a * math.sin(t) * math.cos(t) / denom
            path.append([x, y, cz, rx, ry, rz])

    elif shape == "pentagon":
        sides = 5
        for i in range(sides + 1):
            angle = 2 * math.pi * i / sides - math.pi / 2
            x = cx + size * math.cos(angle)
            y = cy + size * math.sin(angle)
            path.append([x, y, cz, rx, ry, rz])

    elif shape == "hexagon":
        sides = 6
        for i in range(sides + 1):
            angle = 2 * math.pi * i / sides
            x = cx + size * math.cos(angle)
            y = cy + size * math.sin(angle)
            path.append([x, y, cz, rx, ry, rz])

    elif shape == "diamond":
        s = size
        vertices = [
            [cx,       cy - s],
            [cx + s/2, cy],
            [cx,       cy + s],
            [cx - s/2, cy],
            [cx,       cy - s],
        ]
        for x, y in vertices:
            path.append([x, y, cz, rx, ry, rz])

    elif shape == "cross":
        t = size / 3.0
        s = size
        vertices = [
            [cx - t, cy - s], [cx + t, cy - s],
            [cx + t, cy - t], [cx + s, cy - t],
            [cx + s, cy + t], [cx + t, cy + t],
            [cx + t, cy + s], [cx - t, cy + s],
            [cx - t, cy + t], [cx - s, cy + t],
            [cx - s, cy - t], [cx - t, cy - t],
            [cx - t, cy - s],
        ]
        for x, y in vertices:
            path.append([x, y, cz, rx, ry, rz])

    elif shape == "arrow":
        s = size
        vertices = [
            [cx,           cy],
            [cx,           cy + s * 0.3],
            [cx + s * 0.5, cy + s * 0.3],
            [cx + s * 0.5, cy + s * 0.6],
            [cx + s,       cy],
            [cx + s * 0.5, cy - s * 0.6],
            [cx + s * 0.5, cy - s * 0.3],
            [cx,           cy - s * 0.3],
            [cx,           cy],
        ]
        for x, y in vertices:
            path.append([x, y, cz, rx, ry, rz])

    else:
        raise ValueError(f"Unknown shape: {shape}")

    for p in path:
        if not is_valid_pose(p):
            raise ValueError(f"Generated invalid pose: {p}")

    return path


# -------------------------------------------------------------------
# IK + RUCKIG PIPELINE
# -------------------------------------------------------------------
def unwrap_joints(q_prev, q_curr):
    """Unwrap q_curr to minimize per-joint angular distance from q_prev."""
    diff = q_curr - q_prev
    offset = 2.0 * np.pi * np.round(diff / (2.0 * np.pi))
    return q_curr - offset

def cartesian_path_to_joint_path(robot, cartesian_path):
    """
    Convert Cartesian path to a continuous joint-space path.
    Uses GetInverseKin(type=0, desc_pos=pose, config=-1).
    """
    joint_path = []
    q_prev = None

    for i, pose in enumerate(cartesian_path):
        err, q_sol = robot.GetInverseKin(type=0, desc_pos=pose, config=-1)
        if err != 0:
            raise RuntimeError(
                f"IK failed at point {i}: pose={rounded_pose(pose)}, error={err}"
            )

        q = np.deg2rad(np.array(q_sol[:6], dtype=float))

        if q_prev is not None:
            q = unwrap_joints(q_prev, q)

        joint_path.append(q)
        q_prev = q

    return joint_path

def generate_ruckig_trajectory(joint_path, dt=0.008, blend_vel_deg=0.0):
    """Generate a smooth, jerk-limited 6-DOF trajectory through joint waypoints."""
    if len(joint_path) < 2:
        return np.array(joint_path)

    otg = Ruckig(6, dt)
    inp = InputParameter(6)
    out = OutputParameter(6)

    inp.max_velocity = V_MAX.tolist()
    inp.max_acceleration = A_MAX.tolist()
    inp.max_jerk = J_MAX.tolist()
    inp.synchronization = Synchronization.Time

    full_traj = []
    blend_vel = np.deg2rad(blend_vel_deg)

    for i in range(len(joint_path) - 1):
        inp.current_position = joint_path[i].tolist()
        inp.target_position = joint_path[i + 1].tolist()

        # Target velocity: stop at final waypoint, blend through intermediates
        if i == len(joint_path) - 2:
            inp.target_velocity = [0.0] * 6
        else:
            if blend_vel > 0:
                direction = joint_path[i + 1] - joint_path[i]
                norm = np.linalg.norm(direction)
                if norm > 1e-6:
                    v_blend = (blend_vel / norm) * direction
                else:
                    v_blend = np.zeros(6)
                inp.target_velocity = v_blend.tolist()
            else:
                inp.target_velocity = [0.0] * 6

        # State inheritance
        if i == 0:
            inp.current_velocity = [0.0] * 6
            inp.current_acceleration = [0.0] * 6
        else:
            inp.current_velocity = out.new_velocity
            inp.current_acceleration = out.new_acceleration

        # Generate segment
        res = Result.Working
        while res == Result.Working:
            res = otg.update(inp, out)
            full_traj.append(np.array(out.new_position))
            out.pass_to_input(inp)

    return np.array(full_traj)

def execute_servo_trajectory(robot, trajectory, dt=0.008):
    """Stream trajectory to robot via ServoJ with hard real-time timing."""
    if len(trajectory) == 0:
        print("Empty trajectory.")
        return False

    print(f"Streaming {len(trajectory)} samples at {1/dt:.0f} Hz...")
    robot.ServoMoveStart()
    start_time = time.perf_counter()

    try:
        for idx, q_rad in enumerate(trajectory):
            q_deg = np.rad2deg(q_rad).tolist()
            robot.ServoJ(q_deg, [0.0] * 4, 0.0, 0.0, dt, 0.0, 0.0, 0)

            expected = start_time + (idx + 1) * dt
            while time.perf_counter() < expected:
                pass

        robot.ServoMoveEnd()
        print("Execution complete.")
        return True

    except Exception as e:
        robot.ServoMoveEnd()
        print(f"Execution interrupted: {e}")
        return False


# -------------------------------------------------------------------
# MAIN
# -------------------------------------------------------------------
def main():
    robot = Robot.RPC(ROBOT_IP)
    if robot is None:
        print("RPC connection failed.")
        return

    try:
        robot.SetSpeed(20)

        # Current Cartesian pose (used as shape center / reference)
        ret_pose, start_pose = robot.GetActualTCPPose()
        if ret_pose != 0:
            print(f"Failed to get TCP pose, errcode: {ret_pose}")
            return

        start_pose = list(start_pose)[:6]
        if not is_valid_pose(start_pose):
            print(f"Invalid start pose: {start_pose}")
            return

        # Active tool / user frames
        ret_tool, actual_tool = robot.GetActualTCPNum()
        ret_user, actual_user = robot.GetActualWObjNum()
        if ret_tool != 0 or ret_user != 0:
            print("Failed to get active tool/user frames.")
            return

        tool_id = int(actual_tool)
        user_id = int(actual_user)

        print(f"Start pose : {rounded_pose(start_pose)}")
        print(f"Tool active : {tool_id}")
        print(f"User active : {user_id}")

        # Shape selection
        print("\nAvailable shapes:")
        for s in SHAPES:
            print(f"  • {s}")

        shape = input("\nEnter shape: ").strip().lower()
        if shape not in SHAPES:
            print(f"Unknown shape: '{shape}'")
            return

        # Generate Cartesian path (fixed resolution)
        if shape == "ellipse":
            size_a = ask_float("Semi-major axis a (mm) [default 30]: ", 30.0)
            size_b = ask_float("Semi-minor axis b (mm) [default 15]: ", 15.0)
            path = generate_shape(shape, start_pose, size=size_a, size_b=size_b)
        else:
            size = ask_float("Size in mm [default 20]: ", 20.0)
            path = generate_shape(shape, start_pose, size=size)

        print(f"\nCartesian path points: {len(path)}")
        print(f"First point: {rounded_pose(path[0])}")
        print(f"Last point : {rounded_pose(path[-1])}")

        # Convert to joint space
        print("\nSolving IK...")
        try:
            joint_path = cartesian_path_to_joint_path(robot, path)
        except RuntimeError as e:
            print(f"IK Error: {e}")
            return

        print(f"Joint path: {len(joint_path)} configs")
        print(f"Start joints: {rounded_joints(joint_path[0])}")

        # Pre-position to start via MoveJ (safe blocking move)
        print("\nMoving to start configuration...")
        start_joints_deg = np.rad2deg(joint_path[0]).tolist()
        robot.MoveJ(start_joints_deg, tool=tool_id, user=user_id, vel=30)
        time.sleep(0.5)

        # Generate Ruckig trajectory
        print("Generating Ruckig trajectory...")
        trajectory = generate_ruckig_trajectory(
            joint_path, dt=dt, blend_vel_deg=BLEND_VEL_DEG
        )
        print(f"Trajectory samples: {len(trajectory)}")

        # Execute
        success = execute_servo_trajectory(robot, trajectory, dt=dt)

        # Return to start
        print("\nReturning to start...")
        robot.MoveJ(start_joints_deg, tool=tool_id, user=user_id, vel=30)

        if success:
            print("\nShape completed successfully.")
        else:
            print("\nExecution failed.")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    except Exception as e:
        print(f"\nUnexpected exception: {e}")
    finally:
        try:
            robot.ServoMoveEnd()
        except Exception:
            pass
        try:
            robot.CloseRPC()
        except Exception:
            pass


if __name__ == "__main__":
    main()