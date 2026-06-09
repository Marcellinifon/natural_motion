import time
import numpy as np
from fairino import Robot
from ruckig import Ruckig, InputParameter, OutputParameter, Result, Synchronization
from scipy.optimize import minimize

# =========================================================================
# CONFIG
# =========================================================================
ROBOT_IP = "192.168.58.4"
dt = 0.008

# =========================================================================
# WAYPOINTS: Toggle per pose
# Comment the CARTESIAN line, uncomment the JOINT line to switch to joint space
# =========================================================================

# 0. ALERT (start / end)
ALERT = [-26.072, -205.431, 678.455, -117.920, 87.304, 106.820]       # Cartesian
# ALERT = [42.483, -137.551, 116.818, -158.004, -87.769, 87.5682]     # Joint

# 1. REFUSE_BASE
REFUSE_BASE = [-26.073, -205.433, 678.454, -104.682, 85.013, 120.086]  # Cartesian
# REFUSE_BASE = [42.4829, -137.5508, 116.818, -158.0038, -87.7688, 85.1262]  # Joint

# 2. REFUSE_LEFT (J1 left + J5 left combined)
REFUSE_LEFT = [-57.796, -180.292, 678.507, -103.467, 84.702, 101.300] # Cartesian
# REFUSE_LEFT = [37.4829, -137.5508, 116.818, -158.0038, -102.7688, 85.1262]  # Joint

# 3. REFUSE_RIGHT (J1 right + J5 right combined)
REFUSE_RIGHT = [13.876, -221.060, 678.551, -105.029, 85.342, 139.736] # Cartesian
# REFUSE_RIGHT = [47.4829, -137.5508, 116.818, -158.0038, -72.7688, 85.1262]  # Joint

# Sequence: ALERT → BASE → LEFT → BASE → RIGHT → BASE → LEFT → BASE → ALERT
WAYPOINTS = [
    ALERT,
    REFUSE_BASE,
    REFUSE_LEFT,
    REFUSE_BASE,
    REFUSE_RIGHT,
    REFUSE_BASE,
    REFUSE_LEFT,
    REFUSE_BASE,
    ALERT
]

# Seed for IK (ALERT joint pose)
SEED = [42.483, -137.551, 116.818, -158.004, -87.769, 87.5682]

# =========================================================================
# CUSTOM FK
# =========================================================================
def fairino_f5_fk(joint_angles_deg):
    d1, a2, a3, d4, d5, d6 = 152.0, -425.0, -395.0, 130.0, 102.0, 100.0
    t = np.radians(joint_angles_deg)
    def dh(th, d, a, al):
        c, s = np.cos, np.sin
        return np.array([
            [c(th), -s(th)*c(al),  s(th)*s(al), a*c(th)],
            [s(th),  c(th)*c(al), -c(th)*s(al), a*s(th)],
            [0,      s(al),        c(al),       d],
            [0,      0,            0,           1]
        ])
    T = (dh(t[0], d1, 0, np.pi/2) @ dh(t[1], 0, a2, 0) @ dh(t[2], 0, a3, 0) @
         dh(t[3], d4, 0, np.pi/2) @ dh(t[4], d5, 0, -np.pi/2) @ dh(t[5], d6, 0, 0))
    return np.array([T[0,3], T[1,3], T[2,3]]), T[:3,:3]

def rotmat_to_zyx_euler(R):
    sy = -R[2,0]
    if abs(sy) < 1 - 1e-6:
        ry = np.arcsin(sy); rx = np.arctan2(R[2,1], R[2,2]); rz = np.arctan2(R[1,0], R[0,0])
    else:
        sign_sy = np.sign(sy) if abs(sy) > 1e-6 else 1.0
        ry = sign_sy * np.pi/2; rx = np.arctan2(-R[1,2], R[1,1]); rz = 0.0
    return np.degrees([rx, ry, rz])

# =========================================================================
# IK SOLVER
# =========================================================================
def fairino_f5_ik(target_pose, seed, lambda_eff=0.01):
    target_pos = np.array(target_pose[:3])
    rx, ry, rz = np.radians(target_pose[3:])
    cx, sx, cy, sy, cz, sz = np.cos(rx), np.sin(rx), np.cos(ry), np.sin(ry), np.cos(rz), np.sin(rz)
    target_R = np.array([
        [cy*cz, cz*sx*sy - cx*sz, cx*cz*sy + sx*sz],
        [cy*sz, cx*cz + sx*sy*sz, cx*sy*sz - cz*sx],
        [-sy,   cy*sx,            cx*cy]
    ])
    def cost(joints):
        pos, R = fairino_f5_fk(joints)
        pos_err = np.sum((pos - target_pos)**2)
        ori_err = 3.0 - np.trace(target_R.T @ R)
        return pos_err + 500.0 * ori_err + lambda_eff * np.sum((joints - seed)**2)
    result = minimize(cost, np.array(seed), method='L-BFGS-B',
                      bounds=[(-175,175)]*6, options={'maxiter':1000})
    if result.success:
        sol = [float(j) for j in result.x]
        fpos, fR = fairino_f5_fk(result.x)
        pos_err = np.linalg.norm(fpos - target_pos)
        ori_err = np.degrees(np.arccos(np.clip((np.trace(target_R.T @ fR)-1)/2, -1, 1)))
        return {'joints': [round(j,4) for j in sol], 'pos_err': pos_err, 'ori_err': ori_err}
    return None

def solve_ik_sequence(poses, seed):
    joints = []
    s = np.array(seed)
    for p in poses:
        r = fairino_f5_ik(p, s, lambda_eff=0.01)
        if r is None: raise RuntimeError(f"IK failed for pose {p}")
        j = np.array(r['joints']); joints.append(j); s = j
        print(f"  IK ok | pos_err={r['pos_err']:.3f} mm | ori_err={r['ori_err']:.3f}°")
    return joints

# =========================================================================
# RUCKIG TRAJECTORY GENERATOR
# =========================================================================
def generate_joint_trajectory(joint_waypoints, v_max, a_max, j_max):
    n = len(joint_waypoints)
    full_traj = []
    for i in range(n - 1):
        otg = Ruckig(6, dt)
        inp = InputParameter(6)
        out = OutputParameter(6)
        inp.current_position  = np.deg2rad(joint_waypoints[i]).tolist()
        inp.target_position   = np.deg2rad(joint_waypoints[i+1]).tolist()
        inp.current_velocity  = [0.0]*6
        inp.current_acceleration = [0.0]*6
        inp.target_velocity   = [0.0]*6
        inp.target_acceleration = [0.0]*6
        inp.max_velocity     = [np.deg2rad(v_max)]*6
        inp.max_acceleration = [np.deg2rad(a_max)]*6
        inp.max_jerk         = [np.deg2rad(j_max)]*6
        inp.synchronization  = Synchronization.Phase
        seg = []
        res = Result.Working
        while res == Result.Working:
            res = otg.update(inp, out)
            seg.append(list(out.new_position))
            out.pass_to_input(inp)
        if i == 0:
            full_traj.extend(seg)
        else:
            full_traj.extend(seg[1:])
    return np.array(full_traj)

# =========================================================================
# SERVO EXECUTION
# =========================================================================
def execute_servo(robot, traj_rad):
    robot.ServoMoveStart()
    t0 = time.perf_counter()
    for i, q_rad in enumerate(traj_rad):
        robot.ServoJ(np.rad2deg(q_rad).tolist(), [0.0]*4, 0.0, 0.0, dt, 0.0, 0.0, 0)
        while time.perf_counter() < t0 + (i + 1) * dt:
            pass
    robot.ServoMoveEnd()

# =========================================================================
# MAIN
# =========================================================================
def main():
    robot = Robot.RPC(ROBOT_IP)

    # -----------------------------------------------------------------
    # TRAJECTORY PREPARATION
    # -----------------------------------------------------------------
    # Option A: CARTESIAN mode (runs IK)
    # Comment out this block if you switched the WAYPOINTS to joint angles
    print("[MODE] Cartesian → IK")
    joint_waypoints = solve_ik_sequence(WAYPOINTS, SEED)

    # Option B: JOINT-SPACE direct (bypass IK)
    # Uncomment the line below if you switched the WAYPOINTS to joint angles
    # joint_waypoints = [np.array(wp) for wp in WAYPOINTS]

    # -----------------------------------------------------------------
    # RUCKIG TRAJECTORY
    # -----------------------------------------------------------------
    traj = generate_joint_trajectory(joint_waypoints, v_max=100, a_max=300, j_max=1200)
    print(f"Refuse trajectory: {len(traj)} points ({len(traj)*dt:.3f}s)")

    # -----------------------------------------------------------------
    # EXECUTE
    # -----------------------------------------------------------------
    print("\n[1/2] Move to start (ALERT) via MoveJ...")
    robot.MoveJ(joint_waypoints[0].tolist(), tool=0, user=0, vel=30)
    time.sleep(0.5)

    print("[2/2] Refuse No (ServoJ)...")
    execute_servo(robot, traj)

    print("Complete. Holding ALERT.")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"Error: {e}")