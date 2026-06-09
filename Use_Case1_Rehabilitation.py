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
# VERDICT
# =========================================================================
VERDICT = 1  # <<<< 0 = REFUSE, 1 = ACCEPT

# =========================================================================
# WAYPOINTS: Toggle per pose
# Comment the CARTESIAN line, uncomment the JOINT line to switch to joint space
# =========================================================================

# 0. CASE1_ALERT (social ready pose)
CASE1_ALERT = [111.783, -391.593, 621.384, -176.649, 44.909, 90.989]      # Cartesian
# CASE1_ALERT = [87.5547, -110.0, 100.0, -125.0, -90.9235, 87.5680]     # Joint

# 1. MEDICINE_CHECK (leaned-in inspection pose)
MEDICINE_CHECK = [103.027, -596.621, 590.626, -178.851, 27.336, 88.046]   # Cartesian
# MEDICINE_CHECK = [87.5547, -81.2268, 72.4075, -108.5249, -90.9235, 89.5638]  # Joint

# 2. INSPECT_LEFT (J5 left + J1 counter-right)
INSPECT_LEFT = [123.617, -587.387, 592.864, -166.857, 27.246, 92.033]    # Cartesian
# INSPECT_LEFT = [91.5547, -81.2268, 72.4075, -108.5249, -102.9235, 89.5638]   # Joint

# 3. INSPECT_RIGHT (J5 right + J1 counter-left)
INSPECT_RIGHT = [81.945, -603.848, 592.269, 169.145, 27.427, 84.037]     # Cartesian
# INSPECT_RIGHT = [83.5547, -81.2268, 72.4075, -108.5249, -78.9235, 89.5638]   # Joint

# 4. ACCEPT_DOWN (J4 down + J2 fwd from CASE1_ALERT)
ACCEPT_DOWN = [112.309, -379.259, 693.440, -167.336, 78.211, 99.540]     # Cartesian
# ACCEPT_DOWN = [87.5547, -108.5, 100.0, -160.0, -90.9235, 87.5680]      # Joint

# 5. REFUSE_LEFT (J5 left + J1 left from CASE1_ALERT)
REFUSE_LEFT = [51.972, -393.818, 624.088, -161.780, 44.286, 85.822]      # Cartesian
# REFUSE_LEFT = [82.5547, -110.0, 100.0, -125.0, -105.9235, 87.5680]     # Joint

# 6. REFUSE_RIGHT (J5 right + J1 right from CASE1_ALERT)
REFUSE_RIGHT = [171.300, -377.092, 623.498, 168.317, 45.542, 95.924]    # Cartesian
# REFUSE_RIGHT = [92.5547, -110.0, 100.0, -125.0, -75.9235, 87.5680]     # Joint

# Build behavior sequences
LEAN_SEQ = [CASE1_ALERT, MEDICINE_CHECK]

INSPECT_SEQ = [
    MEDICINE_CHECK,
    INSPECT_LEFT,
    MEDICINE_CHECK,
    INSPECT_RIGHT,
    MEDICINE_CHECK
]

WITHDRAW_SEQ = [MEDICINE_CHECK, CASE1_ALERT]

if VERDICT == 1:
    VERDICT_SEQ = [
        CASE1_ALERT,
        ACCEPT_DOWN,
        CASE1_ALERT,
        ACCEPT_DOWN,
        CASE1_ALERT
    ]
else:
    VERDICT_SEQ = [
        CASE1_ALERT,
        REFUSE_LEFT,
        CASE1_ALERT,
        REFUSE_RIGHT,
        CASE1_ALERT,
        REFUSE_LEFT,
        CASE1_ALERT
    ]

# Seed for IK (original CASE1_ALERT joint pose — keeps solutions close to intended posture)
SEED = [87.5547, -110.0, 100.0, -125.0, -90.9235, 87.5680]

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
# RUCKIG MULTI-WAYPOINT TRAJECTORY
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

    verdict_str = "ACCEPT" if VERDICT == 1 else "REFUSE"
    print(f"\n=== CASE 1: MEDICAL INSPECTION ===")
    print(f"Verdict configured: {verdict_str}")

    err, curr = robot.GetActualJointPosDegree()
    if err == 0:
        print(f"Current: {[round(j, 2) for j in curr[:6]]}")

    # -----------------------------------------------------------------
    # TRAJECTORY PREPARATION
    # -----------------------------------------------------------------
    # Option A: CARTESIAN mode (runs IK)
    # Comment out this block if you switched the WAYPOINTS to joint angles
    print("\n[MODE] Cartesian → IK")
    print("\n[Lean sequence]")
    lean_joints = solve_ik_sequence(LEAN_SEQ, SEED)
    print("\n[Inspect sequence]")
    inspect_joints = solve_ik_sequence(INSPECT_SEQ, SEED)
    print("\n[Withdraw sequence]")
    withdraw_joints = solve_ik_sequence(WITHDRAW_SEQ, SEED)
    print(f"\n[{verdict_str} sequence]")
    verdict_joints = solve_ik_sequence(VERDICT_SEQ, SEED)

    # Option B: JOINT-SPACE direct (bypass IK)
    # Uncomment the four lines below if you switched the WAYPOINTS to joint angles
    # lean_joints = [np.array(wp) for wp in LEAN_SEQ]
    # inspect_joints = [np.array(wp) for wp in INSPECT_SEQ]
    # withdraw_joints = [np.array(wp) for wp in WITHDRAW_SEQ]
    # verdict_joints = [np.array(wp) for wp in VERDICT_SEQ]

    # -----------------------------------------------------------------
    # RUCKIG TRAJECTORIES
    # -----------------------------------------------------------------
    lean_traj = generate_joint_trajectory(lean_joints, v_max=60, a_max=120, j_max=600)
    inspect_traj = generate_joint_trajectory(inspect_joints, v_max=80, a_max=200, j_max=800)
    withdraw_traj = generate_joint_trajectory(withdraw_joints, v_max=70, a_max=140, j_max=700)

    if VERDICT == 1:
        verdict_traj = generate_joint_trajectory(verdict_joints, v_max=80, a_max=200, j_max=800)
    else:
        verdict_traj = generate_joint_trajectory(verdict_joints, v_max=100, a_max=300, j_max=1200)

    print(f"\nTrajectories ready:")
    print(f"  Lean:    {len(lean_traj)} pts ({len(lean_traj)*dt:.3f}s)")
    print(f"  Inspect: {len(inspect_traj)} pts ({len(inspect_traj)*dt:.3f}s)")
    print(f"  Withdraw:{len(withdraw_traj)} pts ({len(withdraw_traj)*dt:.3f}s)")
    print(f"  {verdict_str}: {len(verdict_traj)} pts ({len(verdict_traj)*dt:.3f}s)")

    # -----------------------------------------------------------------
    # EXECUTE
    # -----------------------------------------------------------------
    print("\n[1/5] Moving to CASE1_ALERT (MoveJ)...")
    robot.MoveJ(lean_joints[0].tolist(), tool=0, user=0, vel=30)
    time.sleep(0.5)

    print("[2/5] Leaning in to inspect...")
    execute_servo(robot, lean_traj)
    time.sleep(0.1)

    print("[3/5] Inspecting left to right...")
    execute_servo(robot, inspect_traj)
    time.sleep(0.1)

    print("[4/5] Withdrawing to alert pose...")
    execute_servo(robot, withdraw_traj)
    time.sleep(0.2)

    print(f"[5/5] Executing {verdict_str}...")
    execute_servo(robot, verdict_traj)

    print(f"\nDone. Verdict: {verdict_str}")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"Error: {e}")