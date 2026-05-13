#!/usr/bin/env python3
import math
import signal
import time
from pathlib import Path

import numpy as np
import pybullet as p
import pybullet_data

from com_estimation_pkg.positions import HOME_POSE, ALL_POSITIONS

ASSETS_PATH = Path("/root/ros2/ws/src/assets/models")

GRIPPER_OPEN        = 0.0
GRIPPER_BOTTLE_GRIP = 0.75  

_BOTTLE_HALF_H = 0.10                                      
_BOTTLE_POS    = [0.50, 0.00, 0.20]                       
_CAP_Z         = _BOTTLE_POS[2] + _BOTTLE_HALF_H         
_GRASP_Z       = _CAP_Z - 0.05                         


_attachment = None   


def grasp_bottle(robot_id: int, eef_idx: int, bottle_id: int):
    global _attachment
    p.changeDynamics(bottle_id, -1, linearDamping=0, angularDamping=0)

    ls       = p.getLinkState(robot_id, eef_idx, computeForwardKinematics=True)
    eef_pos  = np.array(ls[0])
    eef_orn  = np.array(ls[1])

    bot_pos, bot_orn = p.getBasePositionAndOrientation(bottle_id)
    bot_pos  = np.array(bot_pos)
    bot_orn  = np.array(bot_orn)

    R        = np.array(p.getMatrixFromQuaternion(eef_orn.tolist())).reshape(3, 3)
    local_offset = R.T @ (bot_pos - eef_pos)        

    _, inv_orn = p.invertTransform(eef_pos.tolist(), eef_orn.tolist())
    _, rel_orn = p.multiplyTransforms([0, 0, 0], inv_orn,
                                      [0, 0, 0], bot_orn.tolist())

    _attachment = (robot_id, eef_idx, bottle_id, local_offset, np.array(rel_orn))


def release_bottle():
    global _attachment
    _attachment = None


def sync_bottle():
    if _attachment is None:
        return
    robot_id, eef_idx, bottle_id, local_offset, rel_orn = _attachment
    ls      = p.getLinkState(robot_id, eef_idx, computeForwardKinematics=True)
    eef_pos = np.array(ls[0])
    eef_orn = np.array(ls[1])
    R       = np.array(p.getMatrixFromQuaternion(eef_orn.tolist())).reshape(3, 3)
    new_pos = (eef_pos + R @ local_offset).tolist()
    _, new_orn = p.multiplyTransforms([0, 0, 0], eef_orn.tolist(),
                                      [0, 0, 0], rel_orn.tolist())
    p.resetBasePositionAndOrientation(bottle_id, new_pos, list(new_orn))


def load_robot(assets: Path) -> int:
    urdf = assets / "xarm7_with_gripper.urdf"
    if not urdf.exists():
        urdf = assets / "xarm7.urdf"
    return p.loadURDF(str(urdf), basePosition=[0, 0, 0], useFixedBase=True)


def arm_joint_indices(robot_id: int) -> list:
    indices = []
    for i in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, i)
        name, jtype = info[1].decode(), info[2]
        if jtype == p.JOINT_REVOLUTE and name.startswith("joint") and name[5:].isdigit():
            num = int(name[5:])
            if 1 <= num <= 7:
                indices.append((num, i))
    indices.sort()
    return [idx for _, idx in indices]


def gripper_drive_index(robot_id: int) -> int:
    for i in range(p.getNumJoints(robot_id)):
        if p.getJointInfo(robot_id, i)[1].decode() == "drive_joint":
            return i
    return -1


def eef_index(robot_id: int) -> int:
    for i in range(p.getNumJoints(robot_id)):
        if p.getJointInfo(robot_id, i)[12].decode() == "link_tcp":
            return i
    for i in range(p.getNumJoints(robot_id)):
        if p.getJointInfo(robot_id, i)[12].decode() == "link_eef":
            return i
    return p.getNumJoints(robot_id) - 1


def set_arm(robot_id, arm_idx, q):
    for ji, angle in zip(arm_idx, q):
        p.resetJointState(robot_id, ji, angle)


def set_gripper(robot_id, drive_idx, value):
    if drive_idx < 0:
        return
    mimic = {
        "left_finger_joint":         1.0,
        "left_inner_knuckle_joint":  1.0,
        "right_outer_knuckle_joint": 1.0,
        "right_finger_joint":        1.0,
        "right_inner_knuckle_joint": 1.0,
    }
    p.resetJointState(robot_id, drive_idx, value)
    for i in range(p.getNumJoints(robot_id)):
        name = p.getJointInfo(robot_id, i)[1].decode()
        if name in mimic:
            p.resetJointState(robot_id, i, value * mimic[name])


def smooth_move(robot_id, arm_idx, target, steps=120, dt=0.018, on_step=None):
    current = [p.getJointState(robot_id, ji)[0] for ji in arm_idx]
    for s in range(1, steps + 1):
        alpha = 0.5 - 0.5 * math.cos(math.pi * s / steps)
        interp = [c + alpha * (t - c) for c, t in zip(current, target)]
        set_arm(robot_id, arm_idx, interp)
        sync_bottle()               
        p.stepSimulation()
        if on_step:
            on_step(alpha)
        time.sleep(dt)


def animate_gripper(robot_id, drive_idx, from_val, to_val, steps=45, dt=0.018):
    for s in range(steps + 1):
        alpha = 0.5 - 0.5 * math.cos(math.pi * s / steps)
        set_gripper(robot_id, drive_idx, from_val + alpha * (to_val - from_val))
        sync_bottle()              
        p.stepSimulation()
        time.sleep(dt)


def _ik_solve(robot_id, eef_idx, arm_idx, target_pos, target_orn):
    q = p.calculateInverseKinematics(
        robot_id, eef_idx,
        targetPosition=target_pos,
        targetOrientation=target_orn,
        maxNumIterations=500,
        residualThreshold=0.0005,
    )
    q_arm = list(q[:len(arm_idx)])
    for ji, angle in zip(arm_idx, q_arm):
        p.resetJointState(robot_id, ji, angle)
    ls  = p.getLinkState(robot_id, eef_idx, computeForwardKinematics=True)
    err = math.sqrt(sum((a - b) ** 2 for a, b in zip(ls[0], target_pos)))
    return q_arm, err


def find_overhead_grasp(robot_id, eef_idx, arm_idx):
    bx, by = _BOTTLE_POS[0], _BOTTLE_POS[1]
    saved  = [p.getJointState(robot_id, ji)[0] for ji in arm_idx]

    euler_candidates = [
        [ math.pi,      0,          0       ],
        [ 0,            math.pi,    0       ],
        [-math.pi / 2,  0,          0       ],
        [ math.pi / 2,  0,  math.pi         ],
        [ math.pi,      0,  math.pi         ],
        [ 0,           -math.pi,    0       ],
    ]

    result_pre = result_grasp = None
    for euler in euler_candidates:
        orn = p.getQuaternionFromEuler(euler)
        q_grasp, e_grasp = _ik_solve(robot_id, eef_idx, arm_idx,
                                      [bx, by, _GRASP_Z], orn)
        if e_grasp > 0.025:
            continue
        q_pre, e_pre = _ik_solve(robot_id, eef_idx, arm_idx,
                                  [bx, by, _GRASP_Z + 0.22], orn)
        if e_pre > 0.025:
            continue
        result_pre, result_grasp = q_pre, q_grasp
        print(f"[demo] IK OK  euler={[f'{e:.2f}' for e in euler]}"
              f"  err_grasp={e_grasp:.4f}  err_pre={e_pre:.4f}")
        break

    for ji, angle in zip(arm_idx, saved):
        p.resetJointState(robot_id, ji, angle)

    if result_pre is None:
        raise RuntimeError("IK failed for all candidate orientations")
    return result_pre, result_grasp


def draw_world_frame(length=0.4, width=3):
    p.addUserDebugLine([0,0,0], [length,0,0], [1,0,0], width)
    p.addUserDebugLine([0,0,0], [0,length,0], [0,1,0], width)
    p.addUserDebugLine([0,0,0], [0,0,length], [0,0,1], width)
    p.addUserDebugText("X", [length+0.03, 0,        0       ], [1,0,0], 1.4)
    p.addUserDebugText("Y", [0,        length+0.03, 0       ], [0,1,0], 1.4)
    p.addUserDebugText("Z", [0,        0,        length+0.03], [0,0,1], 1.4)


_clr_line_id  = None
_clr_text_id  = None

def show_clearance(bottle_id, plane_id):
    global _clr_line_id, _clr_text_id

    if _clr_line_id is not None:
        p.removeUserDebugItem(_clr_line_id); _clr_line_id = None
    if _clr_text_id is not None:
        p.removeUserDebugItem(_clr_text_id); _clr_text_id = None

    contacts = p.getContactPoints(bottle_id, plane_id)
    in_contact = bool(contacts)

    bot_pos, bot_orn = p.getBasePositionAndOrientation(bottle_id)
    R     = np.array(p.getMatrixFromQuaternion(bot_orn)).reshape(3, 3)
    u     = R[:, 2]                    
    com_z = bot_pos[2]
    lowest_z = com_z - _BOTTLE_HALF_H * abs(u[2]) \
                     - 0.035 * math.sqrt(max(0.0, 1.0 - u[2]**2))

    clearance_m = max(0.0, lowest_z)

    if in_contact or clearance_m < 0.05:
        color = [1.0, 0.2, 0.2]        # red
        label = f"CONTACT  {clearance_m*100:.1f} cm"
    elif clearance_m < 0.10:
        color = [1.0, 0.75, 0.0]       # yellow
        label = f"LOW  {clearance_m*100:.1f} cm"
    else:
        color = [0.2, 1.0, 0.3]        # green
        label = f"OK  {clearance_m*100:.1f} cm"

    bx, by = bot_pos[0], bot_pos[1]
    _clr_line_id = p.addUserDebugLine(
        [bx, by, lowest_z], [bx, by, 0.0],
        color, lineWidth=3, lifeTime=0,
    )
    _clr_text_id = p.addUserDebugText(
        label, [bx + 0.06, by, lowest_z + 0.02],
        color, textSize=1.1, lifeTime=0,
    )

def clear_clearance():
    global _clr_line_id, _clr_text_id
    if _clr_line_id  is not None: p.removeUserDebugItem(_clr_line_id);  _clr_line_id  = None
    if _clr_text_id  is not None: p.removeUserDebugItem(_clr_text_id);  _clr_text_id  = None


def load_platform_and_bottle(assets: Path) -> tuple:
    bottom_z = _BOTTLE_POS[2] - _BOTTLE_HALF_H   
    half_h   = bottom_z / 2.0                    

    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.08, 0.08, half_h])
    vis = p.createVisualShape(p.GEOM_BOX,    halfExtents=[0.08, 0.08, half_h],
                              rgbaColor=[0.55, 0.35, 0.15, 1.0])
    platform_id = p.createMultiBody(
        baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
        basePosition=[_BOTTLE_POS[0], _BOTTLE_POS[1], half_h],
    )
    bottle_id = p.loadURDF(
        str(assets / "bottle_mybottle.urdf"),
        basePosition=_BOTTLE_POS,
        baseOrientation=[0, 0, 0, 1],
    )
    p.changeDynamics(bottle_id, -1, linearDamping=100, angularDamping=100)
    return platform_id, bottle_id


_text_id = None

def show(msg: str, color=(1.0, 1.0, 0.0)):
    global _text_id
    if _text_id is not None:
        p.removeUserDebugItem(_text_id)
    _text_id = p.addUserDebugText(msg, [0.0, 0.0, 1.35],
                                   textColorRGB=list(color),
                                   textSize=1.5, lifeTime=0)
    print(f"[demo] {msg}")


def main():
    p.connect(p.GUI, options="--opengl2 --width=1024 --height=768")
    p.resetDebugVisualizerCamera(
        cameraDistance=1.5, cameraYaw=45, cameraPitch=-30,
        cameraTargetPosition=[0, 0, 0.5],
    )
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    plane_id = p.loadURDF("plane.urdf")
    p.setAdditionalSearchPath(str(ASSETS_PATH))

    robot     = load_robot(ASSETS_PATH)
    arm_idx   = arm_joint_indices(robot)
    drive_idx = gripper_drive_index(robot)
    eef_idx   = eef_index(robot)

    set_arm(robot, arm_idx, HOME_POSE)
    set_gripper(robot, drive_idx, GRIPPER_OPEN)
    p.stepSimulation()
    draw_world_frame()

    show("Computing overhead grasp IK...")
    try:
        pre_q, grasp_q = find_overhead_grasp(robot, eef_idx, arm_idx)
    except RuntimeError as e:
        show(f"IK failed: {e}", color=(1, 0, 0))
        time.sleep(4)
        p.disconnect()
        return

    platform_id = bottle_id = None

    def move(target, steps=140, dt=0.018, gripper_val=GRIPPER_OPEN):
        smooth_move(robot, arm_idx, target, steps=steps, dt=dt,
                    on_step=lambda _: set_gripper(robot, drive_idx, gripper_val))

    show("PHASE 1: Empty arm — no object (UPP baseline)")
    time.sleep(2.0)

    move(pre_q,   steps=200, dt=0.018)
    show("Phase 1: Descending to grasp height (empty)")
    move(grasp_q, steps=120, dt=0.016)
    time.sleep(0.5)
    show("Phase 1: Closing gripper (empty) — matches Phase 2 state")
    animate_gripper(robot, drive_idx, GRIPPER_OPEN, GRIPPER_BOTTLE_GRIP)
    time.sleep(0.4)
    move(pre_q, steps=100, dt=0.016, gripper_val=GRIPPER_BOTTLE_GRIP)

    show(f"PHASE 1: {len(ALL_POSITIONS)} measurement positions (HOME → each pose)")
    for name, q in ALL_POSITIONS.items():
        show(f"Phase 1 — {name}", color=(0.8, 0.8, 0.3))
        move(HOME_POSE, steps=100, dt=0.016, gripper_val=GRIPPER_BOTTLE_GRIP)
        move(q,         steps=120, dt=0.016, gripper_val=GRIPPER_BOTTLE_GRIP)
        time.sleep(0.4)

    show("PHASE 2: Returning home — placing bottle on platform")
    move(HOME_POSE, steps=160, dt=0.018)

    platform_id, bottle_id = load_platform_and_bottle(ASSETS_PATH)
    p.stepSimulation()
    time.sleep(1.5)

    show("PHASE 2: Moving above bottle")
    move(pre_q, steps=200, dt=0.018)
    time.sleep(0.5)

    show("Descending onto bottle...")
    move(grasp_q, steps=120, dt=0.016)
    time.sleep(0.4)

    grasp_bottle(robot, eef_idx, bottle_id)
    show("Closing gripper...")
    animate_gripper(robot, drive_idx, GRIPPER_OPEN, GRIPPER_BOTTLE_GRIP)

    show("Lifting bottle off platform...", color=(0.2, 1.0, 0.3))
    move(pre_q, steps=120, dt=0.016, gripper_val=GRIPPER_BOTTLE_GRIP)
    time.sleep(0.5)

    show(f"PHASE 2: {len(ALL_POSITIONS)} positions WITH bottle (HOME → each pose)")
    for name, q in ALL_POSITIONS.items():
        show(f"Phase 2 — {name}", color=(0.3, 1.0, 0.5))
        clear_clearance()
        move(HOME_POSE, steps=100, dt=0.016, gripper_val=GRIPPER_BOTTLE_GRIP)
        move(q,         steps=120, dt=0.016, gripper_val=GRIPPER_BOTTLE_GRIP)
        show_clearance(bottle_id, plane_id)
        time.sleep(0.8)

    move(HOME_POSE, steps=160, dt=0.018, gripper_val=GRIPPER_BOTTLE_GRIP)
    clear_clearance()

    show("PHASE 3: Returning bottle to platform", color=(0.8, 0.5, 1.0))
    time.sleep(1.0)

    move(pre_q,   steps=180, dt=0.018, gripper_val=GRIPPER_BOTTLE_GRIP)
    show("Descending to place position...", color=(0.8, 0.5, 1.0))
    move(grasp_q, steps=120, dt=0.016, gripper_val=GRIPPER_BOTTLE_GRIP)
    time.sleep(0.4)

    release_bottle()
    show("Opening gripper — releasing bottle", color=(0.8, 0.5, 1.0))
    animate_gripper(robot, drive_idx, GRIPPER_BOTTLE_GRIP, GRIPPER_OPEN)
    time.sleep(0.5)

    show("Retracting arm...", color=(0.8, 0.5, 1.0))
    move(pre_q,     steps=120, dt=0.016)
    move(HOME_POSE, steps=160, dt=0.018)

    show("Demo complete!  Ctrl+C to exit.", color=(0.6, 0.9, 1.0))

    p.setRealTimeSimulation(1)
    stop = [False]
    signal.signal(signal.SIGINT, lambda s, f: stop.__setitem__(0, True))
    while not stop[0]:
        time.sleep(0.05)
    if bottle_id   is not None: p.removeBody(bottle_id)
    if platform_id is not None: p.removeBody(platform_id)
    p.disconnect()
    print("Demo ended.")


if __name__ == "__main__":
    main()
