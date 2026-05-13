#!/usr/bin/env python3
import time
import signal
from pathlib import Path

import numpy as np
import pybullet as p

from sim_robot_pkg.pybullet_world import XArm7PyBulletWorld
from com_estimation_pkg.positions import get_positions

_GRIPPER_LINKS = {"link7", "link_eef", ""}


def run_check(gui: bool = True, heavy: bool = False, slow: bool = False):
    bottle_urdf = "bottle_mybottle_heavy.urdf" if heavy else "bottle_mybottle.urdf"
    variant = "bottom-heavy (nails↓ salt↑)" if heavy else "uniform CoM"

    world = XArm7PyBulletWorld(cube_choice=0, gui=gui)

    bottle_id = p.loadURDF(
        str(world.assets_path / bottle_urdf),
        basePosition=[0, 0, -10],
    )
    cube_offset = np.array(
        world.OBJECT_ATTACHMENT_OFFSETS[10 if heavy else 9]
    )

    def _place_and_check(q):
        for i, ji in enumerate(world.joint_indices):
            p.resetJointState(world.robot_id, ji, q[i])
        eef = p.getLinkState(world.robot_id, world.eef_link_index)
        R = np.array(p.getMatrixFromQuaternion(eef[1])).reshape(3, 3)
        com_world = np.array(eef[0]) + R @ cube_offset
        p.resetBasePositionAndOrientation(bottle_id, com_world.tolist(), eef[1])
        p.performCollisionDetection()
        bad_links = set()
        for c in p.getContactPoints(bottle_id, world.robot_id):
            if c[8] >= -0.001:
                continue
            li = c[4]
            info = p.getJointInfo(world.robot_id, li) if li >= 0 else None
            lname = info[12].decode() if info else "base"
            if lname not in _GRIPPER_LINKS:
                bad_links.add(lname)
        return bad_links

    positions = get_positions("noisy")
    collision_positions = []

    print(f"\n{'='*64}")
    print(f"  Bottle MyBottle  d=7 cm  h=20 cm  2.0 kg  [{variant}]")
    print(f"  Checking {len(positions)} positions (link7/EEF contacts ignored)")
    print(f"  {'Position':<22}  Status")
    print(f"  {'-'*22}  {'-'*38}")

    for name, q in positions.items():
        bad = _place_and_check(q)
        if bad:
            status = f"COLLISION — {', '.join(sorted(bad))}"
            collision_positions.append(name)
        else:
            status = "OK"
        print(f"  {name:<22}  {status}")
        if gui and slow:
            time.sleep(1.0)

    print(f"\n  {'='*62}")
    if collision_positions:
        print(f"  Collisions at {len(collision_positions)}/{len(positions)} positions:")
        for pn in collision_positions:
            print(f"    - {pn}")
    else:
        print(f"  No collisions in all {len(positions)} positions.")
    print(f"  {'='*62}")

    if gui:
        print("\nSimulation running. Press Ctrl+C to exit.")
        stop = [False]
        signal.signal(signal.SIGINT, lambda s, f: stop.__setitem__(0, True))
        while not stop[0]:
            time.sleep(0.1)

    world.disconnect()


def main():
    import argparse
    parser = argparse.ArgumentParser(
        description="Check bottle-arm collisions through 21 positions"
    )
    parser.add_argument("--no-gui", action="store_true", help="Run without GUI")
    parser.add_argument("--heavy", action="store_true",
                        help="Use bottom-heavy bottle (CoM shifted 4 cm down)")
    parser.add_argument("--slow", action="store_true",
                        help="1 s pause between positions (GUI only)")
    known, _ = parser.parse_known_args()
    run_check(gui=not known.no_gui, heavy=known.heavy, slow=known.slow)


if __name__ == "__main__":
    main()
