#!/usr/bin/env python3
import os
import sys
import time
import argparse
from typing import Dict, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node

from com_estimation_pkg.calculator import CoMCalculator, format_measurements_table
from com_estimation_pkg.positions import get_positions
from com_estimation_pkg.analysis import print_analysis
from com_estimation_pkg.visualizer import CoMVisualizer, CoMEstimate, ObjectInfo
from com_estimation_pkg import robust_estimator
from real_robot_pkg.torque_reader import TorqueReader
from real_robot_pkg.arm_mover import ArmMover


class _JacobianBackend:
    def __init__(self):
        from sim_robot_pkg.pybullet_world import XArm7PyBulletWorld
        self._world = XArm7PyBulletWorld(cube_choice=0, gui=False)

    def get_jacobians(self, joint_positions: List[float]):
        self._world.set_joint_positions(joint_positions)
        return self._world.compute_eef_jacobian(joint_positions)

    def close(self):
        self._world.disconnect()

def _sample_torques(
    reader: TorqueReader,
    spin_fn,
    n_samples: int = 15,
    sigma_reject: float = 2.0,
) -> Optional[List[float]]:
    samples: List[List[float]] = []
    for _ in range(n_samples):
        spin_fn(timeout_sec=0.3)
        t = reader.get_torques()
        if t is not None:
            samples.append(t)
        time.sleep(0.05)

    if len(samples) < 3:
        return None

    arr  = np.array(samples)
    mean = np.mean(arr, axis=0)
    std  = np.std(arr, axis=0) + 1e-9
    z    = np.max(np.abs(arr - mean) / std, axis=1)
    mask = z < sigma_reject
    if mask.sum() < 3:
        mask = np.ones(len(samples), dtype=bool)

    return np.mean(arr[mask], axis=0).tolist()


class RealRobotCoMRunner(Node):
    def __init__(
        self,
        position_mode: str = "noisy",
        method: str = "ransac",
        compare: bool = False,
        n_samples: int = 15,
        settle_sec: float = 1.0,
        raw_efforts: bool = True,
    ):
        super().__init__("real_robot_com_runner")

        self.position_mode = position_mode
        self.method        = method
        self.compare       = compare
        self.n_samples     = n_samples
        self.settle_sec    = settle_sec

        self._torque_reader = TorqueReader(
            "torque_reader_internal", raw_efforts=raw_efforts
        )
        self._arm     = ArmMover("arm_mover_internal")
        self._calc    = CoMCalculator(num_joints=7)
        self._vis     = CoMVisualizer()
        self._jac: Optional[_JacobianBackend] = None

        self._empty_data: Dict[str, dict] = {}

        effort_label = "raw Nm" if raw_efforts else "current→torque"
        self.get_logger().info("=" * 60)
        self.get_logger().info("  xArm7 CoM Estimation — Real Robot (autonomous)")
        self.get_logger().info(
            f"  positions={position_mode}  method={method}  "
            f"efforts={effort_label}  settle={settle_sec}s"
        )
        self.get_logger().info("=" * 60)

    def _spin(self, timeout_sec: float = 0.3):
        rclpy.spin_once(self._torque_reader, timeout_sec=timeout_sec)

    def _read_torques(self) -> Optional[List[float]]:
        return _sample_torques(self._torque_reader, self._spin, n_samples=self.n_samples)

    def _get_position(self) -> Optional[List[float]]:
        state = self._torque_reader.get_current_state()
        return state.joint_positions if state else None

    def _settle(self):
        """Wait for vibrations to damp out before reading torques."""
        time.sleep(self.settle_sec)

    def _banner(self, text: str):
        print(f"\n{'='*60}\n  {text}\n{'='*60}")

    def _run_phase1(self, positions: dict) -> bool:
        self._banner("PHASE 1 — Baseline (no object, gripper at bottle-grip position)")
        print("  [UPP] Each pose approached from HOME.\n")

        self._arm.close_to_bottle()

        for i, (name, q) in enumerate(positions.items()):
            print(f"  [{i+1}/{len(positions)}] {name}")

            if not self._arm.move_home():
                print("    [WARN] HOME failed — skipping.")
                continue
            if not self._arm.move_to_joints(q):
                print("    [WARN] Motion failed — skipping.")
                continue

            self._settle()
            torques  = self._read_torques()
            position = self._get_position()

            if torques is None or position is None:
                print("    [WARN] Could not read torques — skipping.")
                continue

            self._empty_data[name] = {
                "torques":  torques,
                "position": position,
                "target_q": q,
            }
            print(f"    τ = [{', '.join(f'{t:+.3f}' for t in torques)}]")

        return bool(self._empty_data)

    def _run_phase2(self, positions: dict):
        self._banner("PHASE 2 — Loaded (object attached, gripper at bottle-grip position)")
        print("  [UPP] Each pose approached from HOME.\n")

        for i, name in enumerate(self._empty_data.keys()):
            q = positions[name]
            print(f"  [{i+1}/{len(self._empty_data)}] {name}")

            if not self._arm.move_home():
                print("    [WARN] HOME failed — skipping.")
                continue
            if not self._arm.move_to_joints(q):
                print("    [WARN] Motion failed — skipping.")
                continue

            self._settle()
            torques_with = self._read_torques()

            if torques_with is None:
                print("    [WARN] Could not read torques — skipping.")
                continue

            calib          = self._empty_data[name]
            torques_empty  = calib["torques"]
            joint_positions = calib["position"]

            jac_lin, jac_rot, R_eef = self._jac.get_jacobians(joint_positions)
            self._calc.add_measurement(
                position_name=name,
                joint_positions=joint_positions,
                torques_empty=torques_empty,
                torques_with_object=torques_with,
                jacobian_lin=jac_lin,
                jacobian_rot=jac_rot,
                eef_rotation=R_eef,
            )

            delta = [w - e for w, e in zip(torques_with, torques_empty)]
            print(f"    Δτ = [{', '.join(f'{d:+.3f}' for d in delta)}]")

    def run(self):
        positions = get_positions(self.position_mode)

        self.get_logger().info("Waiting for /joint_states …")
        if not self._torque_reader.wait_for_state(timeout=15.0):
            self.get_logger().error("Timeout: cannot reach robot.")
            return
        self.get_logger().info("Robot connected.")

        print("\nWaiting for xarm_planner services …")
        if not self._arm.wait_for_services(timeout=20.0):
            self.get_logger().error("Planner services unavailable. Is xarm_planner running?")
            return
        print("  Planner ready.\n")

        print("Starting PyBullet kinematics backend …")
        self._jac = _JacobianBackend()
        print("  Done.\n")

        print("Resetting robot state …")
        self._arm.reset_robot()

        self._arm.open_gripper()
        self._arm.move_home()

        if not self._run_phase1(positions):
            print("[ERROR] No baseline data collected. Aborting.")
            self._cleanup()
            return

        self._arm.move_home()

        self._banner("PICK — Automated bottle pickup")
        if not self._arm.pick_bottle():
            print("[ERROR] Bottle pick failed. Aborting.")
            self._cleanup()
            return

        time.sleep(0.5)

        self._run_phase2(positions)
        self._arm.move_home()
        self._arm.open_gripper()
        self._cleanup()

        if not self._calc.measurements:
            print("[ERROR] No measurements collected. Aborting.")
            return

        self._banner("RESULTS")
        print("\nΔτ table:")
        print(format_measurements_table(self._calc.measurements))

        if self.compare:
            self._print_comparison_table()
        else:
            print_analysis(self._calc)

        self._visualize(method=self.method if not self.compare else "ransac")

    def _cleanup(self):
        if self._jac is not None:
            self._jac.close()
            self._jac = None

    def _print_comparison_table(self):
        methods  = ["lsq", "huber", "ransac"]
        results  = {}
        for m in methods:
            try:
                results[m] = robust_estimator.estimate_com(self._calc, method=m)
            except Exception as e:
                print(f"  [{m}] failed: {e}")

        print("\n" + "=" * 72)
        print("  METHOD COMPARISON")
        print("=" * 72)
        print(f"  {'Method':<22} | {'Mass (kg)':>9} | {'CoM (m)':^26} | {'Conf':>5}")
        print("  " + "-" * 70)
        for m, r in results.items():
            cx, cy, cz = r.position
            print(
                f"  {r.method:<22} | {r.mass:9.4f} | "
                f"[{cx:+.4f}, {cy:+.4f}, {cz:+.4f}] | {r.confidence:5.3f}"
            )
        print("=" * 72)

    def _visualize(self, method: str):
        try:
            result = robust_estimator.estimate_com(self._calc, method=method)
        except Exception as e:
            print(f"[WARN] Visualisation skipped: {e}")
            return

        com_estimate = CoMEstimate(
            position=result.position,
            mass=result.mass,
            uncertainty=result.uncertainty,
            confidence=result.confidence,
        )

        output_dir = os.path.join(os.getcwd(), "output")
        os.makedirs(output_dir, exist_ok=True)
        n = len(self._calc.measurements)
        save_path = os.path.join(
            output_dir,
            f"com_real_robot_{self.position_mode}_n{n}_{method}.png",
        )

        self._vis.visualize(
            com_estimate=com_estimate,
            object_info=ObjectInfo(),
            eef_position=(0, 0, 0),
            save_path=save_path,
            show=False,
        )

        print(f"\nVisualization saved: {save_path}  [method={method}]")
        print(
            f"CoM (EEF frame): [{result.position[0]:+.4f}, "
            f"{result.position[1]:+.4f}, {result.position[2]:+.4f}] m"
        )
        print(f"Mass: {result.mass:.4f} kg  |  Confidence: {result.confidence:.3f}")


def main(args=None):
    parser = argparse.ArgumentParser(
        description="xArm7 CoM estimation — real robot (autonomous)"
    )
    def _str_to_bool(value: str) -> bool:
        return value.lower() in {"1", "true", "yes", "y", "on"}

    parser.add_argument(
        "--positions", default="noisy", choices=["clean", "noisy"],
        help="Pose set: clean=6, noisy=21 (default)",
    )
    parser.add_argument(
        "--method", default="ransac", choices=["lsq", "huber", "ransac"],
        help="Estimation method (default: ransac)",
    )
    parser.add_argument(
        "--compare", nargs="?", const=True, default=False, type=_str_to_bool,
        help="Print comparison table for all three methods",
    )
    parser.add_argument(
        "--samples", type=int, default=15,
        help="Torque samples per pose (default: 15)",
    )
    parser.add_argument(
        "--settle", type=float, default=1.0,
        help="Seconds to wait after each move before reading torques (default: 1.0)",
    )
    parser.add_argument(
        "--convert-current", action="store_true",
        help=(
            "Multiply raw efforts by motor current-to-torque constants. "
            "Use ONLY when the xArm driver reports current (set_report_tau_or_i=1)."
        ),
    )
    known, _ = parser.parse_known_args(args)

    rclpy.init(args=args)
    try:
        runner = RealRobotCoMRunner(
            position_mode=known.positions,
            method=known.method,
            compare=known.compare,
            n_samples=known.samples,
            settle_sec=known.settle,
            raw_efforts=not known.convert_current,
        )
        runner.run()
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def run_quick_test(args=None):
    parser = argparse.ArgumentParser(description="Quick xArm7 connection test")
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument("--convert-current", action="store_true")
    known, _ = parser.parse_known_args(args)

    rclpy.init(args=args)
    try:
        reader = TorqueReader("quick_test_reader", raw_efforts=not known.convert_current)
        print("Waiting for /joint_states …")
        if not reader.wait_for_state(timeout=10.0):
            print("[ERROR] Timeout — is the xArm7 driver running?")
            return

        print(f"\n{'Joint':<10} {'Position (rad)':>15} {'Velocity':>10} {'Effort':>10}")
        print("-" * 48)
        for i in range(known.samples):
            rclpy.spin_once(reader, timeout_sec=0.3)
            state = reader.get_current_state()
            if state is None:
                continue
            print(f"\n-- sample {i+1} --")
            for j, (p, v, e) in enumerate(
                zip(state.joint_positions, state.joint_velocities, state.joint_efforts)
            ):
                print(f"  joint{j+1:<4} {p:+15.4f} {v:+10.4f} {e:+10.4f}")
            time.sleep(0.2)
        print("\nConnection OK.")
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
