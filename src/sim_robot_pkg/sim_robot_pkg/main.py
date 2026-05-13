#!/usr/bin/env python3
import os
import signal
import time
from typing import Optional
import numpy as np

from sim_robot_pkg.pybullet_world import XArm7PyBulletWorld
from com_estimation_pkg.calculator import CoMCalculator
from com_estimation_pkg.positions import (
    get_positions,
    HOME_POSE, PRE_APPROACH_POSE, GRASP_POSITION, BOTTLE_COM_WORLD,
)
from com_estimation_pkg.analysis import print_measurement, print_analysis
from com_estimation_pkg.visualizer import CoMVisualizer, CoMEstimate, ObjectInfo
from com_estimation_pkg.noise_model import FrictionNoiseModel
from com_estimation_pkg import robust_estimator


class SimulationCoMRunner:
    def __init__(
        self,
        gui: bool = True,
        noisy: bool = False,
        use_upp: bool = True,
        position_mode: str = "noisy",
        method: str = "ransac",
        compare: bool = False,
        noise_scale: float = 1.0,
        noise_seed: int = None,
    ):
        self.gui = gui
        self.noisy = noisy
        self.use_upp = use_upp
        self.position_mode = position_mode
        self.method = method
        self.compare = compare
        self.noise_scale = noise_scale
        self.noise_seed = noise_seed

        self.world: Optional[XArm7PyBulletWorld] = None
        self.calculator = CoMCalculator(num_joints=7)
        self.visualizer = CoMVisualizer()
        self.selected_object = 0
        self.actual_mass_for_comparison = None

        if noisy:
            self.noise_model = FrictionNoiseModel(scale=noise_scale, seed=noise_seed)
        else:
            self.noise_model = None

    def select_object(self, preselected: int = None) -> int:
        available_objects = [num for num in XArm7PyBulletWorld.OBJECTS.keys() if num > 0]

        if preselected is not None:
            if preselected in available_objects:
                self.selected_object = preselected
                urdf_file = XArm7PyBulletWorld.OBJECTS[preselected]
                object_name = urdf_file.replace('.urdf', '').replace('_', ' ').title()
                print(f"\nObject: [{preselected}] {object_name}")
                return preselected
            else:
                print(f"Invalid --object {preselected}. Choose from {sorted(available_objects)}")
                return 0

        print("\n Select object for CoM estimation:\n")
        for num in sorted(available_objects):
            urdf_file = XArm7PyBulletWorld.OBJECTS[num]
            object_name = urdf_file.replace('.urdf', '').replace('_', ' ').title()
            print(f"  [{num}] {object_name}")
        print()

        while True:
            try:
                choice = input(f"Your choice ({min(available_objects)}-{max(available_objects)}): ").strip()
                choice = int(choice)
                if choice in available_objects:
                    self.selected_object = choice
                    urdf_file = XArm7PyBulletWorld.OBJECTS[choice]
                    object_name = urdf_file.replace('.urdf', '').replace('_', ' ').title()
                    print(f"\nSelected: {object_name}")
                    return choice
                print(f"Choose from {min(available_objects)} to {max(available_objects)}")
            except ValueError:
                print("Enter a number")
            except KeyboardInterrupt:
                print("\n\nExit...")
                return 0

    def _approach_sequence(self, grab: bool):
        obj_choice = self.selected_object

        if self.gui:
            self.world.move_to_positions(HOME_POSE)
            self.world.move_to_positions(PRE_APPROACH_POSE)
            self.world.move_to_positions(GRASP_POSITION)
        else:
            self.world.set_joint_positions(GRASP_POSITION)

        if grab:
            self.world.remove_standing_object()
            self.world.attach_cube(obj_choice)

    def _measure_positions(self, positions: dict) -> tuple:
        """Move through all positions and collect (torques, jacobians)."""
        torques = {}
        jacobians = {}
        for name, q in positions.items():
            if self.gui:
                self.world.move_to_positions(HOME_POSE)  # UPP: same approach dir
                self.world.move_to_positions(q)
            else:
                self.world.set_joint_positions(q)
                time.sleep(0.05)
            torques[name] = self.world.compute_gravity_torques(q)
            jacobians[name] = self.world.compute_eef_jacobian(q)
        return torques, jacobians

    def run_measurements(self):
        self.calculator.clear()
        positions = get_positions(self.position_mode)

        if self.noisy:
            seed_label = f"  seed={self.noise_seed}" if self.noise_seed is not None else "  seed=random"
            upp_label = "UPP active — Coulomb cancels in Δτ" if self.use_upp else "no UPP — Coulomb does NOT cancel"
            print(f"\n[Noise] {self.noise_model.describe()}{seed_label}")
            print(f"[Protocol] {upp_label}")

        if self.gui:
            self.world.set_joint_positions(HOME_POSE)
        else:
            self.world.set_joint_positions(HOME_POSE)

        if self.gui:
            self.world.place_standing_object(self.selected_object, BOTTLE_COM_WORLD)

        print("\n[Phase 1] Approach without object (UPP: identical trajectory)...")
        self.world.detach_cube()
        self._approach_sequence(grab=False)

        print("[Phase 1] Measuring without object (HOME → each position)...")
        empty_torques, jacobians = self._measure_positions(positions)

        print("\n[Phase 2] Approach and GRAB object (same trajectory as Phase 1)...")
        if self.gui:
            self.world.place_standing_object(self.selected_object, BOTTLE_COM_WORLD)
            self.world.move_to_positions(HOME_POSE)
        self._approach_sequence(grab=True)

        print("[Phase 2] Measuring with object (HOME → each position)...")
        with_torques, _ = self._measure_positions(positions)

        for name, q in positions.items():
            jac_lin, jac_rot, R_eef = jacobians[name]

            if self.noise_model is not None:
                if self.use_upp:
                    t_empty, t_with = self.noise_model.apply_pair(
                        empty_torques[name], with_torques[name]
                    )
                else:
                    t_empty = self.noise_model.apply(empty_torques[name])
                    t_with  = self.noise_model.apply(with_torques[name])
            else:
                t_empty = empty_torques[name]
                t_with  = with_torques[name]

            measurement = self.calculator.add_measurement(
                position_name=name,
                joint_positions=q,
                torques_empty=t_empty,
                torques_with_object=t_with,
                jacobian_lin=jac_lin,
                jacobian_rot=jac_rot,
                eef_rotation=R_eef,
            )
            print_measurement(measurement)

    def _print_comparison_table(self, actual_mass: float = None, actual_com=None):
        methods = ["lsq", "huber", "ransac"]
        results = {}
        for m in methods:
            try:
                results[m] = robust_estimator.estimate_com(self.calculator, method=m)
            except Exception as e:
                print(f"  [{m}] failed: {e}")

        print("\n" + "=" * 84)
        print(" METHOD COMPARISON")
        print("=" * 84)
        header = (
            f"{'Method':<18} | {'Mass (kg)':>9} | {'Err%':>6} | "
            f"{'CoM err (m)':>11} | {'Confidence':>10} | {'MAD (Nm)':>8}"
        )
        print(header)
        print("-" * 84)
        for m, r in results.items():
            mass_err = "  n/a "
            com_err_str = "    n/a    "
            if actual_mass is not None:
                pct = abs(r.mass - actual_mass) / actual_mass * 100
                mass_err = f"{pct:6.1f}"
            if actual_com is not None:
                com_err = float(np.linalg.norm(np.array(r.position) - np.array(actual_com)))
                com_err_str = f"{com_err:.4f}"
            mad_str = f"{r.mad:8.3f}" if getattr(r, 'mad', 0.0) > 0 else "      —  "
            print(
                f"  {r.method:<16} | {r.mass:9.4f} | {mass_err:>6} | "
                f"{com_err_str:>11} | {r.confidence:10.3f} | {mad_str}"
            )
        print("=" * 84)

    def run(self, preselected_object: int = None):
        if self.select_object(preselected=preselected_object) == 0:
            return

        if (self.noisy and self.position_mode == "clean"
                and self.selected_object in XArm7PyBulletWorld.LIGHT_OBJECTS):
            print(
                f"\n[WARN] Object {self.selected_object} is too light (<1 kg) for clean mode "
                f"with noise — friction noise ≈ signal. Switching to 'noisy' (21 positions)."
            )
            self.position_mode = "noisy"

        mode_label = {"clean": "6", "noisy": "21"}[self.position_mode]
        if self.noisy:
            upp_tag = "+UPP" if self.use_upp else " no-UPP"
            noisy_label = f"  noise_scale={self.noise_scale} {upp_tag}"
        else:
            noisy_label = "  (no noise)"
        print(f"\n Initializing PyBullet...  [{mode_label} positions]{noisy_label}")
        self.world = XArm7PyBulletWorld(cube_choice=0, gui=self.gui)

        self.run_measurements()

        obj_info = self.world.get_attached_object_info()
        if obj_info:
            self.actual_mass_for_comparison = obj_info['mass']

        actual_com = None
        if obj_info and obj_info.get('actual_com') is not None:
            attachment = np.array(obj_info['position'])
            com_local  = np.array(obj_info['actual_com'])
            actual_com = tuple(attachment + com_local)

        if self.compare:
            self._print_comparison_table(self.actual_mass_for_comparison, actual_com)
            self._visualize_all_methods(actual_com)
        else:
            print_analysis(self.calculator, actual_mass=self.actual_mass_for_comparison)
            self._visualize_com(self.actual_mass_for_comparison, method=self.method)

        if self.gui:
            self.world.run_realtime()
            print("\nSimulation running. Press Ctrl+C to exit...")

            self._stop_requested = False

            def _handle_sigint(sig, frame):
                self._stop_requested = True
            signal.signal(signal.SIGINT, _handle_sigint)

            while not self._stop_requested:
                time.sleep(0.1)

        self.world.disconnect()
        print("Done.")

    def _visualize_all_methods(self, actual_com=None):
        """Generate one PNG per method + one combined comparison figure."""
        if not self.calculator.measurements:
            return

        methods = ["lsq", "huber", "ransac"]
        obj_info = self.world.get_attached_object_info()

        if obj_info:
            object_info = ObjectInfo(
                name=obj_info['name'],
                size=obj_info['size'],
                color=obj_info['color'],
                actual_com=obj_info['actual_com'],
                attachment_offset=obj_info['position'],
                mass=obj_info['mass'],
                urdf_file=obj_info['urdf_file'],
                mesh_filename=obj_info.get('mesh_filename', ''),
            )
        else:
            object_info = ObjectInfo()

        if self.noisy:
            upp_tag = "_upp" if self.use_upp else "_no_upp"
            noise_tag = f"_noisy{self.noise_scale}{upp_tag}"
        else:
            noise_tag = ""
        obj_slug = obj_info.get('slug', object_info.name.replace(' ', '_').lower()) if obj_info else object_info.name.replace(' ', '_').lower()
        output_dir = os.path.join(os.getcwd(), 'output')
        os.makedirs(output_dir, exist_ok=True)

        comparison_results = {}
        for m in methods:
            try:
                result = robust_estimator.estimate_com(self.calculator, method=m)
            except Exception as e:
                print(f"  [{m}] skipped for visualization: {e}")
                continue

            com_estimate = CoMEstimate(
                position=result.position,
                mass=result.mass,
                uncertainty=result.uncertainty,
                confidence=result.confidence,
            )
            comparison_results[result.method] = com_estimate

            save_path = os.path.join(
                output_dir,
                f'com_simulation_{obj_slug}{noise_tag}_{m}.png',
            )
            self.visualizer.visualize(
                com_estimate=com_estimate,
                object_info=object_info,
                eef_position=(0, 0, 0),
                title=f"CoM — {result.method}",
                save_path=save_path,
                show=False,
            )
            print(f"  Saved: {save_path}")

        compare_path = os.path.join(
            output_dir,
            f'com_simulation_{obj_slug}{noise_tag}_COMPARE.png',
        )
        self.visualizer.visualize_comparison(
            results=comparison_results,
            object_info=object_info,
            eef_position=(0, 0, 0),
            save_path=compare_path,
        )

    def _visualize_com(self, actual_mass: float = None, method: str = "ransac"):
        if not self.calculator.measurements:
            print("No measurements available for visualization")
            return

        result = robust_estimator.estimate_com(self.calculator, method=method)

        com_estimate = CoMEstimate(
            position=result.position,
            mass=result.mass,
            uncertainty=result.uncertainty,
            confidence=result.confidence,
        )

        obj_info = self.world.get_attached_object_info()
        if obj_info:
            object_info = ObjectInfo(
                name=obj_info['name'],
                size=obj_info['size'],
                color=obj_info['color'],
                actual_com=obj_info['actual_com'],
                attachment_offset=obj_info['position'],
                mass=obj_info['mass'],
                urdf_file=obj_info['urdf_file'],
                mesh_filename=obj_info.get('mesh_filename', ''),
            )
        else:
            object_info = ObjectInfo()

        if self.noisy:
            upp_tag = "_upp" if self.use_upp else "_no_upp"
            noise_tag = f"_noisy{self.noise_scale}{upp_tag}"
        else:
            noise_tag = ""
        object_filename = obj_info.get('slug', object_info.name.replace(' ', '_').lower()) if obj_info else object_info.name.replace(' ', '_').lower()
        output_dir = os.path.join(os.getcwd(), 'output')
        os.makedirs(output_dir, exist_ok=True)
        save_path = os.path.join(
            output_dir,
            f'com_simulation_{object_filename}{noise_tag}_{method}.png',
        )

        self.visualizer.visualize(
            com_estimate=com_estimate,
            object_info=object_info,
            eef_position=(0, 0, 0),
            save_path=save_path,
            show=False,
        )

        print(f"\nVisualization saved: {save_path}  [method={method}]")
        if getattr(result, 'mad', 0.0) > 0:
            print(f"MAD of initial residuals (Huber f_scale): {result.mad:.3f} Nm")
        distance = np.linalg.norm(result.position)
        print(
            f"CoM (relative to EEF): "
            f"[{result.position[0]:.4f}, {result.position[1]:.4f}, {result.position[2]:.4f}] m"
        )
        print(f"Distance from EEF:     {distance:.4f} m")

        if actual_mass:
            error_percent = abs(result.mass - actual_mass) / actual_mass * 100
            print(f"\nMass comparison [{method}]:")
            print(f"  Estimated: {result.mass:.4f} kg")
            print(f"  Actual:    {actual_mass:.4f} kg")
            print(f"  Error:     {error_percent:.1f}%")


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="xArm7 CoM estimation (simulation)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Noise states:
  (no --noisy)          clean — no noise; 6 positions suffice
  --noisy               noisy with UPP — Coulomb cancels in Δτ; 21 positions
  --noisy --no-upp      noisy without UPP — Coulomb does NOT cancel; 21 positions
        """,
    )
    parser.add_argument("--no-gui", action="store_true", help="Run without GUI")
    parser.add_argument(
        "--noisy", action="store_true",
        help="Apply friction noise to torque measurements",
    )
    parser.add_argument(
        "--no-upp", action="store_true",
        help="Disable Unidirectional Positioning Protocol (Coulomb noise will NOT cancel in Δτ)",
    )
    parser.add_argument(
        "--noise-scale", type=float, default=1.0,
        help="Friction noise amplitude multiplier (default 1.0 = realistic xArm7)",
    )
    parser.add_argument(
        "--noise-seed", type=int, default=None,
        help="RNG seed for noise (fixes noise for reproducible comparison runs)",
    )
    parser.add_argument(
        "--positions", type=str, default="noisy",
        choices=["clean", "noisy"],
        help="Position set: clean=6 (for noise-free), noisy=21 (optimal under friction noise)",
    )
    parser.add_argument(
        "--method", type=str, default="ransac",
        choices=["lsq", "huber", "ransac"],
        help="CoM estimation method",
    )
    parser.add_argument(
        "--compare", action="store_true",
        help="Run all three methods and print a comparison table",
    )
    parser.add_argument(
        "--object", type=int, default=None,
        help="Object ID (1–8) to skip interactive selection (for scripting)",
    )

    args = parser.parse_args()

    runner = SimulationCoMRunner(
        gui=not args.no_gui,
        noisy=args.noisy,
        use_upp=not args.no_upp,
        position_mode=args.positions,
        method=args.method,
        compare=args.compare,
        noise_scale=args.noise_scale,
        noise_seed=args.noise_seed,
    )
    runner.run(preselected_object=args.object)


if __name__ == '__main__':
    main()
