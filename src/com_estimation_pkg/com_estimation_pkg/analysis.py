from typing import List
from .calculator import CoMCalculator, TorqueMeasurement, format_statistics, format_measurements_table


def print_measurement(measurement: TorqueMeasurement, show_raw: bool = True):
    print(f"\n Position: {measurement.position_name}")
    print(f"   Joints: {[f'{q:.2f}' for q in measurement.joint_positions]}")
    
    if show_raw:
        print(f"   τ (without object): {[f'{t:.3f}' for t in measurement.torques_empty]}")
        print(f"   τ (with object):  {[f'{t:.3f}' for t in measurement.torques_with_object]}")
    
    print(f"   Δτ: {[f'{d:.4f}' for d in measurement.delta_torques]}")


def print_analysis(calculator: CoMCalculator, actual_mass: float = None):

    print("\n")
    print("ANALYSIS RESULTS")
    
    if not calculator.measurements:
        print("No measurements to analyze")
        return
    
    stats = calculator.get_statistics()
    print("\n Δτ statistics for joints:")
    print(format_statistics(stats))
    
    joint_idx, mean_delta = calculator.get_most_affected_joint()
    print(f"\nGreatest impact on joint{joint_idx} "
          f"(mean |Δτ| = {mean_delta:.4f} Nm)")
    
    result = calculator.estimate_com()
    method_name = getattr(result, 'method', 'unknown')
    print(f"\nEstimated object mass: {result.mass:.4f} kg")
    print(f"   Method: {method_name}")
    
    if hasattr(result, 'residual_norm') and result.residual_norm > 0:
        print(f"LSQ residual: {result.residual_norm:.4f}")
    
    if actual_mass is not None:
        error = abs(result.mass - actual_mass)
        error_percent = (error / actual_mass * 100) if actual_mass > 0 else 0
        print(f"Actual mass: {actual_mass:.3f} kg")
        print(f"Error: {error:.4f} kg ({error_percent:.1f}%)")
    
    print(f"\nCoM position (relative to EEF): "
          f"({result.position[0]:.4f}, {result.position[1]:.4f}, {result.position[2]:.4f}) m")
    print(f"Uncertainty: "
          f"({result.uncertainty[0]:.4f}, {result.uncertainty[1]:.4f}, {result.uncertainty[2]:.4f}) m")
    print(f"Confidence: {result.confidence*100:.1f}%")
    
    print("\nDetailed Δτ table:")
    print(format_measurements_table(calculator.measurements))


def print_summary(calculator: CoMCalculator):
    if not calculator.measurements:
        print("No measurements")
        return
    
    joint_idx, mean_delta = calculator.get_most_affected_joint()
    estimated_mass = calculator.estimate_mass()
    
    print(f"\nMeasurements: {len(calculator.measurements)}")
    print(f"Greatest impact: joint{joint_idx} ({mean_delta:.4f} Nm)")
    print(f"Estimated mass: {estimated_mass:.3f} kg")