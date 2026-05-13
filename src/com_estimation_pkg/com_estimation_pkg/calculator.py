import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass, field


@dataclass
class TorqueMeasurement:
    position_name: str
    joint_positions: List[float]
    torques_empty: List[float]    
    torques_with_object: List[float]
    delta_torques: List[float] = field(default_factory=list)
    jacobian_lin: Optional[np.ndarray] = None   
    jacobian_rot: Optional[np.ndarray] = None   
    eef_rotation: Optional[np.ndarray] = None   
    
    def __post_init__(self):
        if not self.delta_torques:
            self.delta_torques = [
                tw - te for tw, te in zip(self.torques_with_object, self.torques_empty)
            ]


@dataclass
class CoMResult:
    mass: float
    position: Tuple[float, float, float]     
    uncertainty: Tuple[float, float, float]  
    confidence: float
    
    most_affected_joint: int  
    mean_delta_torque: float   
    residual_norm: float = 0.0 
    method: str = "unknown"    
    mad: float = 0.0           


class CoMCalculator:
    
    GRAVITY = 9.81
    
    def __init__(self, num_joints: int = 7):
        self.num_joints = num_joints
        self.measurements: List[TorqueMeasurement] = []
    
    def clear(self):
        self.measurements = []
    
    def add_measurement(
        self, 
        position_name: str,
        joint_positions: List[float],
        torques_empty: List[float],
        torques_with_object: List[float],
        jacobian_lin: np.ndarray = None,
        jacobian_rot: np.ndarray = None,
        eef_rotation: np.ndarray = None
    ) -> TorqueMeasurement:
        measurement = TorqueMeasurement(
            position_name=position_name,
            joint_positions=joint_positions,
            torques_empty=torques_empty,
            torques_with_object=torques_with_object,
            jacobian_lin=np.array(jacobian_lin) if jacobian_lin is not None else None,
            jacobian_rot=np.array(jacobian_rot) if jacobian_rot is not None else None,
            eef_rotation=np.array(eef_rotation) if eef_rotation is not None else None
        )
        self.measurements.append(measurement)
        return measurement
    
    def get_delta_matrix(self) -> np.ndarray:
        if not self.measurements:
            return np.array([])
        return np.array([m.delta_torques for m in self.measurements])
    
    def get_statistics(self) -> dict:
        delta_matrix = self.get_delta_matrix()
        if delta_matrix.size == 0:
            return {}
        
        stats = {}
        for i in range(self.num_joints):
            col = delta_matrix[:, i]
            stats[f"joint{i+1}"] = {
                "mean": float(np.mean(col)),
                "std": float(np.std(col)),
                "min": float(np.min(col)),
                "max": float(np.max(col)),
                "mean_abs": float(np.mean(np.abs(col))),
            }
        return stats
    
    def get_most_affected_joint(self) -> Tuple[int, float]:
        stats = self.get_statistics()
        if not stats:
            return (0, 0.0)
        
        max_joint = 1
        max_val = 0.0
        for i in range(1, self.num_joints + 1):
            val = stats[f"joint{i}"]["mean_abs"]
            if val > max_val:
                max_val = val
                max_joint = i
        
        return (max_joint, max_val)
    
    def build_lsq_matrices(self):
        if not self._has_jacobians():
            raise ValueError("No Jacobian data available — cannot build LSQ matrices.")

        A_rows, b_rows = [], []
        for meas in self.measurements:
            Jl = meas.jacobian_lin
            Jr = meas.jacobian_rot
            R  = meas.eef_rotation
            delta = np.array(meas.delta_torques)
            for j in range(self.num_joints):
                A_rows.append([
                    -Jl[2, j],
                    R[0, 0] * Jr[1, j] - R[1, 0] * Jr[0, j],
                    R[0, 1] * Jr[1, j] - R[1, 1] * Jr[0, j],
                    R[0, 2] * Jr[1, j] - R[1, 2] * Jr[0, j],
                ])
                b_rows.append(delta[j])
        return np.array(A_rows), np.array(b_rows)

    def build_lsq_matrices_by_pose(self, min_row_norm: float = 0.05):
        if not self._has_jacobians():
            raise ValueError("No Jacobian data available — cannot build LSQ matrices.")

        result = []
        for meas in self.measurements:
            Jl = meas.jacobian_lin
            Jr = meas.jacobian_rot
            R  = meas.eef_rotation
            delta = np.array(meas.delta_torques)

            A_rows, b_rows = [], []
            for j in range(self.num_joints):
                row = np.array([
                    -Jl[2, j],
                    R[0, 0] * Jr[1, j] - R[1, 0] * Jr[0, j],
                    R[0, 1] * Jr[1, j] - R[1, 1] * Jr[0, j],
                    R[0, 2] * Jr[1, j] - R[1, 2] * Jr[0, j],
                ])
                if np.linalg.norm(row) < min_row_norm:
                    continue
                A_rows.append(row)
                b_rows.append(float(delta[j]))

            if len(A_rows) >= 2:
                result.append((np.array(A_rows), np.array(b_rows), meas.position_name))

        return result

    def _has_jacobians(self) -> bool:
        return (
            len(self.measurements) > 0 
            and all(
                m.jacobian_lin is not None 
                and m.jacobian_rot is not None
                and m.eef_rotation is not None
                for m in self.measurements
            )
        )
    
    def estimate_com_jacobian(self) -> CoMResult:
        A, b = self.build_lsq_matrices()
        
        result = np.linalg.lstsq(A, b, rcond=None)
        x = result[0]
        
        mg = x[0]
        mass = abs(mg) / self.GRAVITY
        
        if abs(mg) > 0.01:
            rx = x[1] / mg
            ry = x[2] / mg
            rz = x[3] / mg
        else:
            rx, ry, rz = 0.0, 0.0, 0.0
        
        com_position = (float(rx), float(ry), float(rz))
        
        predicted = A @ x
        residual_vec = b - predicted
        lsq_residual = float(np.linalg.norm(residual_vec))
        
        n_obs = A.shape[0]
        n_params = A.shape[1]
        if n_obs > n_params and abs(mg) > 0.01:
            mse = lsq_residual**2 / (n_obs - n_params)
            try:
                cov = mse * np.linalg.inv(A.T @ A)
                unc_rx = float(np.sqrt(abs(cov[1, 1]))) / abs(mg)
                unc_ry = float(np.sqrt(abs(cov[2, 2]))) / abs(mg)
                unc_rz = float(np.sqrt(abs(cov[3, 3]))) / abs(mg)
                uncertainty = (
                    float(np.clip(unc_rx, 0.001, 0.1)),
                    float(np.clip(unc_ry, 0.001, 0.1)),
                    float(np.clip(unc_rz, 0.001, 0.1)),
                )
            except np.linalg.LinAlgError:
                uncertainty = (0.02, 0.02, 0.02)
        else:
            uncertainty = (0.03, 0.03, 0.03)
        
        signal_norm = np.linalg.norm(b)
        if signal_norm > 1e-6:
            snr = signal_norm / max(lsq_residual, 1e-9)
            confidence = float(np.clip(snr / (1.0 + snr), 0.01, 0.99))
        else:
            confidence = 0.01
        
        joint_idx, mean_delta = self.get_most_affected_joint()
        
        return CoMResult(
            mass=mass,
            position=com_position,
            uncertainty=uncertainty,
            confidence=confidence,
            most_affected_joint=joint_idx,
            mean_delta_torque=mean_delta,
            residual_norm=lsq_residual,
            method="jacobian_lsq_4dof"
        )
    
    def estimate_com(self) -> CoMResult:
        if not self.measurements:
            return CoMResult(
                mass=0.0, position=(0, 0, 0), uncertainty=(1, 1, 1),
                confidence=0.0, most_affected_joint=0, mean_delta_torque=0.0
            )
        
        if self._has_jacobians():
            return self.estimate_com_jacobian()
        else:
            return self._estimate_com_fallback()
    
    def _estimate_com_fallback(self) -> CoMResult:
        delta_matrix = self.get_delta_matrix()
        joint_idx, mean_delta = self.get_most_affected_joint()
        
        all_abs_deltas = np.abs(delta_matrix)
        overall_mean = np.mean(all_abs_deltas)
        estimated_mass = overall_mean / (self.GRAVITY * 0.4)
        
        mean_deltas = np.mean(delta_matrix, axis=0)
        std_deltas = np.std(delta_matrix, axis=0)
        
        com_x = float(np.clip(np.sign(mean_deltas[0]) * 0.005, -0.02, 0.02))
        com_y = float(np.clip(np.sign(mean_deltas[1]) * 0.005, -0.02, 0.02))
        com_z = 0.025
        
        uncertainty = (0.02, 0.02, 0.02)
        
        consistency = 1.0 / (1.0 + np.mean(std_deltas))
        confidence = min(consistency * 0.7, 0.6)
        
        return CoMResult(
            mass=estimated_mass,
            position=(com_x, com_y, com_z),
            uncertainty=uncertainty,
            confidence=confidence,
            most_affected_joint=joint_idx,
            mean_delta_torque=mean_delta,
            residual_norm=float(np.mean(std_deltas)),
            method="fallback"
        )
    
    def estimate_mass(self) -> float:
        if self._has_jacobians():
            result = self.estimate_com_jacobian()
            return result.mass
        else:
            result = self._estimate_com_fallback()
            return result.mass


def format_statistics(stats: dict) -> str:
    lines = []
    for joint, data in stats.items():
        lines.append(
            f"   {joint}: mean={data['mean']:+.4f}, std={data['std']:.4f}, "
            f"min={data['min']:+.4f}, max={data['max']:+.4f}"
        )
    return "\n".join(lines)


def format_measurements_table(measurements: List[TorqueMeasurement]) -> str:
    if not measurements:
        return
    
    num_joints = len(measurements[0].delta_torques)
    
    lines = []
    lines.append("-" * 80)
    header = f"{'Position':<15} | " + " | ".join([f"j{i+1}" for i in range(num_joints)])
    lines.append(header)
    lines.append("-" * 80)
    
    for m in measurements:
        row = f"{m.position_name:<15} | "
        row += " | ".join([f"{d:+.3f}" for d in m.delta_torques])
        lines.append(row)
    
    lines.append("-" * 80)
    return "\n".join(lines)
