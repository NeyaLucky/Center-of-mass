import numpy as np
from typing import List


class FrictionNoiseModel:
    DEFAULT_COULOMB = [2.0, 3.0, 2.5, 1.5, 1.0, 0.8, 0.3]
    DEFAULT_SENSOR_STD = [0.3, 0.3, 0.3, 0.2, 0.2, 0.2, 0.1]
    DEFAULT_VISCOUS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def __init__(
        self,
        coulomb: List[float] = None,
        sensor_std: List[float] = None,
        viscous: List[float] = None,
        scale: float = 1.0,
        seed: int = None,
    ):
        self.coulomb = np.array(coulomb if coulomb is not None else self.DEFAULT_COULOMB)
        self.sensor_std = np.array(sensor_std if sensor_std is not None else self.DEFAULT_SENSOR_STD)
        self.viscous = np.array(viscous if viscous is not None else self.DEFAULT_VISCOUS)
        self.scale = scale
        self.rng = np.random.default_rng(seed)

    def apply(self, torques: List[float], q_dot: List[float] = None) -> List[float]:
        torques = np.array(torques, dtype=float)
        n = len(torques)

        coulomb = self.coulomb[:n] * self.scale
        sensor_std = self.sensor_std[:n] * self.scale

        signs = self.rng.choice([-1.0, 1.0], size=n)
        coulomb_noise = signs * coulomb

        sensor_noise = self.rng.normal(0.0, sensor_std)

        if q_dot is not None and np.any(self.viscous[:n] > 0):
            viscous_noise = self.viscous[:n] * self.scale * np.array(q_dot[:n], dtype=float)
        else:
            viscous_noise = 0.0

        return (torques + coulomb_noise + sensor_noise + viscous_noise).tolist()

    def apply_pair(
        self,
        torques_empty: List[float],
        torques_with: List[float],
        q_dot: List[float] = None,
    ):
        t_empty = np.array(torques_empty, dtype=float)
        t_with = np.array(torques_with, dtype=float)
        n = len(t_empty)

        coulomb = self.coulomb[:n] * self.scale
        sensor_std = self.sensor_std[:n] * self.scale

        signs = self.rng.choice([-1.0, 1.0], size=n)
        coulomb_noise = signs * coulomb

        if q_dot is not None and np.any(self.viscous[:n] > 0):
            viscous_noise = self.viscous[:n] * self.scale * np.array(q_dot[:n], dtype=float)
        else:
            viscous_noise = 0.0

        noisy_empty = t_empty + coulomb_noise + self.rng.normal(0.0, sensor_std) + viscous_noise
        noisy_with  = t_with  + coulomb_noise + self.rng.normal(0.0, sensor_std) + viscous_noise

        return noisy_empty.tolist(), noisy_with.tolist()

    def describe(self) -> str:
        lines = ["FrictionNoiseModel (scale={:.2f}):".format(self.scale)]
        for i, (c, s, v) in enumerate(zip(self.coulomb, self.sensor_std, self.viscous)):
            viscous_str = f",  viscous b={v * self.scale:.3f} Nm·s/rad" if v > 0 else ""
            lines.append(
                f"  joint{i+1}: Coulomb ±{c * self.scale:.2f} Nm,"
                f"  sensor σ={s * self.scale:.2f} Nm{viscous_str}"
            )
        return "\n".join(lines)
