
import numpy as np

from com_estimation_pkg.calculator import CoMCalculator, CoMResult

GRAVITY = 9.81

_COULOMB = np.array([2.0, 3.0, 2.5, 1.5, 1.0, 0.8, 0.3])
_SENSOR  = np.array([0.3, 0.3, 0.3, 0.2, 0.2, 0.2, 0.1])


def _row_variances(n_joints: int) -> np.ndarray:
    coulomb = _COULOMB[:n_joints]
    sensor  = _SENSOR[:n_joints]
    return 2.0 * (coulomb ** 2 + sensor ** 2)


def _x_to_result(x: np.ndarray, A: np.ndarray, b: np.ndarray,
                 residual_norm: float, method: str,
                 calculator: CoMCalculator) -> CoMResult:
    mg   = float(x[0])
    mass = abs(mg) / GRAVITY

    if abs(mg) > 0.01:
        rx, ry, rz = float(x[1] / mg), float(x[2] / mg), float(x[3] / mg)
    else:
        rx = ry = rz = 0.0

    n_obs, n_params = A.shape
    if n_obs > n_params and abs(mg) > 0.01:
        mse = residual_norm ** 2 / max(n_obs - n_params, 1)
        try:
            cov = mse * np.linalg.inv(A.T @ A)
            unc = tuple(
                float(np.clip(np.sqrt(abs(cov[i, i])) / abs(mg), 0.001, 0.1))
                for i in (1, 2, 3)
            )
        except np.linalg.LinAlgError:
            unc = (0.02, 0.02, 0.02)
    else:
        unc = (0.03, 0.03, 0.03)

    signal_norm = np.linalg.norm(b)
    if signal_norm > 1e-6:
        snr = signal_norm / max(residual_norm, 1e-9)
        confidence = float(np.clip(snr / (1.0 + snr), 0.01, 0.99))
    else:
        confidence = 0.01

    joint_idx, mean_delta = calculator.get_most_affected_joint()
    return CoMResult(
        mass=mass,
        position=(rx, ry, rz),
        uncertainty=unc,
        confidence=confidence,
        most_affected_joint=joint_idx,
        mean_delta_torque=mean_delta,
        residual_norm=residual_norm,
        method=method,
    )

def estimate_huber(calculator: CoMCalculator, f_scale: float = None) -> CoMResult:
    from scipy.optimize import least_squares

    A, b = calculator.build_lsq_matrices()
    n_joints = calculator.num_joints

    var_per_joint = _row_variances(n_joints)
    n_obs = A.shape[0]
    row_vars = np.tile(var_per_joint, n_obs // n_joints + 1)[:n_obs]
    w = 1.0 / row_vars
    try:
        x0 = np.linalg.solve(A.T @ np.diag(w) @ A, A.T @ (w * b))
    except np.linalg.LinAlgError:
        x0, *_ = np.linalg.lstsq(A, b, rcond=None)

    if f_scale is None:
        init_resid = np.abs(A @ x0 - b)
        f_scale = float(np.clip(np.median(init_resid), 0.5, 5.0))

    def residuals(x):
        return A @ x - b

    res = least_squares(residuals, x0, loss='huber', f_scale=f_scale, method='trf')
    x = res.x
    resid_norm = float(np.linalg.norm(residuals(x)))
    result = _x_to_result(x, A, b, resid_norm, f"huber(f={f_scale:.2f})", calculator)
    result.mad = float(f_scale)
    return result

def estimate_ransac(
    calculator: CoMCalculator,
    inlier_threshold: float = None,
    n_trials: int = 300,
    min_samples: int = None,
    seed: int = 42,
) -> CoMResult:
    pose_data = calculator.build_lsq_matrices_by_pose(min_row_norm=0.05)
    n_poses = len(pose_data)
    n_params = 4

    if n_poses < n_params + 1:
        result = estimate_huber(calculator)
        result.method = f"ransac→huber (n_poses={n_poses}<{n_params + 1})"
        return result

    if min_samples is None:
        min_samples = min(n_params + 2, n_poses)

    A_all = np.vstack([A for A, b, _ in pose_data])
    b_all = np.concatenate([b for A, b, _ in pose_data])

    x_lsq, *_ = np.linalg.lstsq(A_all, b_all, rcond=None)

    pose_resids = np.array([
        np.linalg.norm(A @ x_lsq - b) for A, b, _ in pose_data
    ])

    if inlier_threshold is None:
        inlier_threshold = float(np.clip(np.median(pose_resids), 0.5, 15.0))

    rng = np.random.default_rng(seed)
    best_mask = pose_resids < inlier_threshold
    best_n_inliers = int(best_mask.sum())
    best_x = x_lsq.copy()

    for _ in range(n_trials):
        sample_idx = rng.choice(n_poses, size=min_samples, replace=False)
        A_s = np.vstack([pose_data[i][0] for i in sample_idx])
        b_s = np.concatenate([pose_data[i][1] for i in sample_idx])
        try:
            x_s, *_ = np.linalg.lstsq(A_s, b_s, rcond=None)
        except np.linalg.LinAlgError:
            continue

        resids = np.array([
            np.linalg.norm(pose_data[i][0] @ x_s - pose_data[i][1])
            for i in range(n_poses)
        ])
        mask = resids < inlier_threshold
        n_inliers = int(mask.sum())

        if n_inliers > best_n_inliers:
            best_n_inliers = n_inliers
            best_mask = mask
            best_x = x_s

    if best_n_inliers >= n_params:
        A_in = np.vstack([pose_data[i][0] for i in range(n_poses) if best_mask[i]])
        b_in = np.concatenate([pose_data[i][1] for i in range(n_poses) if best_mask[i]])
        best_x, *_ = np.linalg.lstsq(A_in, b_in, rcond=None)

    outlier_names = [pose_data[i][2] for i in range(n_poses) if not best_mask[i]]
    if outlier_names:
        print(f"  [RANSAC] Excluded {len(outlier_names)} poses: {', '.join(outlier_names)}")

    resid_norm = float(np.linalg.norm(A_all @ best_x - b_all))
    result = _x_to_result(best_x, A_all, b_all, resid_norm, "ransac", calculator)
    result.method = f"ransac ({best_n_inliers}/{n_poses} poses, thr={inlier_threshold:.1f})"
    return result


def estimate_com(
    calculator: CoMCalculator,
    method: str = "ransac",
    **kwargs,
) -> CoMResult:
    if method == "lsq":
        return calculator.estimate_com()
    elif method == "huber":
        return estimate_huber(calculator, **kwargs)
    elif method == "ransac":
        return estimate_ransac(calculator, **kwargs)
    else:
        raise ValueError(
            f"Unknown method '{method}'. Choose: lsq, huber, ransac"
        )
