
TEST_POSITIONS = {
    "high_front":  [ 0.0, -1.1,  0.0, 0.7,  0.0,  0.4,  0.0],  
    "elbow_high":  [ 0.0, -0.8,  0.0, 0.9,  0.0,  0.0,  0.0],
    "extend_fwd":  [ 0.0, -0.1,  0.0, 1.2,  0.0,  0.4,  0.0], 
    "side_left":   [ 1.6, -0.4,  0.2, 0.9,  0.0,  0.3,  0.0],  
    "side_right":  [-1.6, -0.4, -0.2, 0.9,  0.0,  0.3,  0.0], 
    "low_fwd":     [ 0.0,  0.3,  0.0, 1.4,  0.0,  0.2,  0.0], 
}

POSITION_DESCRIPTIONS = {
    "high_front":  "Arm raised high, facing forward",
    "elbow_high":  "Elbow folded high, compact",
    "extend_fwd":  "Arm extended forward, medium height",
    "side_left":   "Arm rotated to the left",
    "side_right":  "Arm rotated to the right",
    "low_fwd":     "Arm extended low and forward",
}
_EXTENDED_POSITIONS = {
    "high_left":   [ 1.4, -0.9,  0.0, 0.7,  0.0,  0.3,  0.0],  
    "high_right":  [-1.4, -0.9,  0.0, 0.7,  0.0,  0.3,  0.0],
    "diag_left":   [ 0.8, -0.5,  0.3, 1.0,  0.0,  0.4,  0.0],  
    "diag_right":  [-0.8, -0.5, -0.3, 1.0,  0.0,  0.4,  0.0],  
    "low_left":    [ 1.2,  0.1,  0.3, 1.3,  0.0,  0.2,  0.0], 
    "low_right":   [-1.2,  0.1, -0.3, 1.3,  0.0,  0.2,  0.0], 
    "rear_left":   [ 2.2, -0.5,  0.2, 0.8,  0.0,  0.3,  0.0], 
    "rear_right":  [-2.2, -0.5, -0.2, 0.8,  0.0,  0.3,  0.0],
    "wrist_tilt":  [ 0.0, -0.4,  0.0, 1.0,  0.0,  1.3,  0.0], 
    "roll_pos":    [ 0.5, -0.3,  0.0, 1.0,  1.3,  0.4,  0.0],
    "roll_neg":    [-0.5, -0.3,  0.0, 1.0, -1.3,  0.4,  0.0], 
}

ALL_POSITIONS = {**TEST_POSITIONS, **_EXTENDED_POSITIONS}

HOME_POSE = [0.0, -0.785, 0.0, 0.785, 0.0, 0.0, 0.0]
PRE_APPROACH_POSE = [0.0, 0.10, 0.0, 1.45, 0.0, 0.3, 0.0]
GRASP_POSITION = [0.0, 0.39, 0.0, 1.45, 0.0, 0.3, 0.0]
BOTTLE_COM_WORLD = [0.806, 0.000, 0.203]
BOTTLE_EEF_OFFSET = [0.0, 0.0, 0.10]
BOTTLE_APPROACH_POSE = [0.0, 0.468, 0.0, 1.980, 0.0, 0.752, 0.0]  
BOTTLE_GRASP_POSE    = [0.0, 0.620, 0.0, 1.500, 0.0, 0.120, 0.0] 

def get_positions(mode: str = "noisy") -> dict:
    if mode == "clean":
        return dict(list(ALL_POSITIONS.items())[:6])
    elif mode == "noisy":
        return dict(ALL_POSITIONS)
    else:
        raise ValueError(f"Unknown mode '{mode}'. Choose: clean, noisy")

def get_position(name: str) -> list:
    if name not in ALL_POSITIONS:
        raise ValueError(f"Unknown position: '{name}'. Available: {list(ALL_POSITIONS.keys())}")
    return ALL_POSITIONS[name].copy()


def get_all_positions() -> dict:
    """Backwards-compatible: returns the 6 core TEST_POSITIONS."""
    return TEST_POSITIONS.copy()
