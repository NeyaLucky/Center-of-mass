#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
from typing import List, Optional, Callable
from dataclasses import dataclass
import time


@dataclass
class RobotState:
    joint_positions: List[float]
    joint_velocities: List[float]
    joint_efforts: List[float]  
    timestamp: float

_XARM7_JOINT_NAMES = [f"joint{i}" for i in range(1, 8)]

_CURRENT_TO_TORQUE_RATIO = [
    0.275, 
    0.275, 
    0.275, 
    0.161, 
    0.161, 
    0.161, 
    0.050, 
]


class TorqueReader(Node):
    CURRENT_TO_TORQUE_RATIO = _CURRENT_TO_TORQUE_RATIO

    def __init__(
        self,
        node_name: str = 'torque_reader',
        raw_efforts: bool = True,
        joint_prefix: str = '',
    ):
        super().__init__(node_name)

        self._raw_efforts = raw_efforts
        self._joint_names = [joint_prefix + n for n in _XARM7_JOINT_NAMES]

        self._latest_state: Optional[RobotState] = None
        self._state_callback: Optional[Callable[[RobotState], None]] = None

        self._joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10,
        )

        mode = "raw efforts" if raw_efforts else "current→torque conversion"
        self.get_logger().info(
            f"TorqueReader ready ({mode}). Waiting for /joint_states …"
        )

    def _joint_state_callback(self, msg: JointState):
        """Extract joint1–7 data in canonical order from the JointState message."""
        name_to_idx = {n: i for i, n in enumerate(msg.name)}

        positions, velocities, efforts = [], [], []
        for jname in self._joint_names:
            idx = name_to_idx.get(jname)
            if idx is None:
                return
            positions.append(float(msg.position[idx]) if msg.position else 0.0)
            velocities.append(float(msg.velocity[idx]) if msg.velocity else 0.0)
            efforts.append(float(msg.effort[idx]) if msg.effort else 0.0)

        self._latest_state = RobotState(
            joint_positions=positions,
            joint_velocities=velocities,
            joint_efforts=efforts,
            timestamp=time.time(),
        )

        if self._state_callback:
            self._state_callback(self._latest_state)

    def get_current_state(self) -> Optional[RobotState]:
        return self._latest_state

    def get_torques(self, convert_current: Optional[bool] = None) -> Optional[List[float]]:
        if self._latest_state is None:
            return None

        efforts = self._latest_state.joint_efforts

        do_convert = (not self._raw_efforts) if convert_current is None else convert_current
        if do_convert:
            return [e * r for e, r in zip(efforts, _CURRENT_TO_TORQUE_RATIO)]
        return list(efforts)

    def wait_for_state(self, timeout: float = 15.0) -> bool:
        start = time.time()
        while self._latest_state is None:
            if time.time() - start > timeout:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return True

    def set_state_callback(self, callback: Callable[[RobotState], None]):
        self._state_callback = callback


class RobotController(Node):
    def __init__(self, node_name: str = 'robot_controller'):
        super().__init__(node_name)
        self.get_logger().info("RobotController initialized")
    
    def move_to_joints(self, joint_angles: List[float], speed: float = 0.3):

        self.get_logger().info(f"Movement command: {joint_angles}")
        pass


def read_torques_once() -> Optional[List[float]]:
    rclpy.init()
    
    try:
        reader = TorqueReader()
        
        if reader.wait_for_state(timeout=5.0):
            torques = reader.get_torques()
            return torques
        else:
            print("Timeout: failed to receive data from the robot")
            return None
    finally:
        rclpy.shutdown()
