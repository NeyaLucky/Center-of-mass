#!/usr/bin/env python3
import time
from typing import List

import rclpy
from rclpy.node import Node

from xarm_msgs.srv import PlanJoint, PlanExec, GripperMove, SetInt16, SetInt16ById, Call

from com_estimation_pkg.positions import HOME_POSE, BOTTLE_APPROACH_POSE, BOTTLE_GRASP_POSE

GRIPPER_OPEN        = 850   
GRIPPER_BOTTLE_GRIP = 480  


class ArmMover(Node):
    _PLAN_TIMEOUT = 5.0    
    _EXEC_TIMEOUT = 30.0   
    _GRIP_TIMEOUT = 10.0 

    def __init__(self, node_name: str = "arm_mover"):
        super().__init__(node_name)
        self._gripper_ready = False 
        self._plan         = self.create_client(PlanJoint,   "xarm_joint_plan")
        self._exec         = self.create_client(PlanExec,    "xarm_exec_plan")
        self._grip         = self.create_client(GripperMove, "/xarm/set_gripper_position")
        self._grip_mode    = self.create_client(SetInt16,    "/xarm/set_gripper_mode")
        self._grip_enable  = self.create_client(SetInt16,    "/xarm/set_gripper_enable")
        self._clean_error        = self.create_client(Call,         "/xarm/clean_error")
        self._clean_gripper_err  = self.create_client(Call,         "/xarm/clean_gripper_error")
        self._set_mode           = self.create_client(SetInt16,     "/xarm/set_mode")
        self._set_state          = self.create_client(SetInt16,     "/xarm/set_state")
        self._motion_enable      = self.create_client(SetInt16ById, "/xarm/motion_enable")

    def wait_for_services(self, timeout: float = 20.0) -> bool:
        deadline = time.time() + timeout
        for client, name in (
            (self._plan,        "xarm_joint_plan"),
            (self._exec,        "xarm_exec_plan"),
            (self._grip,        "/xarm/set_gripper_position"),
            (self._grip_mode,   "/xarm/set_gripper_mode"),
            (self._grip_enable, "/xarm/set_gripper_enable"),
        ):
            remaining = max(0.0, deadline - time.time())
            if not client.wait_for_service(timeout_sec=remaining):
                self.get_logger().error(f"Service not available: {name}")
                return False
        return True

    def reset_robot(self) -> bool:
        future = self._clean_error.call_async(Call.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        time.sleep(0.2)

        req = SetInt16ById.Request(); req.id = 8; req.data = 1
        future = self._motion_enable.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        time.sleep(0.3)

        self.get_logger().info("Robot reset: errors cleared, motion enabled.")
        return True

    def init_gripper(self) -> bool:
        f = self._clean_gripper_err.call_async(Call.Request())
        rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
        time.sleep(0.2)

        for client, val, label in (
            (self._grip_mode,   0, "gripper mode→position"),
            (self._grip_enable, 1, "gripper enable"),
        ):
            req = SetInt16.Request()
            req.data = val
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if not future.done():
                self.get_logger().error(f"Timeout: {label}")
                return False
            ret = future.result().ret
            self.get_logger().info(f"{label} → ret={ret}")
        time.sleep(2.0)
        self._gripper_ready = True
        return True

    def open_gripper(self) -> bool:
        if not self._gripper_ready:
            if not self.init_gripper():
                return False
        return self._set_gripper(GRIPPER_OPEN)

    def move_to_joints(self, q: List[float]) -> bool:
        plan_req = PlanJoint.Request()
        plan_req.target = [float(v) for v in q]

        future = self._plan.call_async(plan_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._PLAN_TIMEOUT)
        if not future.done() or not future.result().success:
            self.get_logger().error(
                f"Planning failed for q={[f'{v:.3f}' for v in q]}"
            )
            return False

        exec_req = PlanExec.Request()
        exec_req.wait = True
        future = self._exec.call_async(exec_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._EXEC_TIMEOUT)
        if not future.done() or not future.result().success:
            self.get_logger().error("Execution failed.")
            return False

        return True

    def move_home(self) -> bool:
        return self.move_to_joints(HOME_POSE)
    
    def _set_gripper(self, pos: int) -> bool:
        self.get_logger().info(f"Gripper → pos={pos}")
        req = GripperMove.Request()
        req.pos     = float(pos)
        req.wait    = True
        req.timeout = 5.0
        future = self._grip.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._GRIP_TIMEOUT)
        if not future.done():
            self.get_logger().error("Gripper service timeout.")
            return False
        ret = future.result().ret
        self.get_logger().info(f"Gripper pos={pos} → ret={ret}")
        if ret != 0:
            self.get_logger().warning(f"Gripper ret={ret} (non-zero).")
        return ret == 0

    def close_to_bottle(self) -> bool:
        return self._set_gripper(GRIPPER_BOTTLE_GRIP)

    def pick_bottle(self) -> bool:
        print("  [PICK] Moving to approach pose …")
        if not self.move_to_joints(BOTTLE_APPROACH_POSE):
            return False

        print("  [PICK] Opening gripper …")
        self.open_gripper()
        time.sleep(0.5)

        print("  [PICK] Descending to bottle …")
        if not self.move_to_joints(BOTTLE_GRASP_POSE):
            return False

        print("  [PICK] Closing gripper …")
        self.close_to_bottle()
        time.sleep(1.0)

        print("  [PICK] Lifting up …")
        if not self.move_to_joints(BOTTLE_APPROACH_POSE):
            return False

        return True
