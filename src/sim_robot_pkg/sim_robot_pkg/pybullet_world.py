#!/usr/bin/env python3
import time
from pathlib import Path
import numpy as np

import pybullet as p
import pybullet_data


class XArm7PyBulletWorld:
    OBJECTS = {
        1: "cube_1.urdf",
        2: "cube_2.urdf",
        3: "parallelepipede.urdf",
        4: "cube_1_heavy.urdf",
        5: "cube_2_heavy.urdf",
        6: "cube_1_large.urdf",
        7: "cube_2_large.urdf",
        8: "parallelepipede_large.urdf",
        9: "bottle_mybottle.urdf",
        10: "bottle_mybottle_heavy.urdf",
    }

    OBJECT_NAMES = {
        1: "Cube small 0.5 kg (CoM centered)",
        2: "Cube small 0.5 kg (CoM offset)",
        3: "Parallelepiped 1.23 kg (CoM offset)",
        4: "Cube small 2.5 kg (CoM centered)",
        5: "Cube small 2.5 kg (CoM offset)",
        6: "Cube large 2.5 kg (CoM centered)",
        7: "Cube large 2.5 kg (CoM offset)",
        8: "Parallelepiped large 2.5 kg (CoM offset)",
        9: "Bottle MyBottle 2.0 kg (CoM centered)",
        10: "Bottle MyBottle 2.0 kg (CoM bottom-heavy)",
    }

    OBJECT_SLUGS = {
        1: "cube_small_0.5kg_com_centered",
        2: "cube_small_0.5kg_com_offset",
        3: "parallelepiped_1.23kg_com_offset",
        4: "cube_small_2.5kg_com_centered",
        5: "cube_small_2.5kg_com_offset",
        6: "cube_large_2.5kg_com_centered",
        7: "cube_large_2.5kg_com_offset",
        8: "parallelepiped_large_2.5kg_com_offset",
        9: "bottle_mybottle_2kg_com_centered",
        10: "bottle_mybottle_2kg_com_bottom",
    }

    OBJECT_ATTACHMENT_OFFSETS = {
        9:  [0.0, 0.0, 0.10],   
        10: [0.0, 0.0, 0.14],   
    }

    HEAVY_OBJECTS = {4, 5}
    LARGE_OBJECTS = {6, 7, 8}
    LIGHT_OBJECTS = {1, 2}

    CUBES = OBJECTS
    
    def __init__(self, cube_choice: int = 0, gui: bool = True):
        if gui:
            self.client = p.connect(p.GUI, options='--opengl2 --width=1024 --height=768')
            p.resetDebugVisualizerCamera(
                cameraDistance=1.5,
                cameraYaw=45,
                cameraPitch=-30,
                cameraTargetPosition=[0, 0, 0.5]
            )
        else:
            self.client = p.connect(p.DIRECT)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        possible_paths = [
            Path(__file__).resolve().parent.parent / "assets" / "models",
            Path("/root/ros2/ws/src/assets/models"),
        ]
        
        self.assets_path = None
        for path in possible_paths:
            if path.exists() and (
                (path / "xarm7_with_gripper.urdf").exists()
                or (path / "xarm7.urdf").exists()
            ):
                self.assets_path = path
                break
        
        if self.assets_path is None:
            raise FileNotFoundError(f"Assets not found in any of: {possible_paths}")

        print(f"Using assets from: {self.assets_path}")
        
        self.plane_id = p.loadURDF("plane.urdf")
        
        self.robot_id = self._load_robot()
        
        self.joint_indices = self._get_joint_indices()
        self.all_movable_indices = self._get_all_movable_joint_indices()
        self.eef_link_index = self._get_eef_link_index()
        
        self.cube_id = None
        self.cube_constraint = None
        self.current_cube_choice = 0
        self.standing_object_id = None

        if cube_choice > 0 and cube_choice in self.CUBES:
            self.attach_cube(cube_choice)
    
    def _load_robot(self) -> int:
        urdf_path = self.assets_path / "xarm7_with_gripper.urdf"
        if not urdf_path.exists():
            urdf_path = self.assets_path / "xarm7.urdf"

        if not urdf_path.exists():
            raise FileNotFoundError(f"URDF not found: {urdf_path}")

        p.setAdditionalSearchPath(str(self.assets_path))
        
        robot_id = p.loadURDF(
            str(urdf_path),
            basePosition=[0, 0, 0],
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
            useFixedBase=True
        )
        
        return robot_id
    
    def _get_joint_indices(self) -> list:
        joint_indices = []
        num_joints = p.getNumJoints(self.robot_id)
        
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode('utf-8')
            joint_type = joint_info[2]
            
            if joint_type == p.JOINT_REVOLUTE and joint_name.startswith('joint') and joint_name[5:].isdigit():
                joint_num = int(joint_name[5:])
                if 1 <= joint_num <= 7:
                    joint_indices.append((joint_num, i))
        
        joint_indices.sort(key=lambda x: x[0])
        return [idx for _, idx in joint_indices]

    def _get_all_movable_joint_indices(self) -> list:
        indices = []
        for i in range(p.getNumJoints(self.robot_id)):
            jtype = p.getJointInfo(self.robot_id, i)[2]
            if jtype != p.JOINT_FIXED:
                indices.append(i)
        return indices

    def _full_positions(self, arm_positions: list) -> list:
        pos_map = {idx: arm_positions[k] for k, idx in enumerate(self.joint_indices)}
        return [pos_map.get(i, 0.0) for i in self.all_movable_indices]

    def _get_eef_link_index(self) -> int:
        num_joints = p.getNumJoints(self.robot_id)
        fallback = None
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            link_name = joint_info[12].decode('utf-8')
            if link_name == 'link_tcp':
                return i
            if link_name == 'link_eef':
                fallback = i
        if fallback is not None:
            return fallback
        print("link_tcp / link_eef not found, using last joint")
        return self.joint_indices[-1] if self.joint_indices else num_joints - 1
    
    def attach_cube(self, cube_choice: int):
        self.detach_cube()
        
        if cube_choice not in self.OBJECTS or cube_choice == 0:
            return
        
        cube_urdf = self.OBJECTS[cube_choice]
        if cube_urdf is None:
            return
            
        cube_path = self.assets_path / cube_urdf
        
        if not cube_path.exists():
            print(f"URDF cube not found: {cube_path}")
            return
        
        eef_state = p.getLinkState(self.robot_id, self.eef_link_index)
        eef_pos = eef_state[0]
        eef_orn = eef_state[1]
        
        cube_offset = list(self.OBJECT_ATTACHMENT_OFFSETS.get(cube_choice, [0.0, 0.0, 0.0]))
        self._cube_attachment_offset = cube_offset

        rot_matrix_pre = np.array(p.getMatrixFromQuaternion(eef_orn)).reshape(3, 3)
        com_world_pre = rot_matrix_pre @ np.array(cube_offset)
        self.cube_id = p.loadURDF(
            str(cube_path),
            basePosition=[eef_pos[0] + com_world_pre[0],
                          eef_pos[1] + com_world_pre[1],
                          eef_pos[2] + com_world_pre[2]],
            baseOrientation=eef_orn
        )

        self.cube_constraint = p.createConstraint(
            parentBodyUniqueId=self.robot_id,
            parentLinkIndex=self.eef_link_index,
            childBodyUniqueId=self.cube_id,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=cube_offset,
            childFramePosition=[0, 0, 0],
            parentFrameOrientation=[0, 0, 0, 1],
            childFrameOrientation=[0, 0, 0, 1]
        )
        
        self.current_cube_choice = cube_choice
        
        dynamics_info = p.getDynamicsInfo(self.cube_id, -1)
        mass = dynamics_info[0]
        com = dynamics_info[3]
        object_name = self.OBJECT_NAMES.get(cube_choice,
                          cube_urdf.replace('.urdf', '').replace('_', ' ').title())

        print(f"Object's mass: {mass:.3f} kg")
        print(f"Center of mass (local): [{com[0]:.3f}, {com[1]:.3f}, {com[2]:.3f}]")
        print(f"Object '{object_name}' attached")
    
    def get_cube_attachment_offset(self) -> tuple:
        return getattr(self, '_cube_attachment_offset', (0, 0, 0))
    
    def get_attached_object_info(self) -> dict:
        if self.cube_id is None or self.current_cube_choice == 0:
            return None
        
        urdf_file = self.OBJECTS[self.current_cube_choice]
        object_name = self.OBJECT_NAMES.get(self.current_cube_choice,
                          urdf_file.replace('.urdf', '').replace('_', ' ').title())
        
        try:
            collision_data = p.getCollisionShapeData(self.cube_id, -1)
            if collision_data and collision_data[0][2] == p.GEOM_BOX:
                dims = collision_data[0][3]
                size = (dims[0], dims[1], dims[2])
            else:
                aabb_min, aabb_max = p.getAABB(self.cube_id, -1)
                size = (
                    aabb_max[0] - aabb_min[0],
                    aabb_max[1] - aabb_min[1],
                    aabb_max[2] - aabb_min[2]
                )
        except:
            size = (0.05, 0.05, 0.05)

        dynamics_info = p.getDynamicsInfo(self.cube_id, -1)
        mass = dynamics_info[0]
        actual_com = dynamics_info[3] 
        
        try:
            visual_data = p.getVisualShapeData(self.cube_id)
            if visual_data and len(visual_data) > 0:
                color = visual_data[0][7]
                mesh_filename = visual_data[0][4].decode('utf-8') if visual_data[0][4] else ''
            else:
                color = (0.5, 0.5, 0.5, 1.0)
                mesh_filename = ''
        except:
            color = (0.5, 0.5, 0.5, 1.0)
            mesh_filename = ''
        
        slug = self.OBJECT_SLUGS.get(self.current_cube_choice,
                   urdf_file.replace('.urdf', ''))
        return {
            'name': object_name,
            'slug': slug,
            'size': size,
            'position': self.get_cube_attachment_offset(),
            'urdf_file': urdf_file,
            'mass': mass,
            'color': color,
            'actual_com': actual_com,
            'mesh_filename': mesh_filename,
        }
    
    def detach_cube(self):
        if self.cube_constraint is not None:
            p.removeConstraint(self.cube_constraint)
            self.cube_constraint = None

        if self.cube_id is not None:
            p.removeBody(self.cube_id)
            self.cube_id = None

        self.current_cube_choice = 0

    def place_standing_object(self, object_choice: int, world_com_pos: list):
        """Load object free-standing at world_com_pos (CoM frame).  No physics."""
        self.remove_standing_object()
        if object_choice not in self.OBJECTS:
            return
        urdf_file = self.OBJECTS[object_choice]
        urdf_path = self.assets_path / urdf_file
        if not urdf_path.exists():
            return
        self.standing_object_id = p.loadURDF(
            str(urdf_path),
            basePosition=world_com_pos,
            baseOrientation=[0, 0, 0, 1],
        )
        p.changeDynamics(self.standing_object_id, -1,
                         linearDamping=100, angularDamping=100)

    def remove_standing_object(self):
        if self.standing_object_id is not None:
            p.removeBody(self.standing_object_id)
            self.standing_object_id = None

    def set_joint_positions(self, positions: list):
        if len(positions) != len(self.joint_indices):
            raise ValueError(f"Waiting {len(self.joint_indices)} positions, got {len(positions)}")

        for i, joint_idx in enumerate(self.joint_indices):
            p.resetJointState(self.robot_id, joint_idx, positions[i])

        if self.cube_id is not None:
            p.stepSimulation()

    def move_to_positions(self, positions: list, steps: int = 80, dt: float = 0.04):
        if len(positions) != len(self.joint_indices):
            raise ValueError(f"Waiting {len(self.joint_indices)} positions, got {len(positions)}")

        current = self.get_joint_positions()
        for step in range(1, steps + 1):
            alpha = step / steps
            interp = [c + alpha * (t - c) for c, t in zip(current, positions)]
            for i, joint_idx in enumerate(self.joint_indices):
                p.resetJointState(self.robot_id, joint_idx, interp[i])
            if self.cube_id is not None:
                p.stepSimulation()
            time.sleep(dt)
    
    def get_joint_positions(self) -> list:
        positions = []
        for joint_idx in self.joint_indices:
            state = p.getJointState(self.robot_id, joint_idx)
            positions.append(state[0])
        return positions
    
    def get_eef_pose(self) -> tuple:
        state = p.getLinkState(self.robot_id, self.eef_link_index)
        return state[0], state[1]
    
    def compute_eef_jacobian(self, positions: list = None) -> tuple:
        if positions is None:
            positions = self.get_joint_positions()

        full_pos = self._full_positions(positions)
        zero_vec = [0.0] * len(full_pos)

        jac_t, jac_r = p.calculateJacobian(
            self.robot_id,
            self.eef_link_index,
            [0, 0, 0],
            full_pos,
            zero_vec,
            zero_vec
        )

        # Return only arm-joint columns
        arm_cols = [self.all_movable_indices.index(i) for i in self.joint_indices]
        jac_t_arr = np.array(jac_t)[:, arm_cols]
        jac_r_arr = np.array(jac_r)[:, arm_cols]

        state = p.getLinkState(self.robot_id, self.eef_link_index)
        quat = state[1]
        R_eef = np.array(p.getMatrixFromQuaternion(quat)).reshape(3, 3)

        return jac_t_arr, jac_r_arr, R_eef

    def compute_gravity_torques(self, positions: list = None) -> list:
        if positions is None:
            positions = self.get_joint_positions()

        full_pos = self._full_positions(positions)
        zero_full = [0.0] * len(full_pos)

        all_torques = list(p.calculateInverseDynamics(
            self.robot_id,
            full_pos,
            zero_full,
            zero_full
        ))

        arm_cols = [self.all_movable_indices.index(i) for i in self.joint_indices]
        robot_torques = [all_torques[k] for k in arm_cols]

        if self.cube_id is None:
            return robot_torques

        cube_dynamics = p.getDynamicsInfo(self.cube_id, -1)
        cube_mass = cube_dynamics[0]
        cube_local_com = cube_dynamics[3]

        cube_offset = self.get_cube_attachment_offset()
        com_in_eef = [
            cube_offset[0] + cube_local_com[0],
            cube_offset[1] + cube_local_com[1],
            cube_offset[2] + cube_local_com[2]
        ]

        jac_t, _ = p.calculateJacobian(
            self.robot_id,
            self.eef_link_index,
            com_in_eef,
            full_pos,
            zero_full,
            zero_full
        )

        jac_t = np.array(jac_t)[:, arm_cols]
        gravity_force = np.array([0.0, 0.0, -cube_mass * 9.81])

        object_torques = jac_t.T @ gravity_force

        total = [r + o for r, o in zip(robot_torques, object_torques)]
        return total
    
    def step_simulation(self, steps: int = 1):
        for _ in range(steps):
            p.stepSimulation()
    
    def run_realtime(self):
        p.setRealTimeSimulation(1)
    
    def stop_realtime(self):
        p.setRealTimeSimulation(0)
    
    def disconnect(self):
        self.detach_cube()
        self.remove_standing_object()
        p.disconnect(self.client)
