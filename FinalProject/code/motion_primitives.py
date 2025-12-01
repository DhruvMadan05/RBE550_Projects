"""Motion primitives that ground high-level Blocksworld actions.

Each primitive follows a simple template:
1. Compute end-effector poses using IK (top-down grasps).
2. Use the OMPL-based planner to reach those poses collision-free.
3. Execute the trajectory and actuate the gripper.

The primitives are intentionally conservative (top-down only) but provide a
baseline that you can refine with better grasp synthesis or control.
"""
from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Dict, Iterable, Optional, Sequence

import numpy as np
import torch

from planning import PlannerInterface

from genesis.utils.misc import tensor_to_array

import matplotlib.pyplot as plt


DEFAULT_HAND_QUAT = np.array([0.0, 1.0, 0.0, 0.0])


class MotionPrimitiveError(RuntimeError):
    """Raised when a primitive cannot be completed."""


@dataclass
class PrimitiveConfig:
    hover_height: float = 0.2 # Height above the block the gripper will hover at
    hover_height_stacking: float = 0.1 # Height above the top block when stacking
    grasp_clearance: float = 0.1 # addition clearance above the block for grasping
    grasp_x_offset: float = 0.01 # offset in x direction for grasping
    gripper_opening: float = 0.039 # gripper open position
    gripper_closed: float = 0.01 # gripper closed position
    motion_waypoints: int = 200
    settle_tolerance: float = 1e-3
    settle_timeout: float = 1.0
    gripper_steps: int = 60
    squeeze_force: float = 20.0

class MotionPrimitiveExecutor:
    """Executes pick/place/stack/unstack primitives via OMPL plans."""

    def __init__(self, scene, franka, blocks_state: Dict[str, any], config: PrimitiveConfig | None = None):
        self.scene = scene
        self.robot = franka
        self.blocks_state = blocks_state
        self.config = config or PrimitiveConfig()
        self.hand_link = self.robot.get_link("hand")
        self.held_block: Optional[str] = None
        self.qpos_history = []
        self.gripper_closed = False

        self.planner = PlannerInterface(self.robot, self.scene)

    # ------------------------------------------------------------------ public API
    def pick(self, block_name: str) -> None:
        block = self._require_block(block_name)
        self._open_gripper()

        print("Current Pos")
        print(self._current_qpos())
        print("Gripper Opened")
        hover = self._hover_pose(block)
        grasp = self._grasp_pose(block)
        self._move_hand(hover)
        print("Moved to hover pose")
        # While the gripper is not at the grasp pose, wait for it until it reachs the desired position
        
        self._move_hand(grasp)
        print("Moved to grasp pose")

        
        self.held_block = block_name
        self._close_gripper(attached_object=block)
        self.gripper_closed = True
        print("Gripper Closed")

        print("current robot pos in world coordinates:")
        hand_pos = np.asarray(self.hand_link.get_pos(), dtype=float)
        print(hand_pos)
        
        self._move_hand(hover, attached_object=block)
        print("Lifted block")

        # # print qpos history to file
        # with open("qpos_history.txt", "w") as f:
        #     for qpos in self.qpos_history:
        #         f.write(", ".join(f"{v:.6f}" for v in qpos) + "\n")

    def putdown(self, block_name: str, target_xy: Optional[Sequence[float]] = None) -> None:
        if self.held_block != block_name:
            raise MotionPrimitiveError(f"Robot is not holding {block_name}")
        target_pos = np.array(self.blocks_state[block_name].get_pos(), dtype=float)
        if target_xy is not None:
            target_pos[0] = target_xy[0]
            target_pos[1] = target_xy[1]

        place = self._place_pose_on_table(target_pos)

        hover = place + np.array([0.0, 0.0, self.config.hover_height])
        block_entity = self.blocks_state[block_name]

        self._move_hand(hover, attached_object=block_entity)
        self._move_hand(place, attached_object=block_entity)
        self._open_gripper(attached_object=block_entity)
        self.gripper_closed = False
        self.held_block = None
        self._move_hand(hover)

    def stack(self, top_block: str, bottom_block: str) -> None:
        if self.held_block != top_block:
            raise MotionPrimitiveError(f"Robot must hold {top_block} before stacking.")
        support = self._require_block(bottom_block)
        target = np.array(support.get_pos(), dtype=float)
        target[2] += self._block_height()
        place = self._place_pose_top(target)
        hover = place + np.array([0.0, 0.0, self.config.hover_height_stacking])
        block_entity = self.blocks_state[top_block]

        # print bottom block position
        print(f"Bottom block '{bottom_block}' position: {support.get_pos()}")
        # print target hover and place positions
        print(f"Hover position for stacking: {hover}")
        print(f"Place position for stacking: {place}")

        self._move_hand(hover, attached_object=block_entity)
        
        self._move_hand(place, attached_object=block_entity)
        self._open_gripper(attached_object=block_entity)
        self.gripper_closed = False
        self.held_block = None
        self._move_hand(hover)

    def unstack(self, top_block: str, bottom_block: str) -> None:
        block = self._require_block(top_block)
        # same as pick but we expect ON relation already satisfied
        self.pick(top_block)

    # ------------------------------------------------------------------ helpers
    def _hover_pose(self, block_entity) -> np.ndarray:
        pos = np.array(block_entity.get_pos(), dtype=float)
        hover = pos.copy()
        hover[2] = pos[2] + self.config.hover_height
        hover[0] += self.config.grasp_x_offset  # offset in x direction for better hovering
        print(f"Hover pose: {hover}")
        return hover

    def _grasp_pose(self, block_entity) -> np.ndarray:
        pos = np.array(block_entity.get_pos(), dtype=float)
        grasp = pos.copy()
        grasp[2] = pos[2] + (self._block_height() / 2.0) + self.config.grasp_clearance
        grasp[0] += self.config.grasp_x_offset  # offset in x direction for better grasping
        print(f"Grasp pose: {grasp}")
        return grasp

    def _place_pose_on_table(self, target_pos: np.ndarray) -> np.ndarray:
        place = target_pos.copy()
        place[2] = self._block_height() + self.config.grasp_clearance

        # set 6 different locations in the area for placing the block. Check if any other block is already there
        # if so pick a different one of the locations to place the block
        # predefined locations (x, y offsets)
        offsets = [(0.65, 0.0), (0.65, 0.2), (0.65, 0.4), (0.45, 0.0), (0.45, 0.2), (0.45, 0.4)]
        occupied_positions = []
        for bname, bentity in self.blocks_state.items():
            if bname != self.held_block:
                # Ensure positions are numpy arrays (convert from torch.Tensor if needed)
                bpos = bentity.get_pos()
                try:
                    bpos = tensor_to_array(bpos)
                except Exception:
                    bpos = np.asarray(bpos, dtype=float)
                occupied_positions.append(np.asarray(bpos[:2], dtype=float))
        
        for offset in offsets:
            candidate_pos = target_pos.copy()
            candidate_pos[0] = offset[0]
            candidate_pos[1] = offset[1]
            # check if candidate pos is occupied
            is_occupied = False
            for occ in occupied_positions:
                if np.linalg.norm(candidate_pos[:2] - occ) < 0.1:  # 7.5 cm threshold
                    is_occupied = True
                    break
            if not is_occupied:
                place[0] = candidate_pos[0]
                place[1] = candidate_pos[1]
                break

        return place

    def _place_pose_top(self, target_pos: np.ndarray) -> np.ndarray:
        place = target_pos.copy()
        place[2] += (self._block_height() / 2.0) + self.config.grasp_clearance
        place[0] += 0.005  # offset in x direction for better placing
        place[1] += 0.002
        return place

    def _move_hand(self, pos: np.ndarray, quat: Optional[np.ndarray] = None, attached_object=None) -> None:
        quat = quat if quat is not None else DEFAULT_HAND_QUAT
        qpos_goal = self.robot.inverse_kinematics(
            link=self.hand_link,
            pos=pos,
            quat=quat,
        )

        print("Planning path to:")
        print(qpos_goal)
        # print position in world coordinates
        print("Target world position:")
        print(pos)

        # current = self._current_qpos()
        # if isinstance(qpos_goal, torch.Tensor):
        #     qpos_goal = qpos_goal.clone()
        #     qpos_goal[-2:] = torch.as_tensor(current[-2:], dtype=qpos_goal.dtype, device=qpos_goal.device)

        # else:
        # qpos_goal = np.asarray(qpos_goal, dtype=float)
        if self.gripper_closed:
            qpos_goal[-2:] = self.config.gripper_closed
        else:
            qpos_goal[-2:] = self.config.gripper_opening


        # make sure values are clamped before calling plan path

        # clamp the qpos_goal within robot limits
        q_lower = np.asarray(self.robot.q_limit[0], dtype=float)
        q_upper = np.asarray(self.robot.q_limit[1], dtype=float)

        qpos_goal = np.clip(qpos_goal, q_lower, q_upper)

        path = self.planner.plan_path(
            qpos_goal=qpos_goal,
            num_waypoints=self.config.motion_waypoints,
            attached_object=attached_object,
        )
        waypoints = self._normalize_waypoints(path)

        # check that waypoints is not just all zeros
        
        # while all(np.allclose(tensor_to_array(wp), 0.0) for wp in waypoints):
        #     print("Received all-zero waypoints, replanning...")
        #     path = self.planner.plan_path(
        #         qpos_goal=qpos_goal,
        #         num_waypoints=self.config.motion_waypoints,
        #         attached_object=attached_object,
        #     )
        #     waypoints = self._normalize_waypoints(path)

        # dump waypoints to a file for debugging
        with open("planned_path.txt", "w") as f:
            for waypoint in waypoints:
                f.write(", ".join(f"{v:.6f}" for v in tensor_to_array(waypoint)) + "\n")  

        if not waypoints:
            raise MotionPrimitiveError("Motion planner failed to find a path.")
        self._execute_path(waypoints)
        self._wait_until_qpos(qpos_goal)

    def _execute_path(self, path: Iterable) -> None:
        for waypoint in path:
            # for each waypoint tensor, set the gripper position to closed or opened based on current state
            if self.gripper_closed:
                waypoint[-2:] = self.config.gripper_closed
            else:
                waypoint[-2:] = self.config.gripper_opening

            self.robot.control_dofs_position(waypoint)
            self._record_qpos()
            self.scene.step()

    def _open_gripper(self, attached_object=None) -> None:
        self._set_gripper(self.config.gripper_opening, attached_object=attached_object)

    def _close_gripper(self, attached_object=None) -> None:
        self._set_gripper(self.config.gripper_closed, attached_object=attached_object)

    def _set_gripper(self, value: float, attached_object=None) -> None:
        current = self._current_qpos()
        target = current.copy()
        target[-2:] = value
        #steps = max(1, self.config.gripper_steps)
        path = self.planner.plan_path(
            qpos_goal=target,
            num_waypoints=self.config.gripper_steps,
            attached_object=attached_object
        )
        # execute the planned path
        for waypoint in path:
            self.robot.control_dofs_position(waypoint)
            self.scene.step()
            self._record_qpos()
        self._wait_until_qpos(target)

    def _current_qpos(self) -> np.ndarray:
        qpos = self.robot.get_qpos()
        if hasattr(qpos, "detach"):
            return tensor_to_array(qpos)
        return np.asarray(qpos, dtype=float)

    def _require_block(self, name: str):
        if name not in self.blocks_state:
            raise MotionPrimitiveError(f"Unknown block '{name}'")
        return self.blocks_state[name]

    def _block_height(self) -> float:
        # assumes cubes (size pulled from scenes.py)
        return 0.04

    def _normalize_waypoints(self, path) -> list:
        """Ensure we always iterate over a list of tensors."""
        if path is None:
            return []
        if isinstance(path, torch.Tensor):
            if path.ndim <= 1:
                return [path]
            return [path[i] for i in range(path.shape[0])]
        return list(path)

    def _wait_until_qpos(self, target_qpos) -> None:
        """Wait until the joints settle near the final configuration."""
        target = tensor_to_array(target_qpos)
        deadline = time.time() + self.config.settle_timeout
        while time.time() < deadline:
            cur = self._current_qpos()
            if np.linalg.norm(cur - target) <= self.config.settle_tolerance:
                break
            self.scene.step()
    
    def _record_qpos(self):
        self.qpos_history.append(self._current_qpos().copy())

    def _squeeze_gripper(self, force_newtons=5.0):
        # torque = np.zeros_like(self._current_qpos())
        # torque[-2:] = -abs(force_newtons)   # negative closes the fingers on the Panda
        # self.robot.control_dofs_force(torque)

        finger_force = np.array([-abs(force_newtons), -abs(force_newtons)], dtype=float)
        self.robot.control_dofs_force(
            finger_force,
            dofs_idx=[self.robot.n_qs - 2, self.robot.n_qs - 1],
        )

    def pause(self):
        while True:
            self.scene.step()
            time.sleep(0.01)   # ~100 Hz viewer updates
        