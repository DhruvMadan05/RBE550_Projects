from ast import parse
import sys
import numpy as np
import genesis as gs
from scenes import create_scene_3_blocks, create_scene_8_blocks
from symbolic import lift_scene, BLOCK_SIZE, NEXT_TO_CENTER_LATERAL

from motion_primitives import MotionPrimitiveError, MotionPrimitiveExecutor

from task_planner import plan_blocksworld, goal_pyramid_with_cap

PLACEMENT_BUFFER = 0.005

# Ensure Genesis is initialized before building scenes
if len(sys.argv) > 1 and sys.argv[1] == "gpu":
    gs.init(backend=gs.gpu, logging_level='Warning', logger_verbose_time=False)
else:
    gs.init(backend=gs.cpu, logging_level='Warning', logger_verbose_time=False)

# build the scene using the factory
scene, franka, BlocksState = create_scene_8_blocks()

# set control gains
# Note: the following values are tuned for achieving best behavior with Franka
# Typically, each new robot would have a different set of parameters.
# Sometimes high-quality URDF or XML file would also provide this and will be parsed.

franka.set_dofs_kp(
    np.array([4500, 4500, 3500, 3500, 2000, 2000, 2000, 100, 100]),
)
franka.set_dofs_kv(
    np.array([450, 450, 350, 350, 200, 200, 200, 10, 10]),
)
franka.set_dofs_force_range(
    np.array([-87, -87, -87, -87, -12, -12, -12, -100, -100]),
    np.array([87, 87, 87, 87, 12, 12, 12, 100, 100]),
)

# move to a fixed pre-grasp pose
# qpos = franka.inverse_kinematics(
#     link=franka.get_link("hand"),
#     pos=np.array([0.65, 0.0, 0.25]),
#     quat=np.array([0, 1, 0, 0]),
# )
# # gripper open pos
# qpos[-2:] = 0.04
# path = franka.plan_path(
#     qpos_goal=qpos,
#     num_waypoints=200,  # 2s duration
# )

# # execute the planned path
# for waypoint in path:
#     franka.control_dofs_position(waypoint)
#     scene.step()

predicates = lift_scene(franka, BlocksState)
for atom in predicates.as_pddl_atoms():
    print(atom)

goal = goal_pyramid_with_cap()
actions = plan_blocksworld(predicates, goal)
# print("Planned actions:")
# for action in actions:
#     print(action)

# Print all block loactions
for block_name, block_entity in BlocksState.items():
    print(f"Block {block_name} position: {block_entity.get_pos()}")

executor = MotionPrimitiveExecutor(scene, franka, BlocksState)
# executor.pick("r")
# executor.stack("r", "g")  # assumes you’re holding r


def _adjacent_xy(blocks_state, reference: str, held: str) -> np.ndarray:
    """Pick an XY target next to the reference block."""

    ref_pos = np.asarray(blocks_state[reference].get_pos(), dtype=float)
    spacing = BLOCK_SIZE + PLACEMENT_BUFFER
    offsets = [
        np.array([BLOCK_SIZE, 0.0]),
        np.array([-BLOCK_SIZE, 0.0]),
        np.array([0.0, spacing]),
        np.array([0.0, -spacing]),
    ]
    others = [
        np.asarray(entity.get_pos(), dtype=float)[:2]
        for name, entity in blocks_state.items()
        if name not in (reference, held)
    ]
    best_score = -float("inf")
    best_xy = ref_pos[:2] + offsets[0]
    for offset in offsets:
        candidate = ref_pos[:2] + offset
        if not others:
            return candidate
        min_dist = min(np.linalg.norm(candidate - other) for other in others)
        if min_dist > best_score:
            best_score = min_dist
            best_xy = candidate
    return best_xy


def _center_xy(blocks_state, first: str, second: str, held: str) -> np.ndarray:
    """Return an XY target centered between the base pair but offset laterally."""

    pos_a = np.asarray(blocks_state[first].get_pos(), dtype=float)
    pos_b = np.asarray(blocks_state[second].get_pos(), dtype=float)
    midpoint = 0.5 * (pos_a[:2] + pos_b[:2])
    base_vec = pos_b[:2] - pos_a[:2]
    base_norm = np.linalg.norm(base_vec)
    if base_norm < 1e-6:
        base_dir = np.array([0.0, 1.0])
    else:
        base_dir = base_vec / base_norm
    perp_vec = np.array([-base_dir[1], base_dir[0]])
    if np.linalg.norm(perp_vec) < 1e-6:
        perp_vec = np.array([1.0, 0.0])

    offset_dist = NEXT_TO_CENTER_LATERAL + PLACEMENT_BUFFER
    others = [
        np.asarray(entity.get_pos(), dtype=float)[:2]
        for name, entity in blocks_state.items()
        if name not in (first, second, held)
    ]
    best_xy = midpoint + perp_vec * offset_dist
    best_score = -float("inf")
    for direction in (perp_vec, -perp_vec):
        candidate = midpoint + direction * offset_dist
        if not others:
            return candidate
        min_dist = min(np.linalg.norm(candidate - other) for other in others)
        if min_dist > best_score:
            best_score = min_dist
            best_xy = candidate
    return best_xy

def execute_action(action, executor, sym):
    name, args = parse_action(action)
    if name == "pick-up":
        try:
            executor.pick(args[0])
            return True
        except MotionPrimitiveError as exc:
            print(exc)
            return False
    elif name == "put-down":
        try:
            executor.putdown(args[0])
            return True
        except MotionPrimitiveError as exc:
            print(exc)
            return False
    elif name == "stack":
        try:
            executor.stack(args[0], args[1])
            return True
        except MotionPrimitiveError as exc:
            print(exc)
            return False
    elif name == "unstack":
        try:
            executor.unstack(args[0], args[1])
            return True
        except MotionPrimitiveError as exc:
            print(exc)
            return False
    elif name == "place-next-to":
        try:
            target_xy = _adjacent_xy(BlocksState, args[1], args[0])
            target_xy[0] += 0.01  # slight offset to avoid collisions
            executor.putdown(args[0], target_xy=target_xy)
            return True
        except MotionPrimitiveError as exc:
            print(exc)
            return False
    elif name == "place-next-to-center":
        try:
            target_xy = _center_xy(BlocksState, args[1], args[2], args[0])
            target_xy[0] += 0.005  # slight offset to avoid collisions
            executor.putdown(args[0], target_xy=target_xy)
            return True
        except MotionPrimitiveError as exc:
            print(exc)
            return False
    elif name == "place-top-center":
        try:
            executor.place_top_center(args[0], args[1], args[2], args[3])
            return True
        except MotionPrimitiveError as exc:
            print(exc)
            return False
    else:
        print(f"Unknown action '{name}'")
        return False
    # handle put-down, stack, unstack similarly

def parse_action(atom: str):
    atom = atom.strip().lstrip("(").rstrip(")")
    parts = atom.split()
    name = parts[0]
    args = parts[1:]
    return name, args

sym = lift_scene(franka, BlocksState)
for atom in sym.as_pddl_atoms():
    print(atom)
plan = plan_blocksworld(sym, goal)
for action in plan:
    print(action)
if not plan:
    print("No plan found");
# action = plan[0]
# success = execute_action(action, executor, sym)
# if not success:
#     print("Primitive failed, re-planning…")
    
# for block_name, block_entity in BlocksState.items():
#     print(f"Block {block_name} position: {block_entity.get_pos()}")

# predicates = lift_scene(franka, BlocksState)
# for atom in predicates.as_pddl_atoms():
#     print(atom)
    
print("\n")
sym = lift_scene(franka, BlocksState)
plan = plan_blocksworld(sym, goal)

for action in plan:
    print(action)

# loop through actions
while plan is not None and len(plan) > 0:
    # print current scene predicates
    print("\nCurrent scene predicates:")
    sym = lift_scene(franka, BlocksState)
    for atom in sym.as_pddl_atoms():
        print(atom)
    print("\nExecuting next action:")
    print(plan[0])
    action = plan[0]
    success = execute_action(action, executor, sym)
    if not success:
        print("Primitive failed, re-planning…")
        sym = lift_scene(franka, BlocksState)
        plan = plan_blocksworld(sym, goal)

    else:
        # action succeeded, move to next action
        sym = lift_scene(franka, BlocksState)
        plan = plan_blocksworld(sym, goal)

import time
while True:
    scene.step()
    time.sleep(0.01)   # ~100 Hz viewer updates
