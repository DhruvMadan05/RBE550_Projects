from ast import parse
import sys
import time
import numpy as np
import genesis as gs

from scenes import create_scene_3_blocks, create_scene_8_blocks
from symbolic import lift_scene, BLOCK_SIZE, NEXT_TO_CENTER_LATERAL
from motion_primitives import MotionPrimitiveError, MotionPrimitiveExecutor
from task_planner import plan_blocksworld, goal_pyramid_with_cap, goal_3_blocks

PLACEMENT_BUFFER = 0.005


def prompt_goal(prompt, options):
    while True:
        print(prompt)
        for k, v in options.items():
            print(f"  {k}: {v['label']}")
        choice = input("Enter choice (1/2/3): ").strip()
        if choice in options:
            return options[choice]
        print("Invalid choice. Please try again.\n")


if len(sys.argv) > 1 and sys.argv[1] == "gpu":
    gs.init(backend=gs.gpu, logging_level="Warning", logger_verbose_time=False)
else:
    gs.init(backend=gs.cpu, logging_level="Warning", logger_verbose_time=False)


goal_scene_options = {
    "1": {
        "label": "Special Task 408",
        "goal_fn": goal_3_blocks,
        "scene_fn": create_scene_3_blocks,
    },
    "2": {
        "label": "Special task 153",
        "goal_fn": goal_pyramid_with_cap,
        "scene_fn": create_scene_8_blocks,
    },
}

selection = prompt_goal(
    "\nSelect a goal configuration:",
    goal_scene_options
)

goal_fn = selection["goal_fn"]
scene_fn = selection["scene_fn"]

scene, franka, BlocksState = scene_fn()
goal = goal_fn()


franka.set_dofs_kp(
    np.array([4500, 4500, 3500, 3500, 2000, 2000, 2000, 100, 100])
)
franka.set_dofs_kv(
    np.array([450, 450, 350, 350, 200, 200, 200, 10, 10])
)
franka.set_dofs_force_range(
    np.array([-87, -87, -87, -87, -12, -12, -12, -100, -100]),
    np.array([87, 87, 87, 87, 12, 12, 12, 100, 100]),
)


sym = lift_scene(franka, BlocksState)
print("\nInitial predicates:")
for atom in sym.as_pddl_atoms():
    print(atom)

plan = plan_blocksworld(sym, goal)

print("\nInitial block positions:")
for block_name, block_entity in BlocksState.items():
    print(f"Block {block_name} position: {block_entity.get_pos()}")

executor = MotionPrimitiveExecutor(scene, franka, BlocksState)


def _adjacent_xy(blocks_state, reference: str, held: str) -> np.ndarray:
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

    best_xy = ref_pos[:2] + offsets[0]
    best_score = -float("inf")

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
    pos_a = np.asarray(blocks_state[first].get_pos(), dtype=float)
    pos_b = np.asarray(blocks_state[second].get_pos(), dtype=float)

    midpoint = 0.5 * (pos_a[:2] + pos_b[:2])
    base_vec = pos_b[:2] - pos_a[:2]
    base_norm = np.linalg.norm(base_vec)

    base_dir = base_vec / base_norm if base_norm > 1e-6 else np.array([0.0, 1.0])
    perp_vec = np.array([-base_dir[1], base_dir[0]])

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



def parse_action(atom: str):
    atom = atom.strip().lstrip("(").rstrip(")")
    parts = atom.split()
    return parts[0], parts[1:]


def execute_action(action, executor):
    name, args = parse_action(action)

    try:
        if name == "pick-up":
            executor.pick(args[0])

        elif name == "put-down":
            executor.putdown(args[0])

        elif name == "stack":
            executor.stack(args[0], args[1])

        elif name == "unstack":
            executor.unstack(args[0], args[1])

        elif name == "place-next-to":
            target_xy = _adjacent_xy(BlocksState, args[1], args[0])
            target_xy[0] += 0.01
            executor.putdown(args[0], target_xy=target_xy)

        elif name == "place-next-to-center":
            target_xy = _center_xy(BlocksState, args[1], args[2], args[0])
            target_xy[0] += 0.005
            executor.putdown(args[0], target_xy=target_xy)

        elif name == "place-top-center":
            executor.place_top_center(args[0], args[1], args[2], args[3])

        else:
            print(f"Unknown action '{name}'")
            return False

        return True

    except MotionPrimitiveError as exc:
        print(exc)
        return False



while plan and len(plan) > 0:
    print("\nCurrent scene predicates:")
    sym = lift_scene(franka, BlocksState)
    for atom in sym.as_pddl_atoms():
        print(atom)

    print("\nExecuting next action:")
    print(plan[0])

    success = execute_action(plan[0], executor)

    if not success:
        print("Primitive failed — replanning...")
    else:
        print("Action succeeded.")

    sym = lift_scene(franka, BlocksState)
    plan = plan_blocksworld(sym, goal)

if not plan:
    print("\nNo further actions. Goal achieved or no plan exists.")


while True:
    scene.step()
    time.sleep(0.01)