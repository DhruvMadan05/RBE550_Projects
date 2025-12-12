from ast import parse
import sys
import numpy as np
import genesis as gs
import time

from scenes import (
    create_scene_6blocks,
    create_scene_8_blocks,
    create_scene_stacked,
)
from symbolic import lift_scene
from motion_primitives import MotionPrimitiveError, MotionPrimitiveExecutor
from task_planner import (
    plan_blocksworld,
    goal_two_towers,
    goal_single_tower,
    goal_tall_tower,
)

def prompt_choice(prompt, options):
    while True:
        print(prompt)
        for k, v in options.items():
            print(f"  {k}: {v['label']}")
        choice = input("Enter choice (1/2/3): ").strip()
        if choice in options:
            return options[choice]["value"]
        print("Invalid choice. Please try again.\n")


if len(sys.argv) > 1 and sys.argv[1] == "gpu":
    gs.init(backend=gs.gpu, logging_level="Warning", logger_verbose_time=False)
else:
    gs.init(backend=gs.cpu, logging_level="Warning", logger_verbose_time=False)


goal_options = {
    "1": {"label": "Two towers", "value": goal_two_towers},
    "2": {"label": "Single tower", "value": goal_single_tower},
    "3": {"label": "Tallest tower", "value": goal_tall_tower},
}

scene_options = {
    "1": {"label": "6 blocks spread", "value": create_scene_6blocks},
    "2": {"label": "8 blocks, for bonus only", "value": create_scene_8_blocks},
    "3": {"label": "Pre-stacked tower scene", "value": create_scene_stacked},
}

goal_fn = prompt_choice("\nSelect a goal configuration:", goal_options)
scene_fn = prompt_choice("\nSelect a starting scene:", scene_options)


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
print("\nInitial scene predicates:")
for atom in sym.as_pddl_atoms():
    print(atom)

plan = plan_blocksworld(sym, goal)

# Print initial block positions
print("\nInitial block positions:")
for block_name, block_entity in BlocksState.items():
    print(f"Block {block_name} position: {block_entity.get_pos()}")

executor = MotionPrimitiveExecutor(scene, franka, BlocksState)


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
        else:
            print(f"Unknown action '{name}'")
            return False
        return True

    except MotionPrimitiveError as exc:
        print(exc)
        return False


# execution loop

while plan and len(plan) > 0:
    print("\nCurrent scene predicates:")
    sym = lift_scene(franka, BlocksState)
    for atom in sym.as_pddl_atoms():
        print(atom)

    print("\nExecuting action:")
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

# keep the viewer open when done
while True:
    scene.step()
    time.sleep(0.01)  # ~100 Hz viewer
