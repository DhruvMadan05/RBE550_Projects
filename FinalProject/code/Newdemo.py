from ast import parse
import sys
import numpy as np
import genesis as gs
from scenes import create_scene_6blocks, create_scene_stacked
from symbolic import lift_scene

from motion_primitives import MotionPrimitiveError, MotionPrimitiveExecutor

from task_planner import plan_blocksworld, goal_two_towers

# Ensure Genesis is initialized before building scenes
if len(sys.argv) > 1 and sys.argv[1] == "gpu":
    gs.init(backend=gs.gpu, logging_level='Warning', logger_verbose_time=False)
else:
    gs.init(backend=gs.cpu, logging_level='Warning', logger_verbose_time=False)

# build the scene using the factory
scene, franka, BlocksState = create_scene_6blocks()
#scene, franka, BlocksState = create_scene_stacked()

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
qpos = franka.inverse_kinematics(
    link=franka.get_link("hand"),
    pos=np.array([0.65, 0.0, 0.25]),
    quat=np.array([0, 1, 0, 0]),
)
# gripper open pos
qpos[-2:] = 0.04
path = franka.plan_path(
    qpos_goal=qpos,
    num_waypoints=200,  # 2s duration
)

# execute the planned path
for waypoint in path:
    franka.control_dofs_position(waypoint)
    scene.step()

predicates = lift_scene(franka, BlocksState)
for atom in predicates.as_pddl_atoms():
    print(atom)

goal = goal_two_towers()
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

def execute_action(action, executor, sym):
    name, args = parse_action(action)
    if name == "pick-up":
        try:
            executor.pick(args[0])
            return True
        except MotionPrimitiveError as exc:
            print(exc)
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
plan = plan_blocksworld(sym, goal_two_towers())
for action in plan:
    print(action)
if not plan:
    print("No plan found");
action = plan[0]
success = execute_action(action, executor, sym)
if not success:
    print("Primitive failed, re-planning…")
# after each primitive the loop reiterates, re-lifts, and re-plans

import time
while True:
    scene.step()
    time.sleep(0.01)   # ~100 Hz viewer updates


