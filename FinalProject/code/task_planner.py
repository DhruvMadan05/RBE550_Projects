"""Task planning helpers built on top of pyperplan.

This module bridges the symbolic predicates produced by `symbolic.lift_scene`
with a classical Blocksworld STRIPS formulation solved by pyperplan.
"""
from __future__ import annotations

from pathlib import Path
import tempfile
from typing import Iterable, List, Sequence, Tuple

from pyperplan import planner

from symbolic import SymbolicState

PDDL_DIR = Path(__file__).with_name("pddl")
BLOCKSWORLD_DOMAIN = PDDL_DIR / "blocksworld_domain.pddl"


class TaskPlanningError(RuntimeError):
    """Raised when task planning fails."""


def plan_blocksworld(
    symbolic_state: SymbolicState,
    goal_atoms: Sequence[str],
    *,
    domain_file: Path | str = BLOCKSWORLD_DOMAIN,
    search: str = "astar",
    heuristic: str = "hff",
) -> List[str]:
    """Solve the Blocksworld problem using pyperplan."""
    domain_path = Path(domain_file)
    if not domain_path.is_file():
        raise TaskPlanningError(f"Domain file not found: {domain_path}")
    objects = _collect_objects(symbolic_state)
    if not objects:
        raise TaskPlanningError("No block objects detected in symbolic state.")
    problem_text = _build_problem_text(symbolic_state, goal_atoms, objects)

    search_fn = planner.SEARCHES.get(search)
    if search_fn is None:
        raise TaskPlanningError(f"Unknown search algorithm '{search}'.")
    heuristic_cls = planner.HEURISTICS.get(heuristic) if heuristic else None

    with tempfile.TemporaryDirectory() as tmpdir:
        problem_path = Path(tmpdir) / "problem.pddl"
        problem_path.write_text(problem_text, encoding="utf-8")
        solution = planner.search_plan(
            str(domain_path),
            str(problem_path),
            search_fn,
            heuristic_cls,
        )

    if not solution:
        return []

    return [op.name for op in solution]


def goal_two_towers(
    first_stack: Sequence[str] = ("r", "g", "b"),
    second_stack: Sequence[str] = ("y", "m", "c"),
) -> List[str]:
    """Goal atoms for the two 3-block towers described in the project spec."""
    atoms = []
    atoms.extend(_tower_goal_atoms(first_stack))
    atoms.extend(_tower_goal_atoms(second_stack))
    atoms.append("(handempty)")
    return _dedupe_preserve_order(atoms)


def goal_single_tower(
        order: Sequence[str] = ("m", "y", "b", "r", "g"),
) -> List[str]:
    """Build goal atoms for a single tower whose order is specified top-down."""
    atoms = _tower_goal_atoms(order)
    atoms.append("(handempty)")
    return _dedupe_preserve_order(atoms)

def goal_tall_tower(
        order: Sequence[str] = ("c", "m", "y", "b", "r", "g"),
) -> List[str]:
    """Build goal atoms for a single tower whose order is specified top-down."""
    atoms = _tower_goal_atoms(order)
    atoms.append("(handempty)")
    return _dedupe_preserve_order(atoms)

def goal_3_blocks(
        base_pair: Sequence[str] = ("b", "o"),
        center_block: str = "r",
) -> List[str]:
    """Goal for the pyramid-like layout with two neighbors and a centered block."""

    if len(base_pair) != 2:
        raise ValueError("base_pair must contain exactly two block names")
    left, right = base_pair
    atoms: List[str] = [
        f"(ontable {left})",
        f"(ontable {right})",
        f"(ontable {center_block})",
        f"(clear {left})",
        f"(clear {right})",
        f"(clear {center_block})",
        f"(nextTo {left} {right})",
        f"(nextTo {right} {left})",
        f"(nextToCenter {center_block} {left} {right})",
        f"(nextToCenter {center_block} {right} {left})",
        "(handempty)",
    ]
    return _dedupe_preserve_order(atoms)

def _tower_goal_atoms(stack: Sequence[str]) -> List[str]:
    atoms: List[str] = []
    if not stack:
        return atoms
    atoms.append(f"(clear {stack[0]})")
    atoms.append(f"(ontable {stack[-1]})")
    for top, bottom in zip(stack, stack[1:]):
        atoms.append(f"(on {top} {bottom})")
    return atoms


def _collect_objects(state: SymbolicState) -> List[str]:
    names = set(state.ontable) | set(state.clear) | set(state.holding)
    for a, b in state.on:
        names.add(a)
        names.add(b)
    return sorted(names)


def _build_problem_text(
    symbolic_state: SymbolicState,
    goal_atoms: Sequence[str],
    objects: Iterable[str],
) -> str:
    init_atoms = symbolic_state.as_pddl_atoms()
    init_section = "\n        ".join(init_atoms)
    goal_section = "\n        ".join(goal_atoms)
    objects_str = " ".join(objects)
    return f"""(define (problem blocksworld-current)
  (:domain blocksworld)
  (:objects {objects_str})
  (:init
        {init_section}
  )
  (:goal (and
        {goal_section}
  ))
)"""


def _dedupe_preserve_order(items: Sequence[str]) -> List[str]:
    seen = set()
    result = []
    for item in items:
        if item in seen:
            continue
        seen.add(item)
        result.append(item)
    return result
