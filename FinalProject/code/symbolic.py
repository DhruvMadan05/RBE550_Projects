"""Symbolic abstraction helpers for the Blocksworld tasks.

This module converts the continuous Genesis scene state into a set of
logical predicates
It utilizes the Blocksworld vocabulary:
ON, ONTABLE, CLEAR, HOLDING, and HANDEMPTY.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Sequence, Set, Tuple

import numpy as np

# Size of each cube in the provided scenes (meters).
BLOCK_SIZE = 0.04
# Height of the table plane in the Genesis scenes.
TABLE_Z = 0.0

# Tolerances (meters) for deciding symbolic relations from noisy poses.
XY_ON_TOL = 0.05          # how close XY projections must be to count as stacked
Z_ON_TOL = 0.01           # acceptable deviation from perfect block height
TABLE_TOL = 0.008         # slack when checking if a block rests on the plane
HOLDING_DIST_TOL = 0.15   # block-hand distance threshold to infer holding
GRIPPER_CLOSED_THRESH = 0.025  # avg finger opening that counts as closed
NEXT_TO_DIST_PAD = 0.02   # slack added when deciding adjacency on the table
NEXT_TO_MIN_SEP = 0.01    # minimum horizontal separation for adjacency
NEXT_TO_CENTER_TOL = 0.02 # allowable displacement along the base direction
NEXT_TO_CENTER_LATERAL = BLOCK_SIZE  # desired perpendicular offset from midpoint
NEXT_TO_CENTER_PERP_TOL = 0.02  # tolerance around the lateral offset


@dataclass(frozen=True)
class BlockState:
    """Snapshot of one cube."""

    name: str
    entity: Any
    pos: np.ndarray
    quat: np.ndarray


@dataclass
class SymbolicState:
    """Symbolic predicates describing the current scene."""

    on: Set[Tuple[str, str]]  # predicates for block a on block b
    ontable: Set[str]  # predicates for block a on the table
    clear: Set[str]  # predicates for block a has nothing on top
    next_to: Set[Tuple[str, str]]  # adjacency predicates at table level
    next_to_center: Set[Tuple[str, str, str]]  # centered-between predicates
    top_center: Set[str]  # blocks stacked atop the center block
    holding: Set[str]  # predicates for block a is held by the robot
    handempty: bool  # predicate for the robot hand being empty

    def as_pddl_atoms(self) -> List[str]:
        """
        Return predicates formatted as strings for a PDDL problem file.
        Returns:
            List[str]: List of predicate strings.
        """
        atoms: List[str] = []
        for a, b in sorted(self.on):
            atoms.append(f"(on {a} {b})")
        for name in sorted(self.ontable):
            atoms.append(f"(ontable {name})")
        for name in sorted(self.clear):
            atoms.append(f"(clear {name})")
        for a, b in sorted(self.next_to):
            atoms.append(f"(nextTo {a} {b})")
        for a, b, c in sorted(self.next_to_center):
            atoms.append(f"(nextToCenter {a} {b} {c})")
        for name in sorted(self.top_center):
            atoms.append(f"(topCenter {name})")
        for name in sorted(self.holding):
            atoms.append(f"(holding {name})")
        if self.handempty:
            atoms.append("(handempty)")
        return atoms


def lift_scene(
    franka: Any,
    blocks_state: Dict[str, Any],
    *,
    block_size: float = BLOCK_SIZE,
    table_z: float = TABLE_Z,
) -> SymbolicState:
    """Convert the Genesis scene state into symbolic predicates.
    Parameters:
        franka (Any): The Franka robot entity.
        blocks_state (Dict[str, Any]): Mapping of block names to their entities.
        block_size (float): The size of each block (default: BLOCK_SIZE).
        table_z (float): The height of the table plane (default: TABLE_Z).
    Returns:
        SymbolicState: The symbolic representation of the scene as PDDL predicates.
    """
    blocks = _snapshot_blocks(blocks_state, size=block_size)
    holding_block = _infer_holding(franka, blocks)
    on_relations = _compute_on_relations(blocks, block_size)
    ontable = _compute_ontable(blocks, block_size, table_z)
    next_to = _compute_next_to(blocks, ontable, block_size)
    next_to_center = _compute_next_to_center(blocks, ontable, next_to, block_size)
    blocked_bases = _infer_blocked_bases(on_relations, next_to_center)
    clear = _compute_clear(blocks, on_relations, extra_blocked=blocked_bases)
    top_center = _compute_top_center(on_relations, next_to_center)

    holding: Set[str] = {holding_block} if holding_block else set()
    handempty = holding_block is None

    # Construct and return the symbolic state
    return SymbolicState(
        on=on_relations,
        ontable=ontable,
        clear=clear,
        next_to=next_to,
        next_to_center=next_to_center,
        top_center=top_center,
        holding=holding,
        handempty=handempty,
    )


def _snapshot_blocks(blocks_state: Dict[str, Any], size: float) -> List[BlockState]:
    """
    Takes a snapshot of the current state of all blocks in the scene.
    Parameters:
        blocks_state (Dict[str, Any]): Mapping of block names to their entities.
        size (float): The size of each block.
    Returns:
        List[BlockState]: List of BlockState dataclasses representing each block.
    """

    blocks: List[BlockState] = []
    for name, entity in blocks_state.items():
        pos = np.asarray(entity.get_pos(), dtype=float)
        quat = np.asarray(entity.get_quat(), dtype=float)
        blocks.append(BlockState(name=name, entity=entity, pos=pos, quat=quat))
    return blocks


def _compute_on_relations(blocks: Sequence[BlockState], block_size: float) -> Set[Tuple[str, str]]:
    """
    Detect which cubes are stacked on top of others.
    
    Parameters:
        blocks (Sequence[BlockState]): List of BlockState dataclasses.
        block_size (float): The size of each block.

    Returns:
        Set[Tuple[str, str]]: Set of (top_block_name, bottom_block_name) pairs indicating ON relations.
    """
    relations: Set[Tuple[str, str]] = set()
    height = block_size
    for top in blocks:
        for bottom in blocks:
            if top is bottom:
                continue
            if _is_on(top.pos, bottom.pos, height):
                relations.add((top.name, bottom.name))
    return relations


def _compute_ontable(
    blocks: Sequence[BlockState],
    block_size: float,
    table_z: float,
) -> Set[str]:
    """
    Determine which blocks are directly on the table.

    Parameters:
        blocks (Sequence[BlockState]): List of BlockState dataclasses.
        block_size (float): The size of each block.
        table_z (float): The height of the table plane.

    Returns:
        Set[str]: Set of block names that are on the table.
    """
    
    names: Set[str] = set()
    half = block_size / 2.0
    for block in blocks:
        resting_height = block.pos[2] - half
        if resting_height <= table_z + TABLE_TOL:
            names.add(block.name)
    return names


def _compute_clear(
    blocks: Sequence[BlockState],
    on_relations: Iterable[Tuple[str, str]],
    extra_blocked: Optional[Iterable[str]] = None,
) -> Set[str]:
    """
    CLEAR(x) if no block is on x.
    Parameters:
        blocks (Sequence[BlockState]): List of BlockState dataclasses.
        on_relations (Iterable[Tuple[str, str]]): Iterable of (top_block_name, bottom_block_name) pairs indicating ON relations.
    
    Returns:
        Set[str]: Set of block names that are clear.
    """
    blocked: Set[str] = {b for _, b in on_relations}
    if extra_blocked:
        blocked.update(extra_blocked)
    return {block.name for block in blocks if block.name not in blocked}


def _compute_next_to(
    blocks: Sequence[BlockState],
    ontable: Set[str],
    block_size: float,
) -> Set[Tuple[str, str]]:
    """Detect adjacency relations between blocks resting on the table."""

    relations: Set[Tuple[str, str]] = set()
    if not blocks:
        return relations
    max_dist = block_size + NEXT_TO_DIST_PAD
    for a in blocks:
        if a.name not in ontable:
            continue
        for b in blocks:
            if a is b or b.name not in ontable:
                continue
            if abs(a.pos[2] - b.pos[2]) > Z_ON_TOL:
                continue
            xy_dist = np.linalg.norm(a.pos[:2] - b.pos[:2])
            if xy_dist < NEXT_TO_MIN_SEP or xy_dist > max_dist:
                continue
            relations.add((a.name, b.name))
    return relations


def _compute_next_to_center(
    blocks: Sequence[BlockState],
    ontable: Set[str],
    next_to: Set[Tuple[str, str]],
    block_size: float,
) -> Set[Tuple[str, str, str]]:
    """Detect blocks centered between adjacent table blocks."""

    triples: Set[Tuple[str, str, str]] = set()
    if not blocks or not next_to:
        return triples

    block_map = {block.name: block for block in blocks}
    # iterate unordered pairs of adjacent blocks
    for a, b in next_to:
        if a >= b:
            continue  # only handle each unordered pair once
        if (b, a) not in next_to:
            continue
        if a not in ontable or b not in ontable:
            continue
        posa = block_map[a].pos
        posb = block_map[b].pos
        midpoint = 0.5 * (posa[:2] + posb[:2])
        base_vec = posb[:2] - posa[:2]
        base_dist = np.linalg.norm(base_vec)
        if base_dist < 1e-6:
            continue
        base_dir = base_vec / base_dist
        perp_vec = np.array([-base_dir[1], base_dir[0]])
        if np.linalg.norm(perp_vec) < 1e-6:
            perp_vec = np.array([1.0, 0.0])
        for center in blocks:
            if center.name in (a, b) or center.name not in ontable:
                continue
            if abs(center.pos[2] - posa[2]) > Z_ON_TOL:
                continue
            rel = center.pos[:2] - midpoint
            parallel_disp = np.dot(rel, base_dir)
            perp_disp = np.dot(rel, perp_vec)
            if (
                abs(parallel_disp) <= NEXT_TO_CENTER_TOL
                and abs(abs(perp_disp) - NEXT_TO_CENTER_LATERAL) <= NEXT_TO_CENTER_PERP_TOL
            ):
                triples.add((center.name, a, b))
                triples.add((center.name, b, a))
    return triples


def _compute_top_center(
    on_relations: Set[Tuple[str, str]],
    next_to_center: Set[Tuple[str, str, str]],
) -> Set[str]:
    """Blocks currently placed atop a centered block."""

    centers_with_base: Set[str] = {center for center, _, _ in next_to_center}
    top_blocks: Set[str] = set()
    for top, center in on_relations:
        if center in centers_with_base:
            top_blocks.add(top)
    return top_blocks


def _infer_blocked_bases(
    on_relations: Set[Tuple[str, str]],
    next_to_center: Set[Tuple[str, str, str]],
) -> Set[str]:
    """Infer which base blocks should be considered blocked by a centered cap."""

    center_to_bases: Dict[str, Set[Tuple[str, str]]] = {}
    for center, a, b in next_to_center:
        pair = tuple(sorted((a, b)))
        center_to_bases.setdefault(center, set()).add(pair)

    blocked: Set[str] = set()
    for top, center in on_relations:
        for pair in center_to_bases.get(center, set()):
            blocked.update(pair)
    return blocked


def _is_on(top_pos: np.ndarray, bottom_pos: np.ndarray, block_height: float) -> bool:
    """
    Check if the top block is on the bottom block based on their positions.

    Parameters:
        top_pos (np.ndarray): Position of the top block.
        bottom_pos (np.ndarray): Position of the bottom block.
        block_height (float): Height of a block.
    
    Returns:
        bool: True if the top block is on the bottom block, False otherwise.
    """


    xy_dist = np.linalg.norm(top_pos[:2] - bottom_pos[:2])
    if xy_dist > XY_ON_TOL:
        return False
    target_z = bottom_pos[2] + block_height
    return abs(top_pos[2] - target_z) <= Z_ON_TOL


def _infer_holding(franka: Any, blocks: Sequence[BlockState]) -> Optional[str]:
    """
    Heuristic holding detector: closed gripper + block near hand frame.
    Parameters:
        franka (Any): The Franka robot entity.
        blocks (Sequence[BlockState]): List of BlockState dataclasses. 

    Returns:
        Optional[str]: The name of the held block, or None if none is held.
    
    """
    if not hasattr(franka, "get_qpos"):
        return None
    qpos = np.asarray(franka.get_qpos(), dtype=float)
    finger_gap = float(np.mean(qpos[-2:]))
    if finger_gap > GRIPPER_CLOSED_THRESH:
        return None
    try:
        hand_pos = np.asarray(franka.get_link("hand").get_pos(), dtype=float)
        print(f"Hand position: {hand_pos}")
    except AttributeError:
        return None

    closest_block: Optional[str] = None
    closest_dist = float("inf")
    for block in blocks:
        dist = np.linalg.norm(block.pos - hand_pos)
        if dist < closest_dist:
            closest_dist = dist
            closest_block = block.name
    if closest_block and closest_dist <= HOLDING_DIST_TOL:
        return closest_block
    return None

def describe_scene(scene, franka, blocks_state) -> None:
    """Print the symbolic predicates for the current scene."""
    symbolic = lift_scene(franka, blocks_state)
    print("Predicates:")
    for atom in symbolic.as_pddl_atoms():
        print(" ", atom)
