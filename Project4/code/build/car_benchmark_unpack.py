#!/usr/bin/env python3
"""
export_car_benchmark_table.py
-----------------------------
Reads multiple OMPL benchmark SQLite databases (AO-RRT, KPIECE1, SST)
and exports:
- Computation Time (average)
- Path Length (average)
- Number of Tree Nodes (average)
- Success Rate (%)

Each database is assumed to contain one main planner of interest
(e.g., KPIECE1.db → KPIECE1 planner data).

Usage:
    python3 export_car_benchmark_table.py benchmark_car_AORRT.db benchmark_car_KPIECE1.db benchmark_car_SST.db
"""

import sqlite3
import sys
import numpy as np
import os
import re

if len(sys.argv) < 2:
    print("Usage: python3 export_car_benchmark_table.py <db1> <db2> <db3> ...")
    sys.exit(1)

db_files = sys.argv[1:]


def extract_results(db_file):
    """Extract benchmark metrics for the planner matching the DB name."""
    # quick sanity: skip empty DB files
    try:
        if os.path.getsize(db_file) == 0:
            print(f"Skipping empty database file: {db_file}")
            return None
    except OSError:
        # file might not exist or be unreadable; let sqlite report that
        pass

    conn = sqlite3.connect(db_file)
    cursor = conn.cursor()

    print(f"\nConnected to database: {db_file}")
    base_name = os.path.basename(db_file).replace(".db", "").lower()

    # Guess target planner name from file (e.g., "KPIECE1" from filename)
    if "kpiece" in base_name:
        target_name = "KPIECE1"
    elif "aorrt" in base_name or "ao-rrt" in base_name or "aor" in base_name:
        target_name = "AO-RRT"
    elif "sst" in base_name:
        target_name = "SST"
    else:
        target_name = None

    # normalize helper
    def _norm(s):
        return re.sub(r'[^0-9a-z]', '', s.lower()) if s else ''

    # Discover planner table and columns robustly
    cursor.execute("SELECT name FROM sqlite_master WHERE type='table';")
    tables = [t[0] for t in cursor.fetchall()]

    # helper to find a column from a list of candidates
    def pick_col(cols, candidates):
        low = [c.lower() for c in cols]
        for cand in candidates:
            if cand.lower() in low:
                return cols[low.index(cand.lower())]
        return None

    planner_table = None
    id_col = None
    name_col = None

    # Preferred table names (common variants)
    preferred = ['planners', 'plannerConfigs', 'plannerconfigs', 'planner_config', 'planner_configurations']
    for p in preferred:
        if p in tables:
            # inspect columns
            cursor.execute(f"PRAGMA table_info({p});")
            cols = [c[1] for c in cursor.fetchall()]
            id_col = pick_col(cols, ['id', 'plannerid', 'planner_id'])
            name_col = pick_col(cols, ['name', 'planner', 'planner_name'])
            if id_col and name_col:
                planner_table = p
                break

    # fallback: search any table that has id-like and name-like columns
    if not planner_table:
        for t in tables:
            cursor.execute(f"PRAGMA table_info({t});")
            cols = [c[1] for c in cursor.fetchall()]
            id_c = pick_col(cols, ['id', 'plannerid', 'planner_id'])
            name_c = pick_col(cols, ['name', 'planner', 'planner_name'])
            if id_c and name_c:
                planner_table = t
                id_col = id_c
                name_col = name_c
                break

    if not planner_table:
        print(f"No planner table found in {db_file}")
        conn.close()
        return None

    cursor.execute(f"SELECT {id_col}, {name_col} FROM {planner_table};")
    planners = cursor.fetchall()

    if target_name:
        # Find matching planner using a relaxed normalization (strip non-alnum)
        tn = _norm(target_name)
        target_planners = [(pid, pname) for pid, pname in planners if tn in _norm(pname)]
    else:
        target_planners = planners  # fallback if unknown

    if not target_planners:
        print(f"No matching planner found in {db_file}")
        conn.close()
        return None

    pid, pname = target_planners[0]  # Assume one matching planner

    # derive a stable display label (AO-RRT, KPIECE1, SST)
    if target_name:
        display_name = target_name
    else:
        n = _norm(pname)
        if 'kpiece' in n or 'kpi' in n:
            display_name = 'KPIECE1'
        elif 'aorrt' in n or 'aor' in n:
            display_name = 'AO-RRT'
        elif 'sst' in n:
            display_name = 'SST'
        else:
            display_name = pname

    # --- runs table & column discovery
    cursor.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='runs';")
    if not cursor.fetchall():
        print(f"No 'runs' table found in {db_file}")
        conn.close()
        return None

    cursor.execute("PRAGMA table_info(runs);")
    run_cols = [c[1] for c in cursor.fetchall()]

    # planner id column in runs may vary
    run_planner_col = pick_col(run_cols, ['plannerid', 'planner_id', 'planner'])

    # convenience to build WHERE clause and parameter
    def where_planner_clause_and_param():
        if run_planner_col:
            return f"{run_planner_col}=?", pid
        # fallback: maybe runs stores planner name
        name_like_col = pick_col(run_cols, ['planner', 'planner_name', 'name'])
        if name_like_col:
            return f"{name_like_col}=?", pname
        # as last resort, no planner filtering
        return None, None

    where_clause, where_param = where_planner_clause_and_param()
    if where_clause is None:
        print(f"Cannot determine planner reference column in runs table of {db_file}")
        conn.close()
        return None

    # --- Computation time (seconds)
    # --- Computation time (seconds)
    time_col = pick_col(run_cols, ['time', 'runtime', 'duration', 'computation_time'])
    if time_col:
        cursor.execute(f"SELECT {time_col} FROM runs WHERE {where_clause};", (where_param,))
        times = [t[0] for t in cursor.fetchall() if t[0] is not None]
    else:
        times = []

    # --- Path length
    # --- Path length
    length_col = pick_col(run_cols, ['solution_length', 'path_length', 'length'])
    if length_col:
        cursor.execute(f"SELECT {length_col} FROM runs WHERE {where_clause};", (where_param,))
        lengths = [l[0] for l in cursor.fetchall() if l[0] is not None and l[0] > 0]
    else:
        lengths = []

    # --- Number of tree nodes
    # --- Number of tree nodes
    nodes_col = pick_col(run_cols, ['graph_states', 'num_states', 'graph_size', 'tree_size', 'num_vertices'])
    if nodes_col:
        cursor.execute(f"SELECT {nodes_col} FROM runs WHERE {where_clause};", (where_param,))
        nodes = [n[0] for n in cursor.fetchall() if n[0] is not None and n[0] > 0]
    else:
        nodes = []

    # --- Success rate
    # --- Success rate
    cols = run_cols
    if 'solved' in [c.lower() for c in cols]:
        solved_col = pick_col(cols, ['solved'])
        cursor.execute(f"SELECT {solved_col} FROM runs WHERE {where_clause};", (where_param,))
        solveds = [s[0] for s in cursor.fetchall()]
        successes = sum(1 for s in solveds if s)
        total_runs = len(solveds)
    else:
        status_col = pick_col(cols, ['status', 'result'])
        if status_col:
            cursor.execute(f"SELECT {status_col} FROM runs WHERE {where_clause};", (where_param,))
            statuses = [s[0] for s in cursor.fetchall()]
            successes = sum(1 for s in statuses if isinstance(s, str) and s.lower() in ("solved", "approximate", "completed"))
            total_runs = len(statuses)
        else:
            # can't determine success column — treat non-empty solutions/time as success heuristic
            total_runs = max(len(times), len(lengths), len(nodes))
            successes = sum(1 for t in times if t is not None)
    success_rate = 100 * successes / total_runs if total_runs > 0 else 0.0

    conn.close()

    return {
        "Planner": display_name,
        "Computation Time (s)": np.mean(times) if times else np.nan,
        "Path Length": np.mean(lengths) if lengths else np.nan,
        "Tree Nodes": np.mean(nodes) if nodes else np.nan,
        "Success Rate (%)": success_rate
    }


# Collect and merge results
results = []
for db in db_files:
    r = extract_results(db)
    if r:
        results.append(r)

# Sort results consistently
order = ["AO-RRT", "KPIECE1", "SST"]
results = sorted(results, key=lambda r: order.index(r["Planner"]) if r["Planner"] in order else 999)

# --- Output LaTeX table
print("\nLaTeX Table:")
print("\\begin{table}[h!]")
print("\\centering")
print("\\begin{tabular}{|l|c|c|c|}")
print("\\hline")
print("\\textbf{Metric} & \\textbf{AO-RRT} & \\textbf{KPIECE1} & \\textbf{SST} \\\\")
print("\\hline")

def val(pname, key):
    for r in results:
        if r["Planner"] == pname:
            v = r[key]
            if key == "Tree Nodes":
                return f"{int(v):,}" if np.isfinite(v) else "--"
            else:
                return f"{v:.2f}" if np.isfinite(v) else "--"
    return "--"

print(f"Computation Time (s) & {val('AO-RRT','Computation Time (s)')} & {val('KPIECE1','Computation Time (s)')} & {val('SST','Computation Time (s)')}\\\\")
print(f"Path Length & {val('AO-RRT','Path Length')} & {val('KPIECE1','Path Length')} & {val('SST','Path Length')}\\\\")
print(f"Tree Nodes & {val('AO-RRT','Tree Nodes')} & {val('KPIECE1','Tree Nodes')} & {val('SST','Tree Nodes')}\\\\")
print(f"Success Rate (\\%) & {val('AO-RRT','Success Rate (%)')} & {val('KPIECE1','Success Rate (%)')} & {val('SST','Success Rate (%)')}\\\\")
print("\\hline")
print("\\end{tabular}")
print("\\caption{Benchmark results for car planners (torque=3).}")
print("\\label{tab:car-benchmark}")
print("\\end{table}")
