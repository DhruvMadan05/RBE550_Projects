#!/usr/bin/env python3
"""
export_benchmark_table.py
-------------------------
Reads an OMPL benchmark SQLite database and exports:
- Computation Time (average)
- Path Length (average)
- Number of Tree Nodes (average)
- Success Rate (%)

Output is printed as a LaTeX table for easy inclusion in reports.

Usage:
    python3 export_benchmark_table.py benchmark_pendulum.db
"""

import sqlite3
import sys
import numpy as np

if len(sys.argv) < 2:
    print("Usage: python3 export_benchmark_table.py <benchmark.db>")
    sys.exit(1)

db_file = sys.argv[1]

# Connect to the OMPL benchmark SQLite database
conn = sqlite3.connect(db_file)
cursor = conn.cursor()

print(f"Connected to database: {db_file}")

# Query planner names. OMPL benchmark schemas vary: older uses `planners`, newer uses `plannerConfigs`.
try:
    cursor.execute("SELECT id, name FROM planners;")
    planners = cursor.fetchall()
except sqlite3.OperationalError:
    # fallback for newer OMPL benchmark schema
    cursor.execute("SELECT id, name FROM plannerConfigs;")
    planners = cursor.fetchall()

results = []

for pid, pname in planners:
    # --- Computation time (seconds)
    cursor.execute("SELECT time FROM runs WHERE plannerid=?;", (pid,))
    times = [t[0] for t in cursor.fetchall() if t[0] is not None]

    # --- Path length
    cursor.execute("SELECT solution_length FROM runs WHERE plannerid=?;", (pid,))
    lengths = [l[0] for l in cursor.fetchall() if l[0] is not None and l[0] > 0]

    # --- Number of tree nodes
    cursor.execute("SELECT graph_states FROM runs WHERE plannerid=?;", (pid,))
    nodes = [n[0] for n in cursor.fetchall() if n[0] is not None and n[0] > 0]

    # --- Success rate (use 'solved' boolean column if available)
    cursor.execute("PRAGMA table_info(runs);")
    cols = [c[1] for c in cursor.fetchall()]
    if 'solved' in cols:
        cursor.execute("SELECT solved FROM runs WHERE plannerid=?;", (pid,))
        solveds = [s[0] for s in cursor.fetchall()]
        successes = sum(1 for s in solveds if s)
        total_runs = len(solveds)
        success_rate = 100 * successes / total_runs if total_runs > 0 else 0.0
    else:
        # fallback to status text (older schemas)
        cursor.execute("SELECT status FROM runs WHERE plannerid=?;", (pid,))
        statuses = [s[0] for s in cursor.fetchall()]
        successes = sum(1 for s in statuses if isinstance(s, str) and s.lower() in ("solved", "approximate", "completed"))
        total_runs = len(statuses)
        success_rate = 100 * successes / total_runs if total_runs > 0 else 0.0

    results.append({
        "Planner": pname,
        "Computation Time (s)": np.mean(times) if times else np.nan,
        "Path Length": np.mean(lengths) if lengths else np.nan,
        "Tree Nodes": np.mean(nodes) if nodes else np.nan,
        "Success Rate (%)": success_rate
    })

conn.close()



# --- Output as LaTeX table
print("\\begin{table}[h!]")
print("\\centering")
print("\\begin{tabular}{lcccc}")
print("\\hline")
print("Planner & Time (s) & Path Length & Tree Nodes & Success Rate (\\%)\\\\")
print("\\hline")

for r in results:
    print(f"{r['Planner']} & "
          f"{r['Computation Time (s)']:.2f} & "
          f"{r['Path Length']:.2f} & "
          f"{r['Tree Nodes']:.0f} & "
          f"{r['Success Rate (%)']:.1f}\\\\")
print("\\hline")
print("\\end{tabular}")
print("\\caption{Benchmark results for pendulum planners (torque=3).}")
print("\\label{tab:pendulum-benchmark}")
print("\\end{table}")
