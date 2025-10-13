import sqlite3
import pandas as pd

DB_PATH = "code/build/narrow_benchmark.db"

conn = sqlite3.connect(DB_PATH)

# Load relevant tables
experiments = pd.read_sql_query("SELECT id, sampler_type FROM experiments;", conn)
runs = pd.read_sql_query("SELECT experimentid, solved, time, solution_length, graph_states FROM runs;", conn)

# Merge to associate sampler type with runs
data = runs.merge(experiments, left_on="experimentid", right_on="id")

# Group by sampler
summary = data.groupby("sampler_type").agg(
    total_runs=("solved", "count"),
    successful_runs=("solved", "sum"),
    success_rate=("solved", "mean"),
    time_min=("time", "min"),
    time_max=("time", "max"),
    time_avg=("time", "mean"),
    path_length_min=("solution_length", "min"),
    path_length_max=("solution_length", "max"),
    path_length_avg=("solution_length", "mean"),
    graph_states_min=("graph_states", "min"),
    graph_states_max=("graph_states", "max"),
    graph_states_avg=("graph_states", "mean")
).reset_index()

# Convert success_rate to percentage
summary["success_rate"] = summary["success_rate"] * 100

# === Print neatly formatted summary ===
print("\n===== PRM Sampler Benchmark Summary =====\n")
print(summary.to_string(index=False, 
    formatters={
        "success_rate": "{:.1f}%".format,
        "time_min": "{:.3f}".format,
        "time_max": "{:.3f}".format,
        "time_avg": "{:.3f}".format,
        "path_length_min": "{:.3f}".format,
        "path_length_max": "{:.3f}".format,
        "path_length_avg": "{:.3f}".format,
        "graph_states_min": "{:,.0f}".format,
        "graph_states_max": "{:,.0f}".format,
        "graph_states_avg": "{:,.1f}".format,
    })
)

    conn.close()


def clearance():
    import sqlite3
    import pandas as pd

    # Path to your benchmark database
    db_path = "code/build/clearance_benchmark.db"

    # Connect to the SQLite database
    conn = sqlite3.connect(db_path)

    # Load runs and planner data
    runs_df = pd.read_sql_query("SELECT * FROM runs;", conn)

    # Different benchmark schema versions may use 'planners' or 'plannerConfigs'.
    # Prefer 'plannerConfigs' if present, otherwise try 'planners'.
    table_list = pd.read_sql_query("SELECT name FROM sqlite_master WHERE type='table';", conn)
    tables = set(table_list['name'].tolist())

    if 'plannerConfigs' in tables:
        planners_df = pd.read_sql_query("SELECT id, name FROM plannerConfigs;", conn)
    elif 'planners' in tables:
        planners_df = pd.read_sql_query("SELECT id, name FROM planners;", conn)
    else:
        raise RuntimeError("No planners or plannerConfigs table found in the database. Available tables: %s" % (', '.join(sorted(tables))))

    # Merge to associate each run with its planner
    # Merge to associate each run with its planner/config name. Planner id column in runs is 'plannerid'.
    merged_df = runs_df.merge(planners_df, left_on="plannerid", right_on="id", suffixes=("", "_planner"))

    # Compute summary statistics
    summary = merged_df.groupby("name").agg(
        total_runs=("id", "count"),
        successful_runs=("solved", "sum"),
        success_rate=("solved", "mean"),
        time_min=("time", "min"),
        time_max=("time", "max"),
        time_avg=("time", "mean"),
        path_length_min=("solution_length", "min"),
        path_length_max=("solution_length", "max"),
        path_length_avg=("solution_length", "mean"),
        graph_states_min=("graph_states", "min"),
        graph_states_max=("graph_states", "max"),
        graph_states_avg=("graph_states", "mean")
    ).reset_index()

    # Convert success_rate to percentage
    summary["success_rate"] = (summary["success_rate"] * 100).round(1).astype(str) + "%"

    # Round numeric columns for readability
    for col in summary.columns[2:]:
        if summary[col].dtype != "O":
            summary[col] = summary[col].round(3)

    # Print summary
    print("\n=== Benchmark Summary ===\n")
    for _, row in summary.iterrows():
        print(f"Planner: {row['name']}")
        print(f"  Total Runs:          {row['total_runs']}")
        print(f"  Successful Runs:     {row['successful_runs']}")
        print(f"  Success Rate:        {row['success_rate']}")
        print(f"  Time (s) – Min:      {row['time_min']}")
        print(f"  Time (s) – Max:      {row['time_max']}")
        print(f"  Time (s) – Avg:      {row['time_avg']}")
        print(f"  Path Length – Min:   {row['path_length_min']}")
        print(f"  Path Length – Max:   {row['path_length_max']}")
        print(f"  Path Length – Avg:   {row['path_length_avg']}")
        print(f"  Graph States – Min:  {row['graph_states_min']}")
        print(f"  Graph States – Max:  {row['graph_states_max']}")
        print(f"  Graph States – Avg:  {row['graph_states_avg']}")
        print("")

    # Close connection
    conn.close()

if __name__ == "__main__":
    #narrow()
    clearance()
