#!/usr/bin/env python3

import sqlite3
import sys
import os

def connect_to_database(db_path):
    """Connect to the SQLite database"""
    if not os.path.exists(db_path):
        print(f"Error: Database file '{db_path}' not found!")
        sys.exit(1)
    
    try:
        conn = sqlite3.connect(db_path)
        return conn
    except sqlite3.Error as e:
        print(f"Error connecting to database: {e}")
        sys.exit(1)

def get_planner_performance_stats(conn):
    """Query database for min, max, and average performance metrics for each planner"""
    
    # First, let's see what tables and columns are available
    cursor = conn.cursor()
    
    # Get table names
    cursor.execute("SELECT name FROM sqlite_master WHERE type='table';")
    tables = cursor.fetchall()
    print("Available tables:")
    for table in tables:
        print(f"  - {table[0]}")
    print()
    
    # Check the structure of the runs table (main data table)
    cursor.execute("PRAGMA table_info(runs);")
    columns = cursor.fetchall()
    print("Columns in 'runs' table:")
    for col in columns:
        print(f"  - {col[1]} ({col[2]})")
    print()
    
    # Query for performance metrics by planner
    query = """
    SELECT 
        plannerid,
        COUNT(*) as num_runs,
        MIN(time) as min_time,
        MAX(time) as max_time,
        AVG(time) as avg_time,
        MIN(solution_length) as min_path_length,
        MAX(solution_length) as max_path_length,
        AVG(solution_length) as avg_path_length,
        MIN(graph_states) as min_graph_states,
        MAX(graph_states) as max_graph_states,
        AVG(graph_states) as avg_graph_states,
        COUNT(CASE WHEN solved = 1 THEN 1 END) as successful_runs,
        (COUNT(CASE WHEN solved = 1 THEN 1 END) * 100.0 / COUNT(*)) as success_rate
    FROM runs 
    GROUP BY plannerid
    ORDER BY plannerid;
    """
    
    try:
        cursor.execute(query)
        results = cursor.fetchall()
        
        print("=== PLANNER PERFORMANCE STATISTICS ===\n")
        
        for row in results:
            planner_id, num_runs, min_time, max_time, avg_time, min_path, max_path, avg_path, min_states, max_states, avg_states, successful_runs, success_rate = row
            
            print(f"Planner: {planner_id}")
            print(f"  Total Runs: {num_runs}")
            print(f"  Successful Runs: {successful_runs}")
            print(f"  Success Rate: {success_rate:.2f}%")
            print(f"  ")
            print(f"  Time Performance:")
            print(f"    Min:  {min_time:.6f} seconds")
            print(f"    Max:  {max_time:.6f} seconds") 
            print(f"    Avg:  {avg_time:.6f} seconds")
            print(f"  ")
            print(f"  Path Length:")
            print(f"    Min:  {min_path:.6f}" if min_path else "    Min:  N/A (no solutions)")
            print(f"    Max:  {max_path:.6f}" if max_path else "    Max:  N/A (no solutions)")
            print(f"    Avg:  {avg_path:.6f}" if avg_path else "    Avg:  N/A (no solutions)")
            print(f"  ")
            print(f"  Graph States:")
            print(f"    Min:  {int(min_states) if min_states else 'N/A'}")
            print(f"    Max:  {int(max_states) if max_states else 'N/A'}")
            print(f"    Avg:  {avg_states:.2f}" if avg_states else "    Avg:  N/A")
            print(f"  " + "="*50)
            print()
            
    except sqlite3.Error as e:
        print(f"Error executing query: {e}")

def get_detailed_planner_names(conn):
    """Get the full planner names from the plannerConfigs table"""
    cursor = conn.cursor()
    
    try:
        cursor.execute("SELECT id, name FROM plannerConfigs ORDER BY id;")
        configs = cursor.fetchall()
        
        print("=== PLANNER CONFIGURATIONS ===")
        for config_id, name in configs:
            print(f"  ID {config_id}: {name}")
        print()
        
    except sqlite3.Error as e:
        print(f"Could not retrieve planner configurations: {e}")

def main():
    # Database path
    db_path = "code/build/benchmarkChain20.db"
    
    print(f"Connecting to database: {db_path}")
    conn = connect_to_database(db_path)
    
    try:
        # Get planner configuration names
        get_detailed_planner_names(conn)
        
        # Get performance statistics
        get_planner_performance_stats(conn)
        
    finally:
        conn.close()

if __name__ == "__main__":
    main()
