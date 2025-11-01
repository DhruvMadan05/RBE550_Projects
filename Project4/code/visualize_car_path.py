#!/usr/bin/env python3
"""
plot_car_path.py
----------------
Plots the *final path* taken by the car from an OMPL output file (no animation).

Usage:
    python3 plot_car_path.py car_path.txt
    python3 plot_car_path.py car_path.txt -o car_path.png
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Circle
import argparse


class CarPathPlotter:
    def __init__(self, path_file, obstacles):
        """
        path_file : str
            Path to the text file output from OMPL planner.
        obstacles : list of tuples
            Each obstacle as (x, y, width, height).
        """
        self.path = np.loadtxt(path_file)
        self.obstacles = obstacles
        self.L = 0.5  # approximate car length for drawing

        # Extract state variables (x, y, theta, v)
        self.x = self.path[:, 0]
        self.y = self.path[:, 1]
        self.theta = self.path[:, 2]
        self.v = self.path[:, 3]

    def draw_environment(self, ax):
        """Draw the static environment (obstacles)."""
        for (x, y, w, h) in self.obstacles:
            rect = patches.Rectangle((x, y), w, h, color="lightgray", ec="black")
            ax.add_patch(rect)
        ax.set_xlim(-10, 15)
        ax.set_ylim(-8, 10)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_aspect('equal', 'box')
        ax.set_title("Final Path of Car")

        # Add goal region marker (same as car.cpp)
        goal_circle = Circle((7.0, 4.0), 0.3, color='gold', fill=False, lw=2)
        ax.add_patch(goal_circle)

    def plot_final_path(self, output_file=None):
        """Plot the final path and the car’s start and end orientation."""
        fig, ax = plt.subplots(figsize=(8, 6))
        self.draw_environment(ax)

        # Draw path line
        ax.plot(self.x, self.y, color='steelblue', lw=2.0, label="Car Path")

        # Draw start and goal points
        ax.plot(self.x[0], self.y[0], 'go', label="Start")
        ax.plot(self.x[-1], self.y[-1], 'ro', label="Goal")

        # Draw car orientation at start and goal
        for (idx, color) in [(0, 'green'), (-1, 'red')]:
            x, y, theta = self.x[idx], self.y[idx], self.theta[idx]
            dx = self.L * np.cos(theta)
            dy = self.L * np.sin(theta)
            ax.arrow(x, y, dx, dy, head_width=0.2, fc=color, ec=color, lw=1.5)

        ax.legend(loc='upper right')
        ax.grid(True)

        if output_file:
            plt.savefig(output_file, dpi=300, bbox_inches='tight')
            print(f"Saved final path plot to {output_file}")
        else:
            plt.show()


def main():
    parser = argparse.ArgumentParser(description="Plot final car path from OMPL planner output.")
    parser.add_argument("path_file", help="Input path file (e.g., car_path.txt)")
    parser.add_argument(
        "--output", "-o",
        help="Output image filename (e.g., car_path.png). If omitted, plot is shown live.",
        default=None
    )
    args = parser.parse_args()

    # Same obstacle layout as makeStreet() in car.cpp
    obstacles = [
        (5.0, -2.0, 7, 5),
        (-4, 5, 16, 2),
        (-4, -2, 7, 4),
        (8, 3, 4, 2),
    ]

    plotter = CarPathPlotter(args.path_file, obstacles)
    plotter.plot_final_path(args.output)


if __name__ == "__main__":
    main()
