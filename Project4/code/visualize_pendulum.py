#!/usr/bin/env python3
"""
Pendulum Path Visualizer (Static or Animated)
Author: Dhruv Madan

Generates either:
1. A static plot of the final pendulum path and phase-space trajectory, OR
2. An animated pendulum trajectory (default behavior).

Usage:
    # For animation (default)
    python3 visualize_pendulum_static.py --path pendulum_path.txt --out pendulum_trajectory.gif

    # For static image
    python3 visualize_pendulum_static.py --path pendulum_path.txt --out pendulum_final.png --static
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math
import os
import argparse


def get_path_from_file(filepath):
    """Read pendulum path data from text file."""
    data = []
    with open(filepath, 'r') as f:
        for line in f:
            vals = line.strip().split()
            if len(vals) >= 2:
                data.append([float(v) for v in vals])
    return np.array(data)


def plot_static_pendulum(path, output="pendulum_final.png"):
    """
    Plot the final pendulum trajectory (no animation).
    Left: pendulum position
    Right: phase-space trajectory
    """
    # Extract data
    thetas = np.unwrap(path[:, 0])
    omegas = path[:, 1]
    thetas_wrapped = (thetas + np.pi) % (2 * np.pi) - np.pi

    # Pendulum visualization
    L = 1.0
    theta_vis = thetas + math.pi / 2
    x = L * np.sin(theta_vis)
    y = -L * np.cos(theta_vis)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))
    plt.suptitle("Pendulum Path and Phase-Space Trajectory", fontsize=14, weight='bold')

    # --- Left: Pendulum Motion ---
    ax1.set_title("Pendulum Motion (Final Path)")
    ax1.set_xlim(-1.5, 1.5)
    ax1.set_ylim(-1.5, 1.5)
    ax1.set_xlabel("x")
    ax1.set_ylabel("y")
    ax1.axhline(0, color="pink", linestyle="--", linewidth=0.7)
    ax1.axvline(0, color="pink", linestyle="--", linewidth=0.7)
    ax1.plot(x, y, color="#880808", lw=2.5, alpha=0.8, label="Path")
    ax1.plot([0, x[-1]], [0, y[-1]], color="black", lw=1.2, linestyle="--", alpha=0.7)
    ax1.plot(x[0], y[0], 'go', markersize=8, label="Start")
    ax1.plot(x[-1], y[-1], 'ro', markersize=8, label="Goal")
    ax1.legend()

    # --- Right: Phase Space ---
    ax2.set_title("Phase-Space Trajectory (θ vs. ω)")
    ax2.set_xlabel(r"$\theta$ (rad)")
    ax2.set_ylabel(r"$\omega$ (rad/s)")
    ax2.set_xlim(-math.pi, math.pi)
    ax2.set_ylim(-10, 10)
    ax2.plot(thetas_wrapped, omegas, color="lightcoral", lw=2, label="Trajectory")
    ax2.scatter(thetas_wrapped[0], omegas[0], s=60, c='lime', edgecolors='black', label='Start')
    ax2.scatter(thetas_wrapped[-1], omegas[-1], s=60, c='crimson', edgecolors='black', marker='s', label='Goal')
    ax2.legend(loc='upper right', frameon=True, fontsize=8)
    ax2.grid(True, linestyle='--', alpha=0.5)

    plt.tight_layout()
    plt.subplots_adjust(top=0.85)

    plt.savefig(output, dpi=300, bbox_inches='tight')
    print(f"Saved static pendulum plot to {output}")
    plt.close(fig)


def animate_pendulum(path, output="pendulum_animation.gif"):
    """Create an animated pendulum trajectory."""
    thetas_unwrapped = np.unwrap(path[:, 0])
    thetas_wrapped = (thetas_unwrapped + np.pi) % (2 * np.pi) - np.pi
    omegas = path[:, 1]
    n = len(thetas_unwrapped)
    t = np.linspace(0, n * 0.1, n)

    L = 1.0
    theta_vis = thetas_unwrapped + math.pi / 2
    x = L * np.sin(theta_vis)
    y = -L * np.cos(theta_vis)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))
    plt.suptitle("Pendulum Motion and Phase-Space Trajectory", fontsize=14, weight='bold')

    # Left: pendulum motion
    ax1.set_title("Pendulum Motion")
    ax1.set_xlim(-1.5, 1.5)
    ax1.set_ylim(-1.5, 1.5)
    ax1.axhline(0, color="pink", linestyle="--", linewidth=0.7)
    ax1.axvline(0, color="pink", linestyle="--", linewidth=0.7)
    line, = ax1.plot([], [], lw=4, color="#880808")
    bob, = ax1.plot([], [], 'o', color="#880808", markersize=12)
    pivot, = ax1.plot([0], [0], 'ko', markersize=6)

    time_text = ax1.text(-1.3, 1.2, '', fontsize=9,
                         bbox=dict(facecolor='lightgrey', alpha=0.7, edgecolor='black'))

    # Right: phase-space
    ax2.set_title("Configuration Space Trajectory")
    ax2.set_xlim(-math.pi, math.pi)
    ax2.set_ylim(-10, 10)
    ax2.plot(thetas_wrapped, omegas, color="lightcoral", linewidth=1.5, alpha=0.6)
    traj_point, = ax2.plot([], [], 'ro', markersize=5)
    ax2.scatter(thetas_wrapped[0], omegas[0], s=60, c='lime', edgecolors='black', label='Start')
    ax2.scatter(thetas_wrapped[-1], omegas[-1], s=60, c='crimson', edgecolors='black', marker='s', label='Goal')
    ax2.legend(loc='upper right', frameon=True, fontsize=8)

    def update(i):
        line.set_data([0, x[i]], [0, y[i]])
        bob.set_data([x[i]], [y[i]])
        wrapped_th = (thetas_unwrapped[i] + np.pi) % (2 * np.pi) - np.pi
        traj_point.set_data([wrapped_th], [omegas[i]])
        time_text.set_text(f"Time: {t[i]:.2f}s")
        return line, bob, traj_point, time_text

    ani = FuncAnimation(fig, update, frames=n, interval=120, blit=True, repeat=False)
    ani.save(output, writer='pillow', fps=30)
    plt.close(fig)
    print(f"Saved animation to {output}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize pendulum trajectory (static or animated).")
    parser.add_argument("--path", type=str, default="pendulum_path.txt",
                        help="Path file containing theta, omega data.")
    parser.add_argument("--out", type=str, default="pendulum_trajectory.gif",
                        help="Output filename (.gif for animation, .png for static).")
    parser.add_argument("--static", action="store_true",
                        help="If set, generates only a static plot instead of animation.")
    args = parser.parse_args()

    data = get_path_from_file(args.path)

    if args.static:
        plot_static_pendulum(data, output=args.out)
    else:
        animate_pendulum(data, output=args.out)
