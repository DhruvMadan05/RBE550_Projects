#!/usr/bin/env python3
"""
Pendulum Trajectory Visualizer for RBE 550 Project 4
Author: Dhruv Madan

Generates an animation showing:
1. Pendulum motion in Cartesian space.
2. Phase-space trajectory (theta vs. omega).
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math
import os
import argparse

def get_path_from_file(filepath):
    """
    Reads pendulum path data from text file.
    Expected format: each line -> theta omega [optional time/torque]
    """
    data = []
    with open(filepath, 'r') as f:
        for line in f:
            vals = line.strip().split()
            if len(vals) >= 2:
                data.append([float(v) for v in vals])
    return np.array(data)

def animate_pendulum(path, output="pendulum_animation.gif"):
    """
    Creates an animation of the pendulum and its phase-space trajectory.

    path: Nx2 or Nx3 numpy array (theta, omega, [optional time])
    output: filename for saved gif
    """
    # --- Unwrap for motion, wrap for phase-space ---
    thetas_unwrapped = np.unwrap(path[:, 0])  # continuous for motion
    thetas_wrapped = (thetas_unwrapped + np.pi) % (2 * np.pi) - np.pi  # [-π, π] for plotting
    omegas = path[:, 1]
    n = len(thetas_unwrapped)
    t = np.linspace(0, n * 0.1, n) if path.shape[1] < 3 else path[:, 2]

    print("First few wrapped θ:", np.round(thetas_wrapped[:10], 3))
    print("First few unwrapped θ:", np.round(thetas_unwrapped[:10], 3))
    print("Last few unwrapped θ:", np.round(thetas_unwrapped[-10:], 3))
    print("Range of unwrapped θ:", thetas_unwrapped.min(), "→", thetas_unwrapped.max())



    # Pendulum parameters
    L = 1.0  # length
    theta_vis = thetas_unwrapped + math.pi/2
    x = L * np.sin(theta_vis)
    y = -L * np.cos(theta_vis)



    # Create figure layout
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))
    plt.suptitle("Pendulum Motion and Phase-Space Trajectory", fontsize=14, weight='bold')

    # --- Left subplot: Pendulum motion
    ax1.set_title("Pendulum Motion")
    ax1.set_xlim(-1.5, 1.5)
    ax1.set_ylim(-1.5, 1.5)
    ax1.set_xlabel("x")
    ax1.set_ylabel("y")
    ax1.axhline(0, color="pink", linestyle="--", linewidth=0.7)
    ax1.axvline(0, color="pink", linestyle="--", linewidth=0.7)
    line, = ax1.plot([], [], lw=4, color="#880808")
    bob, = ax1.plot([], [], 'o', color="#880808", markersize=12)
    pivot, = ax1.plot([0], [0], 'ko', markersize=6)

    # Text box for time and torque
    time_text = ax1.text(-1.3, 1.2, '', fontsize=9, weight='bold',
                         bbox=dict(facecolor='lightgrey', alpha=0.7, edgecolor='black'))
    torque_text = ax1.text(-1.3, 1.05, '', fontsize=9, weight='bold',
                           bbox=dict(facecolor='lightgrey', alpha=0.7, edgecolor='black'))

    # --- Right subplot: Phase-space trajectory
    ax2.set_title("Configuration Space Trajectory")
    ax2.set_xlabel(r"$\theta$ (rad)")
    ax2.set_ylabel(r"$\omega$ (rad/s)")
    ax2.set_xlim(-math.pi, math.pi)
    ax2.set_ylim(-10, 10)
    ax2.plot(thetas_wrapped, omegas, color="lightcoral", linewidth=1.5, alpha=0.6)
    traj_point, = ax2.plot([], [], 'ro', markersize=5)
    ax2.scatter(thetas_wrapped[0], omegas[0], s=60, c='lime', edgecolors='black', label='Start')
    ax2.scatter(thetas_wrapped[-1], omegas[-1], s=60, c='crimson', edgecolors='black', marker='s', label='Goal')
    ax2.legend(loc='upper right', frameon=True, fontsize=8)

    def update(i):
        # Left (pendulum)
        line.set_data([0, x[i]], [0, y[i]])
        bob.set_data([x[i]], [y[i]])
        time_text.set_text(f"Time: {t[i]:.2f} s")
        torque_text.set_text(f"Torque: {2.22*math.sin(thetas_unwrapped[i]):.2f} N·m")  # dummy torque estimate

        # Right (phase-space)
        # set_data expects sequences; wrap scalars in single-element lists
        wrapped_th = (thetas_unwrapped[i] + math.pi) % (2 * math.pi) - math.pi
        traj_point.set_data([wrapped_th], [omegas[i]])

        return line, bob, traj_point, time_text, torque_text

    ani = FuncAnimation(fig, update, frames=n, interval=50, blit=True, repeat=False)

    # Ensure output has a supported animation extension; if not, append .gif
    _root, _ext = os.path.splitext(output)
    supported = {'.gif', '.mp4', '.avi'}
    if _ext.lower() not in supported:
        print(f"Warning: output extension '{_ext}' not supported for animation; appending .gif")
        output = output + '.gif'

    ani.save(output, writer='pillow', fps=30)
    plt.close(fig)
    print(f"Saved animation to {output}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize pendulum trajectory in phase space.")
    parser.add_argument("--path", type=str, default="pendulum_path.txt",
                        help="File containing theta, omega path data.")
    parser.add_argument("--out", type=str, default="pendulum_trajectory.gif",
                        help="Output GIF filename.")
    args = parser.parse_args()

    data = get_path_from_file(args.path)
    animate_pendulum(data, output=args.out)

    import matplotlib.pyplot as plt
    data = np.loadtxt("build/pendulum_path.txt")
    plt.plot(data[:,0], label="θ raw")
    plt.show()

