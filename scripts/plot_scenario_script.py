import csv
import ast
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import numpy as np
from typing import Tuple, List

continuous_state_file_path = "/home/ryan/colav-automaton/colav-automaton-logs/continuous_state.csv"
aux_file_path = "/home/ryan/colav-automaton/colav-automaton-logs/auxiliary_state.csv"


def deserialize_continuous_state(file_path: str) -> Tuple[List, Tuple[List, List]]:
    """
    Deserializes the continuous state at file path.
    Returns:
        timestamps: list of floats
        (x_vals, y_vals): tuple of coordinate lists
    """
    timestamps = []
    x_vals = []
    y_vals = []
    with open(file_path, "r") as f:
        reader = csv.reader(f)
        next(reader)  # skip header: timestamp,state
        for row in reader:
            t_str, state_str = row
            t = float(t_str)
            state = ast.literal_eval(state_str)
            x, y = state[0], state[1]
            timestamps.append(t)
            x_vals.append(x)
            y_vals.append(y)
    return timestamps, (x_vals, y_vals)


def deserialize_auxiliary_states(file_path: str) -> Tuple[List, List]:
    """
    Deserializes an auxiliary states log file.
    Returns:
        timestamps: list of floats
        auxiliary_states: list of dicts with 'waypoints' and 'unsafe_region' keys
    """
    timestamps: List = []
    auxiliary_states: List = []

    with open(file_path, "r") as f:
        reader = csv.reader(f)
        next(reader)
        for row in reader:
            t_str, aux_str = row
            t = float(t_str)
            # Strip trailing comma before closing brace (makes ast.literal_eval happy)
            aux_str_clean = aux_str.strip().rstrip(",}").rstrip() + "}"
            aux = ast.literal_eval(aux_str_clean)
            timestamps.append(t)
            auxiliary_states.append(aux)

    return timestamps, auxiliary_states


def plot_trajectory(
    cont_timestamps: List,
    xy_vals: Tuple[List, List],
    aux_timestamps: List,
    aux_states: List,
) -> None:
    """
    Plots the agent trajectory, unsafe region(s), and waypoints.
    The first waypoint seen is the 'original' waypoint.
    Any subsequent distinct waypoint is a 'virtual' waypoint (shown differently).
    """
    x_vals, y_vals = xy_vals

    fig, ax = plt.subplots(figsize=(10, 8))

    # --- Unsafe region (from last aux state) ---
    if aux_states:
        last_aux = aux_states[-1]
        unsafe = last_aux.get("unsafe_region", [])
        if unsafe:
            poly_pts = np.array(unsafe)
            polygon = Polygon(poly_pts, closed=True, facecolor="red", alpha=0.25,
                              edgecolor="red", linewidth=2, label="Unsafe region")
            ax.add_patch(polygon)

    # --- Collect original and virtual waypoints ---
    original_waypoint = None
    virtual_waypoints = []        # list of (timestamp, [wx, wy])
    seen_waypoints = set()

    for t, aux in zip(aux_timestamps, aux_states):
        wp = aux.get("waypoints", None)
        if wp is None:
            continue
        wp_key = tuple(wp)
        if original_waypoint is None:
            original_waypoint = wp
            seen_waypoints.add(wp_key)
        elif wp_key not in seen_waypoints:
            virtual_waypoints.append((t, wp))
            seen_waypoints.add(wp_key)

    # Plot original waypoint
    if original_waypoint is not None:
        ox, oy = original_waypoint
        ax.plot(ox, oy, marker="*", markersize=20, color="gold",
                markeredgecolor="darkorange", markeredgewidth=1.5,
                zorder=6, label=f"Original waypoint ({ox}, {oy})")

    # Plot virtual waypoints
    for i, (t, wp) in enumerate(virtual_waypoints):
        vx, vy = wp
        ax.plot(vx, vy, marker="*", markersize=20, color="cyan",
                markeredgecolor="dodgerblue", markeredgewidth=1.5,
                zorder=6, label=f"Virtual waypoint {i+1} ({vx}, {vy})  [t={t:.2f}s]")
        # Annotate with an arrow pointing to the virtual waypoint
        ax.annotate(
            f"Virtual WP {i+1}\nt={t:.2f}s",
            xy=(vx, vy),
            xytext=(vx + 8, vy + 8),
            fontsize=8,
            color="dodgerblue",
            arrowprops=dict(arrowstyle="->", color="dodgerblue", lw=1.2),
            zorder=7,
        )

    # --- Trajectory colored by time ---
    x_arr = np.array(x_vals)
    y_arr = np.array(y_vals)
    t_arr = np.array(cont_timestamps)
    t_norm = (t_arr - t_arr.min()) / (t_arr.max() - t_arr.min() + 1e-9)

    scatter = ax.scatter(x_arr, y_arr, c=t_norm, cmap="plasma",
                         s=15, zorder=4, label="Trajectory")
    cbar = fig.colorbar(scatter, ax=ax)
    cbar.set_label("Normalised time (early → late)")

    # Start / end markers
    ax.plot(x_arr[0], y_arr[0], "go", markersize=10, zorder=6, label="Start")
    ax.plot(x_arr[-1], y_arr[-1], "bs", markersize=10, zorder=6, label="End")

    # --- Formatting ---
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title("Agent Trajectory with Unsafe Region and Waypoints")
    ax.legend(loc="upper left", fontsize=8)
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, linestyle="--", alpha=0.4)

    plt.tight_layout()
    plt.show()


def plot_state_over_time(
    cont_timestamps: List,
    xy_vals: Tuple[List, List],
) -> None:
    """
    Plots x and y positions as separate time-series.
    """
    x_vals, y_vals = xy_vals
    t = np.array(cont_timestamps)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6), sharex=True)

    ax1.plot(t, x_vals, color="steelblue", linewidth=1.5)
    ax1.set_ylabel("X position")
    ax1.set_title("Continuous state over time")
    ax1.grid(True, linestyle="--", alpha=0.4)

    ax2.plot(t, y_vals, color="darkorange", linewidth=1.5)
    ax2.set_ylabel("Y position")
    ax2.set_xlabel("Timestamp (s)")
    ax2.grid(True, linestyle="--", alpha=0.4)

    plt.tight_layout()
    plt.show()


def main():
    cont_timestamps, xy_vals = deserialize_continuous_state(continuous_state_file_path)
    aux_timestamps, aux_states = deserialize_auxiliary_states(aux_file_path)

    print(f"Loaded {len(cont_timestamps)} continuous states "
          f"({cont_timestamps[0]:.2f}s – {cont_timestamps[-1]:.2f}s)")
    print(f"Loaded {len(aux_timestamps)} auxiliary states")
    print("Sample aux state:", aux_states[0] if aux_states else "N/A")

    plot_trajectory(cont_timestamps, xy_vals, aux_timestamps, aux_states)
    plot_state_over_time(cont_timestamps, xy_vals)


if __name__ == "__main__":
    main()