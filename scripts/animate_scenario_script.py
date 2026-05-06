import csv
import ast
import re
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon
from typing import Tuple, List


continuous_state_file_path = "/home/ryan/colav-automaton-logs/continuous_state.csv"
aux_file_path = "/home/ryan/colav-automaton-logs/auxiliary_state.csv"


# ----------------------------
# Data loading
# ----------------------------

def deserialize_continuous_state(file_path: str) -> Tuple[List, Tuple[List, List]]:
    timestamps = []
    x_vals = []
    y_vals = []

    with open(file_path, "r") as f:
        reader = csv.reader(f)
        next(reader)

        for row in reader:
            t_str, state_str = row
            t = float(t_str)
            state = ast.literal_eval(state_str)

            timestamps.append(t)
            x_vals.append(state[0])
            y_vals.append(state[1])

    return timestamps, (x_vals, y_vals)


def deserialize_auxiliary_states(file_path: str) -> Tuple[List, List]:
    timestamps = []
    auxiliary_states = []

    with open(file_path, "r") as f:
        reader = csv.reader(f)
        next(reader)

        for row in reader:
            t_str, aux_str = row
            t = float(t_str)

            aux_str_clean = aux_str.strip().rstrip(",}").rstrip() + "}"
            aux_str_clean = re.sub(r"<Element '[^']+' at 0x[0-9a-fA-F]+>", "None", aux_str_clean)

            aux = ast.literal_eval(aux_str_clean)

            timestamps.append(t)
            auxiliary_states.append(aux)

    return timestamps, auxiliary_states


# ----------------------------
# Animation
# ----------------------------

def animate(cont_timestamps, xy_vals, aux_timestamps, aux_states):
    x_vals, y_vals = xy_vals

    fig, ax = plt.subplots(figsize=(8, 8))

    # trajectory + agent
    line, = ax.plot([], [], lw=2, color="blue")
    agent, = ax.plot([], [], "ro", markersize=6)

    # unsafe region (dynamic)
    unsafe_patch = None

    # waypoint
    wp_point, = ax.plot([], [], marker="*", markersize=15, color="gold")

    # bounds
    ax.set_xlim(min(x_vals), max(x_vals))
    ax.set_ylim(min(y_vals), max(y_vals))
    ax.set_aspect("equal")
    ax.grid(True)

    # ----------------------------
    # helper: get latest aux state
    # ----------------------------
    def get_aux(t):
        idx = 0
        for i, at in enumerate(aux_timestamps):
            if at <= t:
                idx = i
            else:
                break
        return aux_states[idx]

    # ----------------------------
    # update function
    # ----------------------------
    def update(i):
        nonlocal unsafe_patch

        # trajectory
        line.set_data(x_vals[:i], y_vals[:i])
        agent.set_data([x_vals[i]], [y_vals[i]])

        aux = get_aux(cont_timestamps[i])

        # unsafe region update
        if unsafe_patch:
            unsafe_patch.remove()
            unsafe_patch = None

        unsafe = aux.get("unsafe_region", [])
        if unsafe:
            poly = np.array(unsafe)
            unsafe_patch = Polygon(poly, closed=True, alpha=0.3, color="red")
            ax.add_patch(unsafe_patch)

        # waypoint update
        wp = aux.get("waypoints", None)
        if wp:
            wp_point.set_data([wp[0]], [wp[1]])
        else:
            wp_point.set_data([], [])

        return line, agent, wp_point

    ani = FuncAnimation(
        fig,
        update,
        frames=len(x_vals),
        interval=30,
        blit=False,
        repeat=False
    )

    plt.show()


# ----------------------------
# Main
# ----------------------------

def main():
    cont_timestamps, xy_vals = deserialize_continuous_state(continuous_state_file_path)
    aux_timestamps, aux_states = deserialize_auxiliary_states(aux_file_path)

    print(f"Loaded {len(cont_timestamps)} trajectory points")
    print(f"Loaded {len(aux_timestamps)} auxiliary states")

    animate(cont_timestamps, xy_vals, aux_timestamps, aux_states)


if __name__ == "__main__":
    main()