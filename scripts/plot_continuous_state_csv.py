import csv
import ast
import matplotlib.pyplot as plt
import numpy as np

def plot_xy_from_csv(file_path: str):
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

    # ---- Plot trajectory (x vs y) ----
    plt.figure()
    plt.plot(x_vals, y_vals, label="trajectory")

    # 🎯 Waypoint
    waypoint = [150, 150]
    plt.scatter(*waypoint, marker='x', s=100, label="waypoint")
    plt.text(waypoint[0], waypoint[1], "  WP", verticalalignment='bottom')

    # 🟥 Unsafe region (polygon)
    unsafe_region = [
        np.array([60.0, 60.0]),
        np.array([90.0, 60.0]),
        np.array([90.0, 90.0]),
        np.array([60.0, 90.0]),
    ]

    # Close the polygon loop
    unsafe_x = [p[0] for p in unsafe_region] + [unsafe_region[0][0]]
    unsafe_y = [p[1] for p in unsafe_region] + [unsafe_region[0][1]]

    plt.plot(unsafe_x, unsafe_y, linestyle='--', label="unsafe boundary")
    plt.fill(unsafe_x, unsafe_y, alpha=0.2)

    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("Trajectory (x vs y)")
    plt.legend()
    plt.grid()

    # 🧭 Fix axis scaling + zoom into region of interest
    plt.axis("equal")
    plt.xlim(-50, 200)
    plt.ylim(-50, 150)

    # ---- Plot x and y over time ----
    plt.figure()
    plt.plot(timestamps, x_vals, label="x")
    plt.plot(timestamps, y_vals, label="y")
    plt.xlabel("time")
    plt.ylabel("position")
    plt.title("State over Time")
    plt.legend()
    plt.grid()

    plt.show()


if __name__ == "__main__":
    file_path = "/home/ryan/colav-automaton/colav-automaton-logs/continuous_state.csv"
    plot_xy_from_csv(file_path)