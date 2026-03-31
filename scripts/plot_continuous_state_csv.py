import csv
import ast
import matplotlib.pyplot as plt

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
    plt.plot(x_vals, y_vals)
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("Trajectory (x vs y)")
    plt.grid()

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
    file_path = "/home/ryan/colav-automaton/colav-automaton-logs/continuous_state.csv"  # 👈 change this
    plot_xy_from_csv(file_path)