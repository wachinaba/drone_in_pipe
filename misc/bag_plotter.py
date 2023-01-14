from pathlib import Path
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import argparse

from rospy import Duration


def plot(bag_file: Path):
    # read from bag file
    bag = rosbag.Bag(str(bag_file))
    altitudes = np.zeros((0, 2))
    thrusts = np.zeros((0, 2))
    alt_targets = np.zeros((0, 2))

    time_offset = Duration(secs=0)

    csv_text = ["", "", ""]

    for topic, msg, t in bag.read_messages():
        duration = (t - time_offset).to_sec()
        if topic == "/pid_controller/in":
            altitudes = np.append(altitudes, np.array([[duration, msg.data]]), axis=0)
            csv_text[0] += f"{duration},{msg.data}\n"
        if topic == "/pid_controller/out":
            thrusts = np.append(thrusts, np.array([[duration, msg.data]]), axis=0)
            csv_text[1] += f"{duration},{msg.data}\n"
        if topic == "/normalized_rc/in":
            alt_targets = np.append(
                alt_targets, np.array([[duration, msg.channel_states[9].value * 1.5]]), axis=0
            )
            csv_text[2] += f"{duration},{msg.channel_states[9].value * 1.5}\n"

    with open(f"result_{bag_file.name}_0.csv", "w") as f:
        f.write(csv_text[0])
    with open(f"result_{bag_file.name}_1.csv", "w") as f:
        f.write(csv_text[1])
    with open(f"result_{bag_file.name}_2.csv", "w") as f:
        f.write(csv_text[2])

    # plot
    fig = plt.figure()
    ax1 = fig.add_subplot(1, 1, 1)
    ax2 = ax1.twinx()

    ax1.grid(True)
    ax2.grid(False)
    ax1.set_axisbelow(True)

    ax1.plot(altitudes[:, 0], altitudes[:, 1], label="Estimated altitude", color="blue")
    ax2.plot(thrusts[:, 0], thrusts[:, 1], label="Thrust", color="red")
    ax1.plot(alt_targets[:, 0], alt_targets[:, 1], label="Target altitude", color="black")


    handler1, label1 = ax1.get_legend_handles_labels()
    handler2, label2 = ax2.get_legend_handles_labels()
    ax1.legend(handler1 + handler2, label1 + label2, borderaxespad=0.)

    plt.show()

    bag.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_file_path", type=Path)

    args = parser.parse_args()
    plot(args.bag_file_path)
