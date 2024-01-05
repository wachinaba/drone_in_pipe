import numpy as np
from scipy.spatial.transform import Rotation as R

import rosbag
import argparse
import csv



def main(bag_file):
    row = {
        "time": 0,
        "x": 0,
        "y": 0,
        "z": 0,
        "yaw": 0,
        "pitch": 0,
        "roll": 0,
    }

    csv_file = open(bag_file + ".csv", mode="w")
    csv_writer = csv.DictWriter(csv_file, delimiter=",", quotechar='"', quoting=csv.QUOTE_MINIMAL, fieldnames=row.keys())

    csv_writer.writeheader()
    
    bag = rosbag.Bag(bag_file)
    
    for topic, msg, t in bag.read_messages(topics="/trackers/base_link/joint_state"):
        position = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        orientation_ypr = R.from_quat(orientation).as_euler('ZYX', degrees=True)
        current_row = {
            "time": msg.header.stamp.to_sec(),
            "x": position[0],
            "y": position[1],
            "z": position[2],
            "yaw": orientation_ypr[0],
            "pitch": orientation_ypr[1],
            "roll": orientation_ypr[2],
        }
        csv_writer.writerow(current_row)

    bag.close()

    
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Extract ros topics from a ROS bag file, and save them as CSV files.")
    parser.add_argument("bag_file", help="Path to the ROS bag file")
    args = parser.parse_args()

    main(args.bag_file)