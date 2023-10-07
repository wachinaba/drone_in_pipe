#!/usr/bin/env python

import datetime
import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from optical_flow_msgs.msg import OpticalFlowDelta

import numpy as np

import csv


class GazeboOpticalFlowToCSVNode:
    def __init__(self):
        rospy.init_node("gazebo_optical_flow_to_csv_node", anonymous=True)

        self.odom_msg = None
        self.flow_msg = None
        self.range_msg = None
        self.odom_msg_old = None

        self.csv_name = rospy.get_param(
            "~csv_name",
            "gazebo_optical_flow_"
            + datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            + ".csv",
        )

        self.csv_file = open(self.csv_name, mode="w")
        self.csv_writer = csv.writer(
            self.csv_file, delimiter=",", quotechar='"', quoting=csv.QUOTE_MINIMAL
        )
        self.csv_writer.writerow(
            [
                "integration_time",
                "delta_px",
                "delta_py",
                "velocity_x",
                "velocity_y",
                "range",
                "surface_quality",
            ]
        )

        rospy.Subscriber(
            "/optical_flow_delta", OpticalFlowDelta, self.optical_flow_callback
        )
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/range", Range, self.range_callback)

    def range_callback(self, msg):
        self.range_msg = msg

    def odom_callback(self, msg):
        if self.odom_msg_old is not None:
            self.odom_msg = msg
            self.odom_msg.twist.twist.linear.x = (
                self.odom_msg.pose.pose.position.y
                - self.odom_msg_old.pose.pose.position.y
            ) / (self.odom_msg.header.stamp - self.odom_msg_old.header.stamp).to_sec()
            self.odom_msg.twist.twist.linear.y = (
                self.odom_msg.pose.pose.position.x
                - self.odom_msg_old.pose.pose.position.x
            ) / (self.odom_msg.header.stamp - self.odom_msg_old.header.stamp).to_sec()
        self.odom_msg_old = msg

    def optical_flow_callback(self, msg):
        self.flow_msg = msg
        self.add_row()

    def add_row(self):
        if self.odom_msg is None or self.flow_msg is None or self.range_msg is None:
            return

        row = []
        row.append(self.flow_msg.integration_time_us / 1e6)
        row.append(self.flow_msg.delta_px)
        row.append(self.flow_msg.delta_py)
        row.append(self.odom_msg.twist.twist.linear.x)
        row.append(self.odom_msg.twist.twist.linear.y)
        row.append(self.range_msg.range)
        row.append(self.flow_msg.surface_quality)

        self.csv_writer.writerow(row)


if __name__ == "__main__":
    try:
        node = GazeboOpticalFlowToCSVNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
