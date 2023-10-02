#!/usr/bin/env python

import datetime
import rospy

from geometry_msgs.msg import PoseStamped, TwistStamped
from optical_flow_msgs.msg import OpticalFlowDelta
from sensor_msgs.msg import Range

import numpy as np

import csv


class MocapOpticalFlowToCSVNode(object):
    def __init__(self):
        rospy.init_node("mocap_optical_flow_to_csv_node", anonymous=True)

        self.mocap_msg = None
        self.flow_msg = None
        self.range_msg = None

        self.csv_name = rospy.get_param("~csv_name", "mocap_optical_flow_")
        self.csv_name += datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + ".csv"

        self.csv_file = open(self.csv_name, mode="w")
        self.csv_writer = csv.writer(
            self.csv_file, delimiter=",", quotechar='"', quoting=csv.QUOTE_MINIMAL
        )
        self.csv_writer.writerow(
            [
                "mocap_time",
                "mocap_x",
                "mocap_y",
                "mocap_z",
                "mocap_qx",
                "mocap_qy",
                "mocap_qz",
                "mocap_qw",
                "flow_time",
                "flow_px",
                "flow_py",
                "flow_quality",
                "range_time",
                "range",
            ]
        )

        rospy.Subscriber(
            "/optical_flow_delta", OpticalFlowDelta, self.optical_flow_callback
        )
        rospy.Subscriber("/pose", PoseStamped, self.mocap_callback)
        rospy.Subscriber("/range", Range, self.range_callback)

    def mocap_callback(self, msg):
        self.mocap_msg = msg
        self.mocap_msg.header.stamp = rospy.Time.now()

    def optical_flow_callback(self, msg):
        self.flow_msg = msg
        self.flow_msg.header.stamp = rospy.Time.now()
        self.add_row()

    def range_callback(self, msg):
        self.range_msg = msg
        self.range_msg.header.stamp = rospy.Time.now()

    def add_row(self):
        if self.mocap_msg is None or self.flow_msg is None or self.range_msg is None:
            return

        row = []
        row.append(self.mocap_msg.header.stamp.to_sec())
        row.append(self.mocap_msg.pose.position.x)
        row.append(self.mocap_msg.pose.position.y)
        row.append(self.mocap_msg.pose.position.z)
        row.append(self.mocap_msg.pose.orientation.x)
        row.append(self.mocap_msg.pose.orientation.y)
        row.append(self.mocap_msg.pose.orientation.z)
        row.append(self.mocap_msg.pose.orientation.w)

        row.append(self.flow_msg.header.stamp.to_sec())
        row.append(self.flow_msg.delta_px)
        row.append(self.flow_msg.delta_py)
        row.append(self.flow_msg.surface_quality)

        row.append(self.range_msg.header.stamp.to_sec())
        row.append(self.range_msg.range)

        self.csv_writer.writerow(row)


if __name__ == "__main__":
    try:
        node = MocapOpticalFlowToCSVNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
