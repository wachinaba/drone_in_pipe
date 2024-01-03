#!/usr/bin/env python

import datetime
import rospy

from geometry_msgs.msg import PoseStamped, TwistStamped
from optical_flow_msgs.msg import OpticalFlowDelta
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range

import numpy as np

import csv


class MocapOpticalFlowToCSVNode(object):
    def __init__(self):
        rospy.init_node("mocap_optical_flow_to_csv_node", anonymous=True)

        self.mocap_d = np.zeros(3)
        self.mocap_v = np.zeros(3)
        self.mocap_msg_old = None
        self.mocap_msg = None
        self.imu_msg = None
        self.flow_msg = None
        self.range_msg = None

        self.row = {
            "mocap_time": 0,
            "mocap_x": 0,
            "mocap_y": 0,
            "mocap_z": 0,
            "mocap_d": 0,
            "mocap_v": 0,
            "mocap_vx": 0,
            "mocap_vy": 0,
            "mocap_vz": 0,
            "mocap_qx": 0,
            "mocap_qy": 0,
            "mocap_qz": 0,
            "mocap_qw": 0,
            "imu_time": 0,
            "imu_wx": 0,
            "imu_wy": 0,
            "imu_wz": 0,
            "flow_time": 0,
            "flow_p": 0,
            "flow_px": 0,
            "flow_py": 0,
            "flow_quality": 0,
            "range_time": 0,
            "range": 0,
            "K": 0,
        }

        self.csv_name = rospy.get_param("~csv_name", "mocap_optical_flow_")
        self.csv_name += datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + ".csv"

        self.csv_file = open(self.csv_name, mode="w")
        self.csv_writer = csv.DictWriter(
            self.csv_file, delimiter=",", quotechar='"', quoting=csv.QUOTE_MINIMAL, fieldnames=self.row.keys()
        )

        self.csv_writer.writeheader()

        rospy.Subscriber(
            "/optical_flow_delta", OpticalFlowDelta, self.optical_flow_callback
        )
        rospy.Subscriber("/pose", PoseStamped, self.mocap_callback)
        rospy.Subscriber("/imu", Imu, self.imu_callback)
        rospy.Subscriber("/range", Range, self.range_callback)

    def mocap_callback(self, msg):
        self.mocap_msg = msg
        self.mocap_msg.header.stamp = rospy.Time.now()

        if self.mocap_msg_old is not None:
            self.mocap_d = np.array(
                [
                    self.mocap_msg.pose.position.x - self.mocap_msg_old.pose.position.x,
                    self.mocap_msg.pose.position.y - self.mocap_msg_old.pose.position.y,
                    self.mocap_msg.pose.position.z - self.mocap_msg_old.pose.position.z,
                ]
            )
            self.mocap_v = self.mocap_d / (
                self.mocap_msg.header.stamp.to_sec()
                - self.mocap_msg_old.header.stamp.to_sec()
            )

        self.mocap_msg_old = self.mocap_msg
        

    def optical_flow_callback(self, msg):
        self.flow_msg = msg
        self.flow_msg.header.stamp = rospy.Time.now()
        self.add_row()

    def imu_callback(self, msg):
        self.imu_msg = msg
        self.imu_msg.header.stamp = rospy.Time.now()

    def range_callback(self, msg):
        self.range_msg = msg
        self.range_msg.header.stamp = rospy.Time.now()

    def add_row(self):
        if None in [self.mocap_msg, self.imu_msg, self.flow_msg, self.range_msg]:
            return
        
        self.row["mocap_time"] = self.mocap_msg.header.stamp.to_sec()
        self.row["mocap_x"] = self.mocap_msg.pose.position.x
        self.row["mocap_y"] = self.mocap_msg.pose.position.y
        self.row["mocap_z"] = self.mocap_msg.pose.position.z

        self.row["mocap_d"] = np.linalg.norm(self.mocap_d)
        self.row["mocap_v"] = np.linalg.norm(self.mocap_v)
        self.row["mocap_vx"] = self.mocap_v[0]
        self.row["mocap_vy"] = self.mocap_v[1]
        self.row["mocap_vz"] = self.mocap_v[2]

        self.row["mocap_qx"] = self.mocap_msg.pose.orientation.x
        self.row["mocap_qy"] = self.mocap_msg.pose.orientation.y
        self.row["mocap_qz"] = self.mocap_msg.pose.orientation.z
        self.row["mocap_qw"] = self.mocap_msg.pose.orientation.w

        self.row["imu_time"] = self.imu_msg.header.stamp.to_sec()
        self.row["imu_wx"] = self.imu_msg.angular_velocity.x
        self.row["imu_wy"] = self.imu_msg.angular_velocity.y
        self.row["imu_wz"] = self.imu_msg.angular_velocity.z

        self.row["flow_time"] = self.flow_msg.header.stamp.to_sec()
        self.row["flow_p"] = np.linalg.norm(np.array([self.flow_msg.delta_px, self.flow_msg.delta_py]))
        self.row["flow_px"] = self.flow_msg.delta_px
        self.row["flow_py"] = self.flow_msg.delta_py

        self.row["flow_quality"] = self.flow_msg.surface_quality

        self.row["range_time"] = self.range_msg.header.stamp.to_sec()
        self.row["range"] = self.range_msg.range

        self.row["K"] = self.row["mocap_d"] / (self.row["flow_p"] * self.row["range"])

        # K is a constant that converts the optical flow pixel delta to the actual velocity
        # K = v / (p * r)

        # v is the actual velocity
        # p is the optical flow pixel delta
        # r is the distance to the object

        #row.append(np.linalg.norm(self.mocap_v) / (np.linalg.norm(np.array([self.flow_msg.delta_px, self.flow_msg.delta_py])) * self.range_msg.range))

        self.csv_writer.writerow(self.row)


if __name__ == "__main__":
    try:
        node = MocapOpticalFlowToCSVNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
