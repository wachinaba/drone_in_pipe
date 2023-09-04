#!/usr/bin/env python

import datetime
import rospy

from nav_msgs.msg import Odometry
from optical_flow_msgs.msg import OpticalFlowDelta

import numpy as np

import csv

class MocapOpticalFlowToCSVNode(object):
    def __init__(self):
        rospy.init_node('mocap_optical_flow_to_csv_node', anonymous=True)

        self.mocap_msg = None
        self.flow_msg = None

        self.csv_name = rospy.get_param('~csv_name', 'mocap_optical_flow_' + datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + '.csv')

        self.csv_file = open(self.csv_name, mode='w')
        self.csv_writer = csv.writer(self.csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        self.csv_writer.writerow(['mocap_time', 'mocap_x', 'mocap_y', 'mocap_z', 'mocap_qx', 'mocap_qy', 'mocap_qz', 'mocap_qw', 'mocap_vx', 'mocap_vy', 'mocap_vz', 'mocap_wx', 'mocap_wy', 'mocap_wz', 'flow_time', 'flow_px', 'flow_py', 'flow_quality'])

        rospy.Subscriber('/optical_flow_delta', OpticalFlowDelta, self.optical_flow_callback)
        rospy.Subscriber('/odom', Odometry, self.mocap_callback)
        
      
    def mocap_callback(self, msg):
        self.mocap_msg = msg

    def optical_flow_callback(self, msg):
        self.flow_msg = msg
        self.add_row()
        
    def add_row(self):
        if self.mocap_msg is None or self.flow_msg is None:
            return

        row = []
        row.append(self.mocap_msg.header.stamp.to_sec())
        row.append(self.mocap_msg.pose.pose.position.x)
        row.append(self.mocap_msg.pose.pose.position.y)
        row.append(self.mocap_msg.pose.pose.position.z)
        row.append(self.mocap_msg.pose.pose.orientation.x)
        row.append(self.mocap_msg.pose.pose.orientation.y)
        row.append(self.mocap_msg.pose.pose.orientation.z)
        row.append(self.mocap_msg.pose.pose.orientation.w)
        row.append(self.mocap_msg.twist.twist.linear.x)
        row.append(self.mocap_msg.twist.twist.linear.y)
        row.append(self.mocap_msg.twist.twist.linear.z)
        row.append(self.mocap_msg.twist.twist.angular.x)
        row.append(self.mocap_msg.twist.twist.angular.y)
        row.append(self.mocap_msg.twist.twist.angular.z)
        row.append(self.flow_msg.header.stamp.to_sec())
        row.append(self.flow_msg.delta_px)
        row.append(self.flow_msg.delta_py)
        row.append(self.flow_msg.surface_quality)

        self.csv_writer.writerow(row)

if __name__ == '__main__':
    try:
        node = MocapOpticalFlowToCSVNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass