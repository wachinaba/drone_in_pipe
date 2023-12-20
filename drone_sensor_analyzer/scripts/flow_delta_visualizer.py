#! /usr/bin/python3

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from optical_flow_msgs.msg import OpticalFlowDelta


class OpticalFlowDeltaVisualizer:
    def __init__(self):
        # ROSパラメータの読み込み
        self.frame_id = rospy.get_param("~frame_id", "")
        self.marker_id = rospy.get_param("~marker_id", 0)
        self.arrow_shaft_diameter = rospy.get_param("~arrow_shaft_diameter", 0.02)
        self.arrow_head_diameter = rospy.get_param("~arrow_head_diameter", 0.04)
        self.arrow_head_length = rospy.get_param("~arrow_head_length", 0.1)

        # color, list of float
        self.color = rospy.get_param("~color", [1.0, 0.0, 0.0, 1.0])

        self.ns = rospy.get_param("~namespace", "flow_delta")
        self.delta_scale = rospy.get_param("~delta_scale", 1.0)

        self.pub = rospy.Publisher("marker", Marker, queue_size=10)
        rospy.Subscriber("flow_delta", OpticalFlowDelta, self.flow_callback)

    def flow_callback(self, msg: OpticalFlowDelta):
        marker = Marker()

        if self.frame_id == "":
            marker.header.frame_id = msg.header.frame_id
        else:
            marker.header.frame_id = self.frame_id

        marker.header.stamp = msg.header.stamp
        marker.ns = self.ns
        marker.id = self.marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.points.append(Point(0, 0, 0))  # Start point (origin)
        marker.points.append(
            Point(msg.delta_px * self.delta_scale, msg.delta_py * self.delta_scale, 0)
        )

        marker.scale.x = self.arrow_shaft_diameter
        marker.scale.y = self.arrow_head_diameter
        marker.scale.z = self.arrow_head_length

        marker.color.r = self.color[0]
        marker.color.g = self.color[1]
        marker.color.b = self.color[2]
        marker.color.a = self.color[3]

        self.pub.publish(marker)


if __name__ == "__main__":
    rospy.init_node("vector_to_arrow_node", anonymous=True)
    node = OpticalFlowDeltaVisualizer()
    rospy.spin()
