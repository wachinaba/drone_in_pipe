#! /usr/bin/python3

import rospy
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray

from localization_in_pipe_msgs.msg import IntegratedFlow


class OpticalFlowDeltaVisualizer:
    def __init__(self):
        self.marker_id = rospy.get_param("~marker_id", 0)
        self.arrow_shaft_diameter = rospy.get_param("~arrow_shaft_diameter", 0.02)
        self.arrow_head_diameter = rospy.get_param("~arrow_head_diameter", 0.04)
        self.arrow_head_length = rospy.get_param("~arrow_head_length", 0.1)
        # color, list of float
        self.color = dict()
        self.color["velocity"] = rospy.get_param(
            "~color_velocity", [1.0, 0.0, 0.0, 1.0]
        )
        self.color["range"] = rospy.get_param("~color_range", [0.0, 1.0, 0.0, 1.0])
        self.color["camera"] = rospy.get_param("~color_camera", [0.0, 0.0, 1.0, 1.0])
        self.ns_prefix = rospy.get_param("~namespace_prefix", "")

        self.pub = rospy.Publisher("marker", MarkerArray, queue_size=10)
        self.sub = rospy.Subscriber(
            "integrated_flow", IntegratedFlow, self.flow_callback
        )

    def flow_callback(self, msg: IntegratedFlow):
        vectors = {
            "velocity": Point(
                msg.measured_velocity.x,
                msg.measured_velocity.y,
                msg.measured_velocity.z,
            ),
            "range": Point(msg.range.x, msg.range.y, msg.range.z),
            "camera": Point(
                msg.camera_z_axis.x, msg.camera_z_axis.y, msg.camera_z_axis.z
            ),
        }
        marker_array = MarkerArray(
            markers=[
                Marker(
                    header=msg.header,
                    ns=f"{self.ns_prefix}_{kind}",
                    id=self.marker_id,
                    type=Marker.ARROW,
                    action=Marker.ADD,
                    points=[
                        Point(0, 0, 0),  # Start point (origin)
                        vectors[kind],
                    ],
                    scale=Vector3(
                        self.arrow_shaft_diameter,
                        self.arrow_head_diameter,
                        self.arrow_head_length,
                    ),
                    color=ColorRGBA(
                        r=self.color[kind][0],
                        g=self.color[kind][1],
                        b=self.color[kind][2],
                        a=self.color[kind][3],
                    ),
                )
                for kind in ["velocity", "range", "camera"]
            ]
        )

        self.pub.publish(marker_array)


if __name__ == "__main__":
    rospy.init_node("vector_to_arrow_node", anonymous=True)
    node = OpticalFlowDeltaVisualizer()
    rospy.spin()
