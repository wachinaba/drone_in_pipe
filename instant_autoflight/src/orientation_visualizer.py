from rospy import Subscriber, Publisher
import rospy

from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu


class OrientationVisualizer:
    def __init__(self) -> None:
        self.marker_pub = Publisher("orientation_marker", Marker, queue_size=10)
        self.imu_sub = Subscriber("/mavros/imu/data", Imu, self.imu_cb)
        self.vision_pose = Subscriber("/mavros/vision_pose/pose", PoseStamped, self.vision_pose_cb)

    def imu_cb(self, msg: Imu):
        marker = Marker()
        marker.header = msg.header
        marker.header.frame_id = "map"
        marker.ns = "imu"
        marker.id = 0
        marker.type = 0
        marker.action = Marker.MODIFY
        marker.pose.orientation = msg.orientation
        marker.color.a = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1.0
        self.marker_pub.publish(marker)

    def vision_pose_cb(self, msg: PoseStamped):
        marker = Marker()
        marker.header = msg.header
        marker.header.frame_id = "map"
        marker.ns = "vision_pose"
        marker.id = 0
        marker.type = 0
        marker.action = Marker.MODIFY
        marker.pose.orientation = msg.pose.orientation
        marker.color.a = 1.0
        marker.color.g = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        self.marker_pub.publish(marker)


def main():
    rospy.init_node("orientation_visualizer")

    OrientationVisualizer()

    rospy.spin()


if __name__ == "__main__":
    main()
