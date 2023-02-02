import rospy
from rospy import Publisher, Subscriber

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class PoseToPathNode:
    def __init__(self) -> None:
        self.path = Path(poses=[])
        self.path_pub = Publisher("/mavros/vision_pose/path", Path, queue_size=10)
        self.pose_sub = Subscriber("/mavros/vision_pose/pose", PoseStamped, self.pose_cb)

    def pose_cb(self, msg: PoseStamped):
        self.path.poses.append(msg)
        self.path.header = msg.header
        self.path_pub.publish(self.path)


def main():
    rospy.init_node("pose_to_path")
    PoseToPathNode()
    rospy.spin()


if __name__ == "__main__":
    main()
