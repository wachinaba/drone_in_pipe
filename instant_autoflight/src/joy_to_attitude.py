from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import Thrust, AttitudeTarget
from rospy import Subscriber, Publisher, Time
import rospy

import numpy as np
from scipy.spatial.transform import Rotation


class JoyToAttitudeNode:
    def __init__(self) -> None:
        self.joy_subscriber = Subscriber("/joy", Joy, self.joy_cb)
        self.attitude_publisher = Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=5)

    def joy_cb(self, msg: Joy):
        q = Rotation.from_euler("ZYX", np.array([0, msg.axes[1] * 20, msg.axes[0] * -20]), True)
        target = AttitudeTarget()
        target.header.stamp = Time.now()
        target.header.frame_id = "map"
        target.orientation.w = q.as_quat()[3]
        target.orientation.x = q.as_quat()[0]
        target.orientation.y = q.as_quat()[1]
        target.orientation.z = q.as_quat()[2]

        target.thrust = (msg.axes[4] + 1.0) / 2.0

        target.type_mask = 1 & 2 & 4

        self.attitude_publisher.publish(target)
        rospy.loginfo(f"publish att tgt: {target}")


def main():
    rospy.init_node("joy_to_attitude")
    JoyToAttitudeNode()

    rospy.spin()


if __name__ == "__main__":
    main()
