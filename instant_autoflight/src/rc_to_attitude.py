import numpy as np
from scipy.spatial.transform import Rotation

import rospy
from rospy import Subscriber, Publisher, Time

from sensor_msgs.msg import Range
from mavros_msgs.msg import RCIn, AttitudeTarget

from instant_autoflight.normalizer import PWMChannelsNormalizer, PWMConverter, AnalogToStateConverter


class RCToAttitudeNode:
    def __init__(self, pwm_normalizer: PWMChannelsNormalizer) -> None:
        self.rc_subscriber = Subscriber("/mavros/rc/in", RCIn, self.rc_cb)
        self.rangefinder_subscriber = Subscriber("/rangefinders/bottom", Range, self.rangefinder_cb)

        self.attitude_publisher = Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=5)

        self.thrust_x = 0.0
        self.thrust_y = 0.0
        self.thrust_z = 0.0
        self.yaw = 0.0

        self.current_altitude = 0.0
        self.target_altitude = 0.5

        self.pwm_normalizer = pwm_normalizer
        self.normalized_control_in = [0] * 16

    def publish_attitude(self) -> None:
        target_attitude = AttitudeTarget()
        target_attitude.header.stamp = Time()
        target_attitude.header.frame_id = "map"
        target_attitude.type_mask = 1 & 2 & 4

        pose_quat = Rotation.from_euler(
            "ZYX", np.array([self.yaw * 10, self.thrust_x * 20, self.thrust_y * -20]), True
        ).as_quat()
        target_attitude.orientation.w = pose_quat[3]
        target_attitude.orientation.x = pose_quat[0]
        target_attitude.orientation.y = pose_quat[1]
        target_attitude.orientation.z = pose_quat[2]

        target_attitude.thrust = self.thrust_z

        self.attitude_publisher.publish(target_attitude)
        rospy.loginfo(f"publish att tgt: {target_attitude}")

    def rangefinder_cb(self, msg: Range) -> None:
        self.current_altitude = msg.range

    def rc_cb(self, msg: RCIn) -> None:
        self.normalized_control_in = self.pwm_normalizer.convert(msg.channels)


def main():
    rospy.init_node("rc_to_attitude")

    pwm_normalizer = PWMChannelsNormalizer(
        [
            PWMConverter((1095, 1915)),
            PWMConverter((1925, 1105)),
            PWMConverter((1136, 1925)),
            PWMConverter((1085, 1906)),
            AnalogToStateConverter((1101, 1515, 1680, 1927)),
            AnalogToStateConverter((965, 2065)),
            AnalogToStateConverter((1101, 1927)),
            AnalogToStateConverter((1101, 1515, 1927)),
            AnalogToStateConverter((965, 2065)),
            PWMConverter((965, 2065)),
        ]
    )

    RCToAttitudeNode(pwm_normalizer)

    rospy.spin()


if __name__ == "__main__":
    main()
