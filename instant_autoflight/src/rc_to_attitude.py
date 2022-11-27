from datetime import datetime
from typing import Callable
import numpy as np
from scipy.spatial.transform import Rotation

import rospy
from rospy import ServiceException, ServiceProxy, Subscriber, Publisher, Time

from sensor_msgs.msg import Range, Imu
from mavros_msgs.msg import RCIn, AttitudeTarget
from mavros_msgs.srv import CommandLong, CommandBool

from instant_autoflight.normalizer import PWMChannelsNormalizer, PolarityPWMConverter, AnalogToStateConverter


class PID:
    def __init__(self, p: float, ti: float, td: float, const: float, postprocessor: Callable) -> None:
        self.gain_p, self.time_i, self.time_d = p, ti, td
        self.current_control_p = 0.0
        self.current_control_i = 0.0
        self.current_control_d = 0.0
        self.constant_control = const
        self.error_history = [0.0, 0.0]
        self.observed_history = [0.0, 0.0, 0.0]
        self.timestamp_history = []
        self.postprocessor = postprocessor

    def calc(self, observed: float, target: float, stamp: datetime) -> float:
        error = target - observed
        self.error_history.insert(0, error)
        self.error_history = self.error_history[0:2]

        self.observed_history.insert(0, observed)
        self.observed_history = self.observed_history[0:3]

        self.timestamp_history.insert(0, stamp)
        self.timestamp_history = self.timestamp_history[0:3]

        if len(self.timestamp_history) < 3:
            print("prepareing controller...")
            return 0.0

        timedelta_01 = (self.timestamp_history[0] - self.timestamp_history[1]).total_seconds()

        error_integ_01 = (self.error_history[0] + self.error_history[1]) * timedelta_01 / 2

        observed_diff_01 = (self.observed_history[0] - self.observed_history[1]) / timedelta_01

        self.current_control_p = self.gain_p * error
        self.current_control_i = self.postprocessor(
            self.current_control_i + self.gain_p * (error_integ_01 / self.time_i)
        )
        self.current_control_d = self.gain_p * (observed_diff_01 * self.time_d)

        self.current_control = (
            self.current_control_p + self.current_control_i + self.current_control_d + self.constant_control
        )

        print(
            f"O={observed:.3f}, E={error:.3f}: {self.current_control_p:.3f}, {self.current_control_i:.3f}, {self.current_control_d:.3f}"
        )
        # print(self.error_history)
        return self.postprocessor(self.current_control)


class RCToAttitudeNode:
    def __init__(self, pwm_normalizer: PWMChannelsNormalizer) -> None:
        self.thrust_x = 0.0
        self.thrust_y = 0.0
        self.thrust_z = 0.0
        self.yaw = 0.0

        self.current_orientaion = Rotation.from_matrix(np.eye(3))
        self.initial_yaw = None

        self.current_altitude = 0.0
        self.target_altitude = 1.0

        self.pwm_normalizer = pwm_normalizer
        self.normalized_control_in = [0] * 16

        self.pid_controller = PID(0.09, 6.0, 0.18, 0.35, lambda x: min(max(x, 0.0), 1.0))

        rospy.wait_for_service("/mavros/cmd/command")
        self.command = ServiceProxy("/mavros/cmd/command", CommandLong)

        rospy.wait_for_service("/mavros/cmd/arming")
        self.command_arm = ServiceProxy("/mavros/cmd/arming", CommandBool)

        self.rc_subscriber = Subscriber("/mavros/rc/in", RCIn, self.rc_cb, queue_size=1)
        self.rangefinder_subscriber = Subscriber(
            "/mavros/distance_sensor/rangefinder_pub", Range, self.rangefinder_cb, queue_size=1
        )
        self.imu_subscriber = Subscriber("/mavros/imu/data", Imu, self.imu_cb, queue_size=1)

        self.attitude_publisher = Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=5)

        self.arm()

    def publish_attitude(self) -> None:
        if self.initial_yaw is None:
            print("initial yaw hasn't estimated yet!")
            return

        target_attitude = AttitudeTarget()
        target_attitude.header.stamp = Time()
        target_attitude.header.frame_id = "map"
        target_attitude.type_mask = 1 & 2 & 4

        pose_quat = Rotation.from_euler(
            "ZYX", np.array([self.initial_yaw + self.yaw, self.thrust_x * -20, self.thrust_y * 20]), True
        ).as_quat()
        target_attitude.orientation.w = pose_quat[3]
        target_attitude.orientation.x = pose_quat[0]
        target_attitude.orientation.y = pose_quat[1]
        target_attitude.orientation.z = pose_quat[2]
        # print(f"tgt: {Rotation(pose_quat).as_euler('ZYX', degrees=True)}")

        target_attitude.thrust = self.thrust_z

        self.attitude_publisher.publish(target_attitude)

    def rangefinder_cb(self, msg: Range) -> None:
        self.set_stream_rate()
        self.current_altitude = msg.range
        print(f"range:{self.current_altitude}")
        self.thrust_z = self.pid_controller.calc(
            self.current_altitude, self.target_altitude, datetime.fromtimestamp(Time.now().to_time())
        )

        self.publish_attitude()

    def imu_cb(self, msg: Imu) -> None:
        self.current_orientaion = Rotation(
            np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        )
        if self.initial_yaw is None:
            self.initial_yaw = self.current_orientaion.as_euler("ZYX", degrees=True)[0]

    def set_thrust(self):
        self.thrust_x = self.normalized_control_in[1]
        self.thrust_y = self.normalized_control_in[3]
        self.yaw += self.normalized_control_in[0] * -1

    def rc_cb(self, msg: RCIn) -> None:
        self.normalized_control_in = self.pwm_normalizer.convert(msg.channels)
        self.set_thrust()

    def set_stream_rate(self) -> None:
        try:
            self.command(False, 511, 1, 65, 20000, 0, 0, 0, 0, 0)
            self.command(False, 511, 1, 132, 20000, 0, 0, 0, 0, 0)
        except ServiceException as e:
            print(f"Service call failed: {e}")

    def arm(self) -> None:
        try:
            self.command_arm(True)
        except ServiceException as e:
            print(f"Arming failed: {e}")


def main():
    rospy.init_node("rc_to_attitude")

    pwm_normalizer = PWMChannelsNormalizer(
        [
            PolarityPWMConverter((1095, 1502, 1915)),
            PolarityPWMConverter((1925, 1505, 1105)),
            PolarityPWMConverter((1136, 1499, 1925)),
            PolarityPWMConverter((1085, 1498, 1906)),
            AnalogToStateConverter((1101, 1515, 1680, 1927)),
            AnalogToStateConverter((965, 2065)),
            AnalogToStateConverter((1101, 1927)),
            AnalogToStateConverter((1101, 1515, 1927)),
            AnalogToStateConverter((965, 2065)),
            PolarityPWMConverter((965, 1510, 2065)),
        ]
    )
    pwm_normalizer = PWMChannelsNormalizer(
        [
            PolarityPWMConverter((1000, 1500, 2000)),
            PolarityPWMConverter((1000, 1500, 2000)),
            PolarityPWMConverter((1000, 1500, 2000)),
            PolarityPWMConverter((1000, 1500, 2000)),
            AnalogToStateConverter((1101, 1515, 1680, 1927)),
            AnalogToStateConverter((965, 2065)),
            AnalogToStateConverter((1101, 1927)),
            AnalogToStateConverter((1101, 1515, 1927)),
            AnalogToStateConverter((965, 2065)),
            PolarityPWMConverter((965, 1510, 2065)),
        ]
    )

    RCToAttitudeNode(pwm_normalizer)

    rospy.spin()


if __name__ == "__main__":
    main()
