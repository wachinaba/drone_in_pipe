import numpy as np
from scipy.spatial.transform import Rotation

import rospy
from rospy import Rate, Subscriber, Publisher, Time

from std_msgs.msg import Float32
from sensor_msgs.msg import Range, Imu
from mavros_msgs.msg import AttitudeTarget, State
from drone_controller_io.msg import NormalizedRCIn, ChannelState
from instant_autoflight.pid_controller import LPF, PIDController


class RCToAttitudeNode:
    def __init__(self) -> None:
        self.thrust_x = 0.0
        self.thrust_y = 0.0
        self.thrust_z = 0.0
        self.yaw = 0.0

        self.current_orientaion = Rotation.from_matrix(np.eye(3))
        self.initial_yaw = None

        self.target_altitude = 0.6
        self.previous_altitude = 0.0

        self.normalized_control_in = [ChannelState() for i in range(16)]

        self.state = State()

        self.rc_subscriber = Subscriber("/normalized_rc/in", NormalizedRCIn, self.rc_cb, queue_size=1)
        self.imu_subscriber = Subscriber("/mavros/imu/data", Imu, self.imu_cb, queue_size=1)
        self.state_subscriber = Subscriber("/mavros/state", State, self.state_cb, queue_size=3)

        self.rangefinder_subscriber = Subscriber(
            "/mavros/distance_sensor/rangefinder_sub", Range, self.rangefinder_cb, queue_size=1
        )

        self.attitude_publisher = Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=5)
        self.pid_controller = PIDController(0.7, 0.0, 10, 0.5, LPF(0.2))

        self.pid_publisher_p = Publisher("pid_controller/p", Float32, queue_size=1)
        self.pid_publisher_d = Publisher("pid_controller/d", Float32, queue_size=1)
        self.pid_publisher = Publisher("pid_controller/out", Float32, queue_size=1)

    def publish_attitude(self) -> None:
        if self.initial_yaw is None:
            rospy.logwarn("initial yaw hasn't estimated yet!")
            return

        target_attitude = AttitudeTarget()
        target_attitude.header.stamp = Time()
        target_attitude.header.frame_id = "map"
        target_attitude.type_mask = 1 & 2 & 4

        pose_quat = Rotation.from_euler(
            "ZYX", np.array([self.initial_yaw + self.yaw, self.thrust_x * 20, self.thrust_y * -20]), True
        ).as_quat()
        target_attitude.orientation.w = pose_quat[3]
        target_attitude.orientation.x = pose_quat[0]
        target_attitude.orientation.y = pose_quat[1]
        target_attitude.orientation.z = pose_quat[2]
        rospy.logwarn(
            f"tgt: {Rotation(pose_quat).as_euler('ZYX', degrees=True)}, curyaw: {self.current_orientaion.as_euler('ZYX', degrees=True)[0]}"
        )

        target_attitude.thrust = self.thrust_z

        self.attitude_publisher.publish(target_attitude)

    def state_cb(self, msg: State):
        self.state = msg

    def calc_thrust_althold(self, altitude):
        error = self.target_altitude - altitude
        if altitude > 0.2:
            self.thrust_z = min(error / 3.0 + 0.5 - (altitude - self.previous_altitude) / 0.2, 0.65)
        else:
            self.thrust_z = min(error + 0.5 - (altitude - self.previous_altitude) / 0.1, 0.8)

        if abs(self.normalized_control_in[2].value) > 0.1 or self.normalized_control_in[7].state == 0:
            self.thrust_z = (self.normalized_control_in[2].value + 1.0) / 2.0
        else:
            rospy.logwarn(f"auto alt: thrust={self.thrust_z}")

    def calc_thrust_stabilize(self, altitude):
        self.thrust_z = min(1, max(0, self.pid_controller.update(altitude, self.target_altitude)))
        rospy.logwarn(f"auto alt pid: thrust={self.thrust_z}")
        self.pid_publisher_p.publish(self.pid_controller.p_output)
        self.pid_publisher_d.publish(self.pid_controller.d_output)
        self.pid_publisher.publish(self.pid_controller.output)

    def rangefinder_cb(self, msg: Range):
        self.calc_thrust_stabilize(msg.range)

        self.previous_altitude = msg.range

    def imu_cb(self, msg: Imu) -> None:
        self.current_orientaion = Rotation(
            np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        )
        if not self.state.armed:
            self.initial_yaw = self.current_orientaion.as_euler("ZYX", degrees=True)[0]

    def set_thrust(self):
        self.thrust_x = self.normalized_control_in[1].value
        self.thrust_y = self.normalized_control_in[3].value
        if abs(self.normalized_control_in[0].value) > 0.1:
            self.yaw += self.normalized_control_in[0].value

    def rc_cb(self, msg: NormalizedRCIn) -> None:
        self.normalized_control_in = msg.channel_states
        self.set_thrust()


def main():
    rospy.init_node("rc_to_attitude")

    node = RCToAttitudeNode()

    while not rospy.is_shutdown():
        node.publish_attitude()
        Rate(60).sleep()


if __name__ == "__main__":
    main()
