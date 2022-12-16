import rospy
from rospy import Subscriber, Publisher, Time

from sensor_msgs.msg import Range
from mavros_msgs.msg import State
from drone_controller_io.msg import NormalizedRCIn, ChannelState, NormalizedRCOut


class AutoAlt:
    def __init__(self) -> None:
        self.thrust_z = 0.0

        self.target_altitude = 0.6
        self.previous_altitude = 0.0

        self.normalized_control_in = [ChannelState() for i in range(16)]

        self.state = State()

        self.rc_subscriber = Subscriber("/normalized_rc/in", NormalizedRCIn, self.rc_cb, queue_size=1)
        self.state_subscriber = Subscriber("/mavros/state", State, self.state_cb, queue_size=3)

        self.rangefinder_subscriber = Subscriber(
            "/mavros/distance_sensor/rangefinder_sub", Range, self.rangefinder_cb, queue_size=1
        )

        self.rc_publisher = Publisher("/normalized_rc/out", NormalizedRCOut, queue_size=1)

    def state_cb(self, msg: State):
        self.state = msg

    def rangefinder_cb(self, msg: Range):
        error = self.target_altitude - msg.range
        if msg.range > 0.2:
            self.thrust_z = min(error / 3.0 + 0.5 - (msg.range - self.previous_altitude) / 0.2, 0.65)
        else:
            self.thrust_z = min(error + 0.7 - (msg.range - self.previous_altitude) / 0.1, 0.8)

        if abs(self.normalized_control_in[2].value) > 0.1 or self.normalized_control_in[7].state == 0:
            self.thrust_z = (self.normalized_control_in[2].value + 1.0) / 2.0
        else:
            rospy.logwarn(f"auto alt: thrust={self.thrust_z}")

        self.previous_altitude = msg.range

        rc_out = NormalizedRCOut()
        rc_out.header.stamp = Time.now()
        rc_out.thrust.value = self.thrust_z * 2 - 1
        rc_out.thrust.override = True

        self.rc_publisher.publish(rc_out)

    def rc_cb(self, msg: NormalizedRCIn) -> None:
        self.normalized_control_in = msg.channel_states


def main():
    rospy.init_node("rc_to_attitude")

    AutoAlt()

    rospy.spin()


if __name__ == "__main__":
    main()
