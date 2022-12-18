import rospy
from rospy import Subscriber, Publisher, Time, Rate

from std_msgs.msg import Float32
from sensor_msgs.msg import Range
from mavros_msgs.msg import State
from drone_controller_io.msg import NormalizedRCIn, ChannelState, NormalizedRCOut
from instant_autoflight.pid_controller import PIDController, LPF


class AutoAlt:
    def __init__(self) -> None:
        self.target_altitude = 0.6

        self.thrust_calc_stamp = Time()
        self.pid_controller = PIDController(0.7, 0.0, 10, 0.5, LPF(0.2))

        self.pid_publisher_p = Publisher("pid_controller/p", Float32, queue_size=1)
        self.pid_publisher_d = Publisher("pid_controller/d", Float32, queue_size=1)
        self.pid_publisher = Publisher("pid_controller/out", Float32, queue_size=1)

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
        self.pid_controller.update(msg.range, self.target_altitude)

    def rc_cb(self, msg: NormalizedRCIn) -> None:
        self.normalized_control_in = msg.channel_states

    def publish_override(self):
        if abs(self.normalized_control_in[2].value) > 0.1 or self.normalized_control_in[7].state == 0:
            return
        rc_out = NormalizedRCOut()
        rc_out.header.stamp = Time.now()
        rc_out.thrust.value = min(1, max(0, self.pid_controller.output)) * 2 - 1
        rc_out.thrust.override = True

        self.rc_publisher.publish(rc_out)


def main():
    rospy.init_node("rc_to_attitude")

    node = AutoAlt()

    rate = Rate(50)

    while True:
        rate.sleep()
        node.publish_override()


if __name__ == "__main__":
    main()
