import rospy
from rospy import Subscriber, Publisher, Time, Rate

from std_msgs.msg import Float32
from sensor_msgs.msg import Range
from mavros_msgs.msg import State
from drone_controller_io.msg import NormalizedRCIn, ChannelState, NormalizedRCOut
from instant_autoflight.pid_controller import PIDController, LPF, DeltaEstimator


class AutoAlt:
    def __init__(self) -> None:
        self.range_bottom = Range(range=0.0)
        self.range_top = Range(range=2.0)
        self.range_bottom_delta_estimator = DeltaEstimator()
        self.range_top_delta_estimator = DeltaEstimator()

        self.estimated_diameter = self.range_top.range
        self.diameter_filter = LPF(0.2)
        self.current_altitude = -1.0

        self.target_altitude = -0.4

        self.thrust_calc_stamp = Time()
        self.pid_controller = PIDController(0.5, 0.005, 16, 0.65, LPF(0.16), 0.05)

        self.pid_publisher_p = Publisher("pid_controller/p", Float32, queue_size=1)
        self.pid_publisher_i = Publisher("pid_controller/i", Float32, queue_size=1)
        self.pid_publisher_d = Publisher("pid_controller/d", Float32, queue_size=1)
        self.pid_publisher = Publisher("pid_controller/out", Float32, queue_size=1)
        self.pid_publisher_in = Publisher("pid_controller/in", Float32, queue_size=1)

        self.normalized_control_in = [ChannelState() for i in range(16)]

        self.state = State()

        self.rc_subscriber = Subscriber("/normalized_rc/in", NormalizedRCIn, self.rc_cb, queue_size=1)
        self.state_subscriber = Subscriber("/mavros/state", State, self.state_cb, queue_size=3)

        self.rangefinder_bottom_subscriber = Subscriber(
            "/rangefinder_bottom", Range, self.rangefinder_bottom_cb, queue_size=1
        )
        self.rangefinder_bottom_subscriber = Subscriber(
            "/rangefinder_top", Range, self.rangefinder_top_cb, queue_size=1
        )

        self.rc_publisher = Publisher("/normalized_rc/out", NormalizedRCOut, queue_size=1)

    def state_cb(self, msg: State):
        self.state = msg

    def calc_altitude(self):
        self.estimated_diameter = self.diameter_filter.update(self.range_top.range + self.range_bottom.range + 0.1)
        origin_altitude_bottom = -self.estimated_diameter / 2
        origin_altitude_top = self.estimated_diameter / 2

        blend_top = 1 / (
            1
            + abs(self.range_top_delta_estimator.current_delta)
            + abs(self.range_top_delta_estimator.current_duration)
        )
        blend_bottom = 1 / (
            1
            + abs(self.range_bottom_delta_estimator.current_delta)
            + abs(self.range_bottom_delta_estimator.current_duration)
        )
        blend_norm = blend_top + blend_bottom

        blend_top /= blend_norm
        blend_bottom /= blend_norm

        self.current_altitude = (origin_altitude_bottom + self.range_bottom.range) * blend_bottom + (
            origin_altitude_top - self.range_top.range
        ) * blend_top

        self.pid_controller.update(self.current_altitude, self.target_altitude)
        self.pid_publisher_in.publish(self.current_altitude)

        rospy.logwarn(
            f"D={self.estimated_diameter:.3f} B={self.range_bottom.range:.3f} T={self.range_top.range:.3f}"
            f" Alt={self.current_altitude:.3f} Tgt={self.target_altitude:.3f}"
        )

    def rangefinder_bottom_cb(self, msg: Range):
        self.range_bottom = msg
        self.range_bottom_delta_estimator.update(msg.range, msg.header.stamp)

    def rangefinder_top_cb(self, msg: Range):
        self.range_top = msg
        self.range_top_delta_estimator.update(msg.range, msg.header.stamp)

    def rc_cb(self, msg: NormalizedRCIn) -> None:
        self.normalized_control_in = msg.channel_states

    def publish_override(self):
        self.target_altitude = self.normalized_control_in[9].value * 1.5

        self.calc_altitude()
        rc_out = NormalizedRCOut()
        rc_out.header.stamp = Time.now()

        if (
            abs(self.normalized_control_in[2].value) > 0.1
            or self.normalized_control_in[7].state == 0
            or self.state.mode != "STABILIZE"
        ):
            """
            rospy.logwarn(
                f"autoalt disabled. mode: {self.state.mode}, ch-C: {self.normalized_control_in[7].state},"
                f" ch_throttle: {self.normalized_control_in[2].value:.3f}"
            )
            """
        else:
            rc_out.thrust.value = min(1, max(0, self.pid_controller.output)) * 2 - 1
            rc_out.thrust.override = True
            # rospy.logwarn(f"autoalt enabled: thrust={rc_out.thrust.value:.3f}")

        self.pid_publisher_p.publish(self.pid_controller.p_output)
        self.pid_publisher_i.publish(self.pid_controller.i_output)
        self.pid_publisher_d.publish(self.pid_controller.d_output)
        self.pid_publisher.publish(self.pid_controller.output)

        self.rc_publisher.publish(rc_out)


def main():
    rospy.init_node("rc_to_attitude")

    node = AutoAlt()

    rate = Rate(50)

    while not rospy.is_shutdown():
        rate.sleep()
        node.publish_override()


if __name__ == "__main__":
    main()
