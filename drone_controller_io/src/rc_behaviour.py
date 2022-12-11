import rospy
from rospy import Subscriber, Publisher

from drone_controller_io.msg import NormalizedRCIn, ChannelState

from mavros_msgs.msg import RCIn

from instant_autoflight.normalizer import PWMChannelsNormalizer, PolarityPWMConverter, AnalogToStateConverter


class RCBehaviourNode:
    def __init__(self, pwm_normalizer: PWMChannelsNormalizer):
        self.normalized_control_in = [0] * 16
        self.pwm_normalizer = pwm_normalizer

        self.rc_subscriber = Subscriber("/mavros/rc/in", RCIn, self.rc_cb, queue_size=1)
        self.normalized_rc_publisher = Publisher("/normalized_rc/in", NormalizedRCIn, queue_size=1)

    def rc_cb(self, msg: RCIn) -> None:
        self.normalized_control_in = self.pwm_normalizer.convert(msg.channels)
        self.publish_normalized_rc()

    def publish_normalized_rc(self) -> None:
        norm_rcin = NormalizedRCIn()
        norm_rcin.channel_states = [ChannelState() for i in range(16)]
        for i, norm_value in enumerate(self.normalized_control_in):
            if isinstance(norm_value, int):
                norm_rcin.channel_states[i].state = norm_value
            elif isinstance(norm_value, float):
                norm_rcin.channel_states[i].value = norm_value

        self.normalized_rc_publisher.publish(norm_rcin)


def main():
    rospy.init_node("rc_behaviour")
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

    RCBehaviourNode(pwm_normalizer)

    rospy.spin()


if __name__ == "__main__":
    main()
