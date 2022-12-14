from copy import deepcopy

import rospy
from rospy import Subscriber, Publisher, ServiceException, ServiceProxy

from drone_controller_io.msg import NormalizedRCIn, ChannelState

from mavros_msgs.msg import RCIn
from mavros_msgs.srv import SetMode, CommandBool

from drone_controller_io.normalizer import PWMChannelsNormalizer, PolarityPWMConverter, AnalogToStateConverter


class RCBehaviourNode:
    def __init__(self, pwm_normalizer: PWMChannelsNormalizer):
        self.normalized_control_in = [0] * 16
        self.previous_control_in = [0] * 16
        self.pwm_normalizer = pwm_normalizer

        self.rc_subscriber = Subscriber("/mavros/rc/in", RCIn, self.rc_cb, queue_size=1)
        self.normalized_rc_publisher = Publisher("/normalized_rc/in", NormalizedRCIn, queue_size=1)

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode = ServiceProxy("/mavros/set_mode", SetMode)

        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming = ServiceProxy("/mavros/cmd/arming", CommandBool)

    def rc_cb(self, msg: RCIn) -> None:
        self.previous_control_in = deepcopy(self.normalized_control_in)
        self.normalized_control_in = self.pwm_normalizer.convert(msg.channels)
        self.publish_normalized_rc()

        self.mode_behaviour()
        self.arming_behaviour()

    def publish_normalized_rc(self) -> None:
        norm_rcin = NormalizedRCIn()
        norm_rcin.channel_states = [ChannelState() for i in range(16)]
        for i, norm_value in enumerate(self.normalized_control_in):
            if isinstance(norm_value, int):
                norm_rcin.channel_states[i].state = norm_value
            elif isinstance(norm_value, float):
                norm_rcin.channel_states[i].value = norm_value

        self.normalized_rc_publisher.publish(norm_rcin)

    def mode_behaviour(self) -> None:
        if len(self.normalized_control_in) < 6:
            return
        if self.previous_control_in[5] == self.normalized_control_in[5]:
            return

        if self.normalized_control_in[5] == 1:
            mode = "LOITER"
        else:
            mode = "LAND"

        try:
            result = self.set_mode(custom_mode=mode)
            rospy.logerr(f"set mode[{mode}] result: {result}")
        except ServiceException as e:
            print(f"Service call failed: {e}")

    def arming_behaviour(self) -> None:
        if len(self.normalized_control_in) < 10:
            return
        if self.previous_control_in[8] == self.normalized_control_in[8]:
            return

        if self.normalized_control_in[8] == 1:
            arm = True
        else:
            arm = False

        try:
            result = self.arming(arm)
            rospy.logerr(f"arming: [{arm}] result: {result}")
        except ServiceException as e:
            print(f"Service call failed: {e}")


def main():
    rospy.init_node("rc_behaviour")
    pwm_normalizer = PWMChannelsNormalizer(
        [
            PolarityPWMConverter((1915, 1502, 1095)),
            PolarityPWMConverter((1925, 1505, 1105)),
            PolarityPWMConverter((1136, 1499, 1925)),
            PolarityPWMConverter((1906, 1498, 1085)),
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
