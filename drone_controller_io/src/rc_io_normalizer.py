from rospy import Subscriber, Publisher
from mavros_msgs.msg import RCIn, OverrideRCIn
from drone_controller_io.msg import NormalizedRCIn, NormalizedRCOut


class RCNormalizerNode:
    def __init__(self) -> None:
        self.normalized_input = NormalizedRCIn
        self.normalized_output = NormalizedRCOut

        self.raw_input_subscriber = Subscriber("/mavros/rc/in", RCIn, self.raw_input_cb)
        self.normalized_input_publisher = Publisher("/normalized_controller/in", NormalizedRCIn, queue_size=5)
        self.normalized_output_subscriber = Subscriber(
            "/normalized_controller/out", NormalizedRCOut, self.normalized_out_cb
        )
        self.raw_override_publisher = Publisher("/mavros/rc/override", OverrideRCIn, queue_size=5)

    def raw_input_cb(self, msg: RCIn):
        pass
