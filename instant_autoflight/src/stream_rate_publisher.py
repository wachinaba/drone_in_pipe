from typing import List

import rospy
from rospy import ServiceProxy, ServiceException, Rate

from mavros_msgs.srv import CommandLong


class StreamRatePublisherNode:
    def __init__(self, stream_rate: int, msg_ids: List[int]) -> None:
        self.command = ServiceProxy("/mavros/cmd/command", CommandLong)
        self.stream_interval_msec = 1000000 // stream_rate
        self.msg_ids = msg_ids

    def set_stream_rate(self) -> None:
        try:
            for msg_id in self.msg_ids:
                self.command(False, 511, 1, msg_id, self.stream_interval_msec, 0, 0, 0, 0, 0)

        except ServiceException as e:
            print(f"Service call failed: {e}")


def main():
    rospy.init_node("stream_rate_publisher")
    stream_rate = int(rospy.get_param("~stream_rate", 50))
    message_ids_string = str(rospy.get_param("~message_ids"))
    message_ids = [int(s) for s in message_ids_string.replace(" ", "").split(",")]

    node = StreamRatePublisherNode(stream_rate, message_ids)
    rate = Rate(stream_rate)

    while not rospy.is_shutdown():
        node.set_stream_rate()
        rate.sleep()


if __name__ == "__main__":
    main()
