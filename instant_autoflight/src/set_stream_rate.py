import rospy
from rospy import ServiceProxy, ServiceException, Rate

from mavros_msgs.srv import StreamRate


def main():
    rospy.init_node("stream_rate_publisher")

    stream_rate = int(rospy.get_param("~stream_rate", 60))
    update_rate = int(rospy.get_param("~update_rate", 10))
    rate = Rate(update_rate)

    rospy.wait_for_service("/mavros/set_stream_rate")
    command = ServiceProxy("/mavros/set_stream_rate")

    while not rospy.is_shutdown():
        try:
            command(StreamRate(0, stream_rate, True))
        except ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        rate.sleep()


if __name__ == "__main__":
    main()
