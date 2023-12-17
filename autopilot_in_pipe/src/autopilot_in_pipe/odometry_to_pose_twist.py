#! /usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped


def odometry_callback(msg):
    # PoseStamped メッセージの作成と発行
    pose_msg = PoseStamped()
    pose_msg.header = msg.header
    pose_msg.pose = msg.pose.pose
    pose_publisher.publish(pose_msg)

    # TwistStamped メッセージの作成と発行
    twist_msg = TwistStamped()
    twist_msg.header = msg.header
    twist_msg.twist = msg.twist.twist
    twist_publisher.publish(twist_msg)


def main():
    rospy.init_node("odometry_to_pose_twist_node")

    # パブリッシャーの設定
    global pose_publisher, twist_publisher
    pose_publisher = rospy.Publisher("/pose_stamped", PoseStamped, queue_size=10)
    twist_publisher = rospy.Publisher("/twist_stamped", TwistStamped, queue_size=10)

    # サブスクライバーの設定
    rospy.Subscriber("/odom", Odometry, odometry_callback)

    rospy.spin()


if __name__ == "__main__":
    main()
