#! /usr/bin/python3

import rospy
from rospy import Subscriber, Publisher
from nav_msgs.msg import Odometry
from mavros_msgs.msg import AttitudeTarget
from geometry_msgs.msg import WrenchStamped


def main():
    rospy.init_node("thrust_test_node")

    # パブリッシャーの設定
    thrust_publisher = Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10)

    odom = Odometry()
    wrench = WrenchStamped()
    def odometry_callback(msg):
        odom.twist.twist.angular.z = msg.twist.twist.angular.z

    def wrench_callback(msg):
        wrench.wrench.force.z = msg.wrench.force.z

    # サブスクライバーの設定
    rospy.Subscriber("/trackers/rotor_0/joint_state", Odometry, odometry_callback)
    rospy.Subscriber("/trackers/rotor_0/wrench", WrenchStamped, wrench_callback)

    rate = rospy.Rate(10)
    thrust = -2.5

    while not rospy.is_shutdown():
        thrust_msg = AttitudeTarget()
        thrust_msg.header.stamp = rospy.Time.now()
        thrust_msg.type_mask = AttitudeTarget.IGNORE_PITCH_RATE + AttitudeTarget.IGNORE_ROLL_RATE + AttitudeTarget.IGNORE_YAW_RATE
        thrust_msg.thrust = thrust
        thrust_msg.orientation.w = 1.0
        thrust_publisher.publish(thrust_msg)

        print(f"{thrust}, {odom.twist.twist.angular.z}, {wrench.wrench.force.z}")

        thrust += 0.05
        rate.sleep()



if __name__ == "__main__":
    main()