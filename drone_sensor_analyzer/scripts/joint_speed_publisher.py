#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64


def velocity_publisher():
    # ノードの初期化
    rospy.init_node("velocity_publisher", anonymous=True)

    # Publisherの設定
    pub = rospy.Publisher(
        "/optical_flow_tester/linear_joint_controller/command", Float64, queue_size=10
    )

    # ループの周期（Hz）
    rate = rospy.Rate(100)

    # 初期速度と加速度
    velocity = 0.0
    acceleration = 0.1 / 20.0  # 0.1 m/s in 20 seconds

    while not rospy.is_shutdown():
        # 速度を更新
        velocity += acceleration * (1.0 / 100.0)  # 50Hzでの更新

        # 速度メッセージを作成
        velocity_msg = Float64()
        velocity_msg.data = velocity

        # 速度メッセージをpublish
        pub.publish(velocity_msg)

        print(velocity_msg.data)

        # 次のループまで待機
        rate.sleep()


if __name__ == "__main__":
    try:
        velocity_publisher()
    except rospy.ROSInterruptException:
        pass
