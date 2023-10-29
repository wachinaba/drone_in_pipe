#!/usr/bin/env python

import rospy
import pandas as pd
from geometry_msgs.msg import PoseStamped, Point, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker

from scipy.spatial.transform import Rotation as R


def publish_pose_from_csv(csv_path):
    # pandasでCSVファイルを読み込む
    df = pd.read_csv(csv_path)

    pub = rospy.Publisher("pose_topic", PoseStamped, queue_size=10)
    marker_array_pub = rospy.Publisher("marker_array_topic", MarkerArray, queue_size=10)

    rate = rospy.Rate(10)  # 0.1秒間隔でpublish

    for index, row in df.iterrows():
        if rospy.is_shutdown():
            break
        pose = PoseStamped()

        # CSVから座標とクォータニオンを読み込む
        pose.pose.position.x = row["mocap_x"]
        pose.pose.position.y = row["mocap_y"]
        pose.pose.position.z = row["mocap_z"]
        pose.pose.orientation.x = row["mocap_qx"]
        pose.pose.orientation.y = row["mocap_qy"]
        pose.pose.orientation.z = row["mocap_qz"]
        pose.pose.orientation.w = row["mocap_qw"]

        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"

        # PoseStampedメッセージをpublish
        pub.publish(pose)

        # クォータニオンから回転行列を計算
        r = R.from_quat(
            [
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ]
        )
        # 回転行列からX, Y軸を取得
        x_axis = r.apply([1, 0, 0])
        y_axis = r.apply([0, 1, 0])

        # オプティカルフローのスケールを設定
        optical_flow_scale = 0.1

        # オプティカルフローデルタを取得
        flow_px = row["flow_px"] * optical_flow_scale
        flow_py = row["flow_py"] * optical_flow_scale

        markers = MarkerArray(
            markers=[
                Marker(
                    type=Marker.ARROW,
                    id=0,
                    ns="flow_x",
                    pose=pose.pose,
                    scale=Vector3(0.1, 0.2, 0.3),
                    header=pose.header,
                    color=ColorRGBA(1.0, 0.0, 0.0, 1.0),  # red
                    points=[
                        Point(),
                        Point(
                            flow_px * optical_flow_scale,
                            0,
                            0,
                        ),
                    ],
                ),
                Marker(
                    type=Marker.ARROW,
                    id=1,
                    ns="flow_y",
                    pose=pose.pose,
                    scale=Vector3(0.1, 0.2, 0.3),
                    header=pose.header,
                    color=ColorRGBA(0.0, 1.0, 0.0, 1.0),  # green
                    points=[
                        Point(),
                        Point(
                            0,
                            flow_py * optical_flow_scale,
                            0,
                        ),
                    ],
                ),
            ]
        )

        # MarkerArrayメッセージをpublish
        marker_array_pub.publish(markers)

        rate.sleep()


if __name__ == "__main__":
    # ROSノードの初期化
    rospy.init_node("csv_log_publisher", anonymous=True)
    try:
        # rosparamからCSVファイルのパスを取得
        csv_path = rospy.get_param("/csv_file_path")
        publish_pose_from_csv(csv_path)
    except rospy.ROSInterruptException:
        pass
