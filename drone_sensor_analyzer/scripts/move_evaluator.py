#!/usr/bin/env python

import argparse
from time import time_ns
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseStamped
from scipy.spatial.transform import Rotation as R

def main(bag_file):
    # 速度とポーズのデータを格納するリスト
    velocity_data = []
    pose_velocity_data = []
    pose_data = []
    time_origin = 0

    def calculate_velocity_from_pose(pose1, pose2, dt):
        """ポーズの差分から速度を計算"""
        dx = pose2.position.x - pose1.position.x
        dy = pose2.position.y - pose1.position.y
        dz = pose2.position.z - pose1.position.z
        return np.array([dx, dy, dz]) / dt

    # ROSバッグファイルを開く
    bag = rosbag.Bag(bag_file, 'r')
    last_pose = None

    optical_flow_topics = ["/optical_flow_delta/bottom", "/optical_flow_delta/top", "/optical_flow_delta/left", "/optical_flow_delta/right"]
    flow_quality_data = [[], [], [], []]

    for topic, msg, t in bag.read_messages(topics=['/velocity', '/pose'] + optical_flow_topics):
        if time_origin == 0:
            time_origin = msg.header.stamp.to_sec()
        if topic == '/velocity':
            # orientationを使ってワールド座標系に変換
            if last_pose is not None and msg.header.frame_id == 'base_link':
                orientation_q = last_pose.pose.orientation
                r = R.from_quat([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
                print(msg)
                v = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
                v = r.apply(v)
                velocity_data.append([msg.header.stamp.to_sec() - time_origin] + v.tolist())
                
        elif topic == '/pose':
            if last_pose is not None:
                dt = (msg.header.stamp - last_pose.header.stamp).to_sec()
                if dt > 0:
                    pose_velocity = calculate_velocity_from_pose(last_pose.pose, msg.pose, dt)
                    pose_velocity_data.append([msg.header.stamp.to_sec() - time_origin] + pose_velocity.tolist())
            last_pose = msg
            pose_data.append([msg.header.stamp.to_sec() - time_origin, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

        elif topic in optical_flow_topics:
            flow_quality_data[optical_flow_topics.index(topic)].append([msg.header.stamp.to_sec() - time_origin, msg.surface_quality])

    bag.close()

    def diff4(x, h):
        """
        1回微分{-f(x+2h)+8f(x+h)-8f(x-h)+f(x-2h)}/12h
        xは時系列データ, hはデータ間の時間(second)
        """
        res = np.zeros_like(x)
        res[2:-2] = -x[4:] + 8*x[3:-1] - 8*x[1:-3] + x[:-4]
        return res/(12*h)

    # プロット
    plt.figure()

    velocity_array = np.array(velocity_data)
    pose_velocity_array = np.array(pose_velocity_data)
    pose_array = np.array(pose_data)
    flow_quality_arrays = [np.array(flow_quality_data[0]), np.array(flow_quality_data[1]), np.array(flow_quality_data[2]), np.array(flow_quality_data[3])]

    # diff4を使って微分
    dt_array = np.diff(pose_array[:, 0])
    dt_array = np.append(dt_array, dt_array[-1])
    
    pose_velocity_array = np.zeros_like(pose_array)
    pose_velocity_array = diff4(pose_array[:, 1:], dt_array[:, None])
    pose_velocity_array = np.concatenate([pose_array[:, 0:1], pose_velocity_array], axis=1)


    # sort by time
    velocity_array = velocity_array[velocity_array[:, 0].argsort()]
    pose_velocity_array = pose_velocity_array[pose_velocity_array[:, 0].argsort()]
    pose_array = pose_array[pose_array[:, 0].argsort()]
    for i in range(4):
        flow_quality_arrays[i] = flow_quality_arrays[i][flow_quality_arrays[i][:, 0].argsort()]
    
    time_origin = 32

    # set font
    

    plt.subplot(2, 1, 1)
    plt.plot(velocity_array[:, 0]-time_origin, velocity_array[:, 2], label='y')
    plt.plot(pose_velocity_array[:, 0]-time_origin, pose_velocity_array[:, 2], label='y from pose')
    plt.legend()
    plt.ylabel('Velocity [m/s]')
    plt.xlim(0, 8)
    
    plt.grid(True)
    plt.grid(which='minor', linestyle=':')

    plt.subplot(2, 1, 2)
    plt.plot(flow_quality_arrays[0][:, 0]-time_origin, flow_quality_arrays[0][:, 1], label='bottom')
    plt.plot(flow_quality_arrays[1][:, 0]-time_origin, flow_quality_arrays[1][:, 1], label='top')
    plt.plot(flow_quality_arrays[2][:, 0]-time_origin, flow_quality_arrays[2][:, 1], label='left')
    plt.plot(flow_quality_arrays[3][:, 0]-time_origin, flow_quality_arrays[3][:, 1], label='right')
    plt.legend()
    plt.xlabel('Time [s]')
    plt.ylabel('Flow Quality')  
    plt.xlim(0, 8)

    plt.grid(True)
    plt.grid(which='minor', linestyle=':')

    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plot velocity data from a bag file.')
    parser.add_argument('bag_file', type=str, help='Input bag file.')
    args = parser.parse_args()
    
    main(args.bag_file)