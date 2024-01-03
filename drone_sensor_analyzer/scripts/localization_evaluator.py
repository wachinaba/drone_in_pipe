import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R

import rosbag
import argparse

def read_topic(bag, topic):
    messages = []
    for topic, msg, t in bag.read_messages(topics=topic):
        time = msg.header.stamp.to_sec()
        position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        messages.append([time] + position)
    return np.array(messages)

def read_orientation_ypr(bag, topic):
    messages = []
    for topic, msg, t in bag.read_messages(topics=topic):
        time = msg.header.stamp.to_sec()
        orientation = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]).as_euler('ZYX', degrees=True)
        messages.append([time] + orientation.tolist())
    return np.array(messages)

def synchronize_topics(topic1, topic2):
    synced_data = []
    index2 = 0
    for data1 in topic1:
        time1 = data1[0]
        closest_index = np.argmin(np.abs(topic2[index2:, 0] - time1))
        closest_time = topic2[closest_index + index2, 0]
        if abs(closest_time - time1) < 0.1:  # 100ms threshold, adjust as necessary
            synced_data.append(data1.tolist() + topic2[closest_index + index2].tolist()[1:])
            index2 += closest_index
    return np.array(synced_data)

def rpy_to_matrix(roll, pitch, yaw):
    # RPYから回転行列への変換
    return R.from_euler('ZYX', [yaw, pitch, roll], degrees=False).as_matrix()

def transform(points, rotation_matrix, translation):
    # 3D点群に回転と平行移動の変換を適用
    transformed_points = rotation_matrix.dot(points.T).T + translation
    # X軸成分を除去
    transformed_points[:, 0] = 0
    return transformed_points

def objective_function(params, source_points, target_points):
    # クォータニオンと平行移動ベクトルをパラメータから取り出す
    
    translation = [0, params[3], params[4]]  # X軸成分は除去
    
    # RPYから回転行列への変換
    rotation_matrix = rpy_to_matrix(params[0], params[1], params[2])
    
    # 変換を適用
    transformed_points = transform(source_points, rotation_matrix, translation)
    
    # 平均二乗誤差を計算
    mse = np.mean((transformed_points - target_points) ** 2)
    return mse

def align_point_clouds(source_points, target_points):
    # 初期パラメータ（RPY ,Y, Z軸方向の平行移動）
    initial_params = np.array([0, 0, 0, 0, 0])  # RPY, Y, Z

    # 最適化
    result = minimize(
        objective_function, 
        initial_params, 
        args=(source_points, target_points),
        method='L-BFGS-B'  # 有界制約を持つ問題に適したアルゴリズム
    )
    
    # 最適化結果を取得
    optimized_params = result.x
    optimized_rpy = optimized_params[:3]
    optimized_translation = [0, optimized_params[3], optimized_params[4]]  # X軸成分は除去
    optimized_rotation_matrix = rpy_to_matrix(*optimized_rpy)

    print(result)

    print("rpy in degree:", np.rad2deg(optimized_rpy))
    
    return optimized_rotation_matrix, optimized_translation


def main(bag_file):
    bag = rosbag.Bag(bag_file)
    drone_world_data = read_topic(bag, '/drone/world')
    pose_data = read_topic(bag, '/pose')
    orientation_pose_data = read_orientation_ypr(bag, '/pose')
    synced_data = synchronize_topics(pose_data, drone_world_data)
    bag.close()
    return synced_data, orientation_pose_data

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Extract and synchronize topics from a ROS bag file.")
    parser.add_argument("bag_file", help="Path to the ROS bag file")
    args = parser.parse_args()

    synced_data, orientation_pose_data = main(args.bag_file)

    synced_data[:, 0] -= synced_data[0, 0]  # Adjust time to start at 0

    print("Synchronized data shape:", synced_data.shape)
    print("Orientation data shape:", orientation_pose_data.shape)
    print("Sample data:", synced_data[:5])  # Print first 5 rows for preview

    np.savetxt(f"{args.bag_file}_synced.csv", synced_data, delimiter=",")  # Save as CSV file

    target_points = synced_data[:, 1:4]
    source_points = synced_data[:, 4:7]
    print("Source points shape:", source_points.shape)
    print("Target points shape:", target_points.shape)

    rotation_matrix, translation = align_point_clouds(source_points, target_points)

    print("Rotation matrix:")
    print(rotation_matrix)
    print("Translation:")
    print(translation)

    # 3D点群に回転と平行移動の変換を適用
    transformed_points = transform(source_points, rotation_matrix, translation)
    print("Transformed points shape:", transformed_points.shape)

    # 平均二乗誤差を計算
    mse = np.mean((transformed_points - target_points)[:, 1:] ** 2)

    print("Mean squared error:", mse)

    print("mean absolute error:", np.mean(np.abs(transformed_points - target_points)[:, 1:]))

    print("max absolute error:", np.max(np.abs(transformed_points - target_points)[:, 1:]))

    # Y-tグラフ, Z-tグラフを出力
    import matplotlib.pyplot as plt
    plt.figure(figsize=(12, 4))
    plt.subplot(3, 1, 1)
    plt.plot(synced_data[:, 0], synced_data[:, 2], label="estimated (Y)")
    plt.plot(synced_data[:, 0], transformed_points[:, 1], label="actual (Y)")

    plt.subplot(3, 1, 2)
    plt.plot(synced_data[:, 0], synced_data[:, 3], label="estimated (Z)")
    plt.plot(synced_data[:, 0], transformed_points[:, 2], label="actual (Z)")

    plt.subplot(3, 1, 3)
    plt.plot(synced_data[:, 0], orientation_pose_data[:, 1], label="estimated (Yaw)")
    plt.plot(synced_data[:, 0], orientation_pose_data[:, 2], label="estimated (Pitch)")
    plt.plot(synced_data[:, 0], orientation_pose_data[:, 3], label="estimated (Roll)")

    plt.legend()
    
    plt.show()

    # 3D点群を出力(タイムスタンプ + ソース + ターゲット + 変換後 + オリエンテーション)
    output_points = np.hstack((synced_data[:, 0].reshape(-1, 1), source_points, target_points, transformed_points, orientation_pose_data[:, 1:]))

    np.savetxt(f"{args.bag_file}_output.csv", output_points, delimiter=",")  # Save as CSV file