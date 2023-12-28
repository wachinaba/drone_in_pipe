import re
import numpy as np
from numpy import sin, cos, arccos, pi, source
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R

import rosbag
import argparse

def read_topic(bag, topic):
    messages = []
    for topic, msg, t in bag.read_messages(topics=topic):
        time = t.to_sec() # Convert ROS Time to time elapsed since start, in seconds
        position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        messages.append([time] + position)
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

def quaternion_to_matrix(quaternion):
    # クォータニオンから回転行列への変換
    return R.from_quat(quaternion).as_matrix()

def normalize_quaternion(quaternion):
    # クォータニオンを正規化
    return quaternion / np.linalg.norm(quaternion)

def transform(points, rotation_matrix, translation):
    # 3D点群に回転と平行移動の変換を適用
    transformed_points = rotation_matrix.dot(points.T).T + translation
    # X軸成分を除去
    transformed_points[:, 0] = 0
    return transformed_points

def objective_function(params, source_points, target_points):
    # クォータニオンと平行移動ベクトルをパラメータから取り出す
    quaternion = normalize_quaternion(params[:4])  # 正規化を確実に行う
    translation = [0, params[4], params[5]]  # X軸成分は除去
    
    # クォータニオンを回転行列に変換
    rotation_matrix = quaternion_to_matrix(quaternion)
    
    # 変換を適用
    transformed_points = transform(source_points, rotation_matrix, translation)
    
    # 平均二乗誤差を計算
    mse = np.mean((transformed_points - target_points) ** 2)
    return mse

def align_point_clouds(source_points, target_points):
    # 初期パラメータ（クォータニオン + 平行移動ベクトル）
    initial_params = np.array([1, 0, 0, 0, 0, 0])  # 単位クォータニオンとゼロ平行移動(Y, Z軸方向のみ考慮)

    # 最適化
    result = minimize(
        objective_function, 
        initial_params, 
        args=(source_points, target_points),
        method='L-BFGS-B'  # 有界制約を持つ問題に適したアルゴリズム
    )
    
    # 最適化結果を取得
    optimized_params = result.x
    optimized_quaternion = normalize_quaternion(optimized_params[:4])  # 結果も正規化
    optimized_translation = [0, optimized_params[4], optimized_params[5]]  # X軸成分は除去
    optimized_rotation_matrix = quaternion_to_matrix(optimized_quaternion)

    print(result)
    
    return optimized_rotation_matrix, optimized_translation


def main(bag_file):
    bag = rosbag.Bag(bag_file)
    drone_world_data = read_topic(bag, '/drone/world')
    pose_data = read_topic(bag, '/pose')
    synced_data = synchronize_topics(pose_data, drone_world_data)
    bag.close()
    return synced_data

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Extract and synchronize topics from a ROS bag file.")
    parser.add_argument("bag_file", help="Path to the ROS bag file")
    args = parser.parse_args()

    synced_data = main(args.bag_file)

    synced_data[:, 0] -= synced_data[0, 0]  # Adjust time to start at 0

    print("Synchronized data shape:", synced_data.shape)
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
    mse = np.mean((transformed_points - target_points) ** 2)

    print("Mean squared error:", mse)

    # 3D点群を出力(タイムスタンプ + ソース + ターゲット + 変換後)
    output_points = np.hstack((synced_data[:, 0:4], target_points, transformed_points))

    np.savetxt(f"{args.bag_file}_output.csv", output_points, delimiter=",")  # Save as CSV file