import argparse

import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation


def main(args):
    df = pd.read_csv(args.filename)
    df["No"] = range(0, len(df.index))

    # calc delta time
    df["delta_time"] = df["flow_time"].diff()

    # calc mocap velocity
    df["mocap_vx"] = df["mocap_x"].diff() / df["delta_time"]
    df["mocap_vy"] = df["mocap_y"].diff() / df["delta_time"]
    df["mocap_vz"] = df["mocap_z"].diff() / df["delta_time"]

    # calc mocap velocity magnitude
    df["mocap_v"] = np.sqrt(
        df["mocap_vx"] ** 2 + df["mocap_vy"] ** 2 + df["mocap_vz"] ** 2
    )

    # calc mocap angular velocity from quaternion
    def compute_angular_velocity(q1, q2, delta_time):
        # scipyのRotationオブジェクトを使用してクォータニオンの差分を計算
        r1 = Rotation.from_quat(q1)
        r2 = Rotation.from_quat(q2)
        delta_r = r2 * r1.inv()

        # 角速度を計算
        angle = delta_r.magnitude()
        axis = delta_r.as_rotvec() / angle
        angular_velocity = axis * angle / delta_time

        return angular_velocity

    # 角速度を計算してデータフレームに追加
    angular_velocities = np.vstack(
        [
            compute_angular_velocity(
                df.iloc[i - 1][["qx", "qy", "qz", "qw"]].values,
                df.iloc[i][["qx", "qy", "qz", "qw"]].values,
                df.iloc[i]["delta_time"],
            )
            for i in range(1, len(df))
        ]
    )
    df.loc[1:, ["wx", "wy", "wz"]] = angular_velocities
    df.loc[0, ["wx", "wy", "wz"]] = [0, 0, 0]


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("filename", help="csv file to be converted")
    args = parser.parse_args()
    main(args)
