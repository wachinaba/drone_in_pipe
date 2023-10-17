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
        # calc quaternion delta and convert to rotation
        r1 = Rotation.from_quat(q1)
        r2 = Rotation.from_quat(q2)
        delta_r = r2 * r1.inv()

        # calc angular velocity
        angular_velocity = delta_r.as_rotvec() / delta_time

        return angular_velocity

    # calc mocap angular velocity from quaternion, and append to dataframe
    angular_velocities = np.vstack(
        [
            compute_angular_velocity(
                df.iloc[i - 1][["mocap_qx", "mocap_qy", "mocap_qz", "mocap_qw"]].values,
                df.iloc[i][["mocap_qx", "mocap_qy", "mocap_qz", "mocap_qw"]].values,
                df.iloc[i]["delta_time"],
            )
            for i in range(1, len(df))
        ]
    )
    df.loc[1:, ["mocap_wx", "mocap_wy", "mocap_wz"]] = angular_velocities
    df.loc[0, ["mocap_wx", "mocap_wy", "mocap_wz"]] = [0, 0, 0]

    def compute_actual_measured_velocity(
        w: np.ndarray, r: float, v: np.ndarray, q: Rotation
    ):
        # calc camera axis
        camera_axis = q.apply([0, 0, -1])

        # calc distance vector
        distance_vector = r * camera_axis

        # calc circumferential velocity
        circumferential_velocity = np.cross(w, distance_vector)

        # calc actual velocity
        actual_velocity = v + circumferential_velocity

        # calc measured velocity by removing camera axis component
        measured_velocity = (
            actual_velocity - np.dot(actual_velocity, camera_axis) * camera_axis
        )

        return measured_velocity

    # calc actual measured velocity, and append to dataframe
    actual_measured_velocities = np.vstack(
        [
            compute_actual_measured_velocity(
                df.iloc[i][["mocap_wx", "mocap_wy", "mocap_wz"]].values,
                df.iloc[i]["range"],
                df.iloc[i][["mocap_vx", "mocap_vy", "mocap_vz"]].values,
                Rotation.from_quat(
                    df.iloc[i][["mocap_qx", "mocap_qy", "mocap_qz", "mocap_qw"]].values
                ),
            )
            for i in range(len(df))
        ]
    )
    df[["flow_vx", "flow_vy", "flow_vz"]] = actual_measured_velocities

    # calc norm of measured velocity
    df["flow_v"] = np.sqrt(df["flow_vx"] ** 2 + df["flow_vy"] ** 2 + df["flow_vz"] ** 2)

    # calc optical flow pixel delta magnitude
    df["flow_p"] = np.sqrt(df["flow_px"] ** 2 + df["flow_py"] ** 2)

    # calc scale factor K
    df["K"] = df["flow_v"] / (df["flow_p"] * df["range"])

    # save to csv
    df.to_csv(args.filename + ".converted.csv", index=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("filename", help="csv file to be converted")
    args = parser.parse_args()
    main(args)
