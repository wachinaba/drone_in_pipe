#! /usr/bin/python3

import numpy as np
import do_mpc
from casadi import *
import matplotlib.pyplot as plt
import matplotlib as mpl
import rospy
from rospy import Rate, Subscriber, Publisher
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import AttitudeTarget
from scipy.spatial.transform import Rotation

mpl.rcParams["lines.linewidth"] = 3
mpl.rcParams["axes.grid"] = True


class QuadcopterMPC:
    def __init__(self) -> None:
        # Model definition
        self.model_type = "continuous"
        self.model = do_mpc.model.Model(self.model_type)

        # Define model variables
        """
        self.x_pos = self.model.set_variable(
            var_type="_x", var_name="x_pos", shape=(3, 1)  # x, y, z
        )
        self.x_vel = self.model.set_variable(
            var_type="_x", var_name="x_vel", shape=(3, 1)  # x, y, z
        )
        self.x_ori = self.model.set_variable(
            var_type="_x",
            var_name="x_ori",
            shape=(3, 1),  # euler angles, roll, pitch, yaw
        )
        self.x_ang_vel = self.model.set_variable(
            var_type="_x", var_name="x_ang_vel", shape=(3, 1)  # x, y, z
        )

        self.u_thrust = self.model.set_variable(
            var_type="_u", var_name="u_thrust", shape=(4, 1)  # 4 rotors thrust
        )
        """

        self.pos_x = self.model.set_variable(
            var_type="_x", var_name="pos_x", shape=(1, 1)  # x
        )
        self.pos_y = self.model.set_variable(
            var_type="_x", var_name="pos_y", shape=(1, 1)  # y
        )
        self.pos_z = self.model.set_variable(
            var_type="_x", var_name="pos_z", shape=(1, 1)  # z
        )
        self.vel_x = self.model.set_variable(
            var_type="_x", var_name="vel_x", shape=(1, 1)  # x
        )
        self.vel_y = self.model.set_variable(
            var_type="_x", var_name="vel_y", shape=(1, 1)  # y
        )
        self.vel_z = self.model.set_variable(
            var_type="_x", var_name="vel_z", shape=(1, 1)  # z
        )
        self.roll = self.model.set_variable(
            var_type="_x", var_name="roll", shape=(1, 1)  # x
        )
        self.pitch = self.model.set_variable(
            var_type="_x", var_name="pitch", shape=(1, 1)  # y
        )
        self.yaw = self.model.set_variable(
            var_type="_x", var_name="yaw", shape=(1, 1)  # z
        )
        self.ang_vel_x = self.model.set_variable(
            var_type="_x", var_name="ang_vel_x", shape=(1, 1)  # x
        )
        self.ang_vel_y = self.model.set_variable(
            var_type="_x", var_name="ang_vel_y", shape=(1, 1)  # y
        )
        self.ang_vel_z = self.model.set_variable(
            var_type="_x", var_name="ang_vel_z", shape=(1, 1)  # z
        )

        self.target_pos_x = self.model.set_variable(
            var_type="_tvp", var_name="target_pos_x", shape=(1, 1)  # x
        )
        self.target_pos_y = self.model.set_variable(
            var_type="_tvp", var_name="target_pos_y", shape=(1, 1)  # y
        )
        self.target_pos_z = self.model.set_variable(
            var_type="_tvp", var_name="target_pos_z", shape=(1, 1)  # z
        )
        self.target_yaw = self.model.set_variable(
            var_type="_tvp", var_name="target_yaw", shape=(1, 1)  # yaw
        )

        self.thrust_0 = self.model.set_variable(
            var_type="_u", var_name="thrust_0", shape=(1, 1)  # 4 rotors thrust
        )
        self.thrust_1 = self.model.set_variable(
            var_type="_u", var_name="thrust_1", shape=(1, 1)  # 4 rotors thrust
        )
        self.thrust_2 = self.model.set_variable(
            var_type="_u", var_name="thrust_2", shape=(1, 1)  # 4 rotors thrust
        )
        self.thrust_3 = self.model.set_variable(
            var_type="_u", var_name="thrust_3", shape=(1, 1)  # 4 rotors thrust
        )

        # Define model parameters
        self.m = 1.6  # mass
        self.g = vertcat(0, 0, -9.81)  # gravity
        self.I = vertcat(
            horzcat(0.029125, 0, 0),
            horzcat(0, 0.029125, 0),
            horzcat(0, 0, 0.055225),
        )
        self.motor_torque_constant = 0.01
        self.motor_position = vertcat(0.0989, 0.1209)  # distance from center of mass

        # Define model equations
        ## define vectors, matrices
        pos = vertcat(self.pos_x, self.pos_y, self.pos_z)
        vel = vertcat(self.vel_x, self.vel_y, self.vel_z)
        ori = vertcat(self.roll, self.pitch, self.yaw)
        ang_vel = vertcat(self.ang_vel_x, self.ang_vel_y, self.ang_vel_z)
        thrust_sum = self.thrust_0 + self.thrust_1 + self.thrust_2 + self.thrust_3
        thrust = vertcat(0, 0, thrust_sum)
        tau = vertcat(
            self.motor_position[1]
            * (-self.thrust_0 - self.thrust_1 + self.thrust_2 + self.thrust_3),
            self.motor_position[0]
            * (-self.thrust_0 + self.thrust_1 + self.thrust_2 - self.thrust_3),
            self.motor_torque_constant
            * (-self.thrust_0 + self.thrust_1 - self.thrust_2 + self.thrust_3),
        )
        ### rotation matrix, ZYX euler angles
        self.R = inv(vertcat(
            horzcat(
                cos(self.pitch) * cos(self.yaw),
                cos(self.pitch) * sin(self.yaw),
                -sin(self.pitch),
            ),
            horzcat(
                sin(self.roll) * sin(self.pitch) * cos(self.yaw)
                - cos(self.roll) * sin(self.yaw),
                sin(self.roll) * sin(self.pitch) * sin(self.yaw)
                + cos(self.roll) * cos(self.yaw),
                sin(self.roll) * cos(self.pitch),
            ),
            horzcat(
                cos(self.roll) * sin(self.pitch) * cos(self.yaw)
                + sin(self.roll) * sin(self.yaw),
                cos(self.roll) * sin(self.pitch) * sin(self.yaw)
                - sin(self.roll) * cos(self.yaw),
                cos(self.roll) * cos(self.pitch),
            ),
        ))

        ## define equations
        d_pos = vel
        self.model.set_rhs("pos_x", d_pos[0])
        self.model.set_rhs("pos_y", d_pos[1])
        self.model.set_rhs("pos_z", d_pos[2])

        d_vel = self.g + (1 / self.m) * mtimes(self.R, thrust)
        self.model.set_rhs("vel_x", d_vel[0])
        self.model.set_rhs("vel_y", d_vel[1])
        self.model.set_rhs("vel_z", d_vel[2])

        d_ori = mtimes(
            inv(
                vertcat(
                    horzcat(
                        1,
                        0,
                        -sin(self.pitch),
                    ),
                    horzcat(0, cos(self.yaw), cos(self.pitch) * sin(self.yaw)),
                    horzcat(
                        0,
                        -sin(self.yaw),
                        cos(self.pitch) * cos(self.yaw),
                    ),
                )
            ),
            ang_vel,
        )
        self.model.set_rhs("roll", d_ori[0])
        self.model.set_rhs("pitch", d_ori[1])
        self.model.set_rhs("yaw", d_ori[2])

        d_ang_vel = mtimes(inv(self.I), tau - cross(ang_vel, mtimes(self.I, ang_vel)))
        self.model.set_rhs("ang_vel_x", d_ang_vel[0])
        self.model.set_rhs("ang_vel_y", d_ang_vel[1])
        self.model.set_rhs("ang_vel_z", d_ang_vel[2])

        # Define model outputs
        self.model.setup()

        # Define the controller
        self.mpc = do_mpc.controller.MPC(self.model)

        setup_mpc = {
            "n_horizon": 20,
            "t_step": 0.04,
            "n_robust": 0,
            "store_full_solution": True,
            "suppress_ipopt_output": True,
        }
        self.mpc.set_param(**setup_mpc)

        self.mpc.settings.supress_ipopt_output()

        self.tvp_template = self.mpc.get_tvp_template()
        self.tvp_template["_tvp", :] = np.array([0, 0, 0, 0])

        def tvp_fun(t_now):
            for k in range(self.mpc.settings.n_horizon + 1):
                self.tvp_template["_tvp", k, "target_pos_x"] = 0.1
                self.tvp_template["_tvp", k, "target_pos_y"] = 0
                self.tvp_template["_tvp", k, "target_pos_z"] = 0
                self.tvp_template["_tvp", k, "target_yaw"] = 0
            return self.tvp_template

        self.mpc.set_tvp_fun(tvp_fun)

        # Define the objective function
        lterm = (
            4
            * (
                (self.model.x["pos_x"] - self.model.tvp["target_pos_x"]) ** 2
                + (self.model.x["pos_y"] - self.model.tvp["target_pos_y"]) ** 2
                + (self.model.x["pos_z"] - self.model.tvp["target_pos_z"]) ** 2
            )
            + 1 * (self.vel_x**2 + self.vel_y**2 + self.vel_z**2)
            + 2 * (self.model.x["yaw"] - self.model.tvp["target_yaw"]) ** 2
            + 0.2 * (self.ang_vel_x**2 + self.ang_vel_y**2 + self.ang_vel_z**2)
        )

        self.mpc.set_objective(lterm=lterm, mterm=lterm)

        self.mpc.set_rterm(
            thrust_0=0.01, thrust_1=0.01, thrust_2=0.01, thrust_3=0.01
        )

        # Define constraints
        self.mpc.bounds["lower", "_u", "thrust_0"] = 0
        self.mpc.bounds["lower", "_u", "thrust_1"] = 0
        self.mpc.bounds["lower", "_u", "thrust_2"] = 0
        self.mpc.bounds["lower", "_u", "thrust_3"] = 0

        self.mpc.bounds["upper", "_u", "thrust_0"] = 12
        self.mpc.bounds["upper", "_u", "thrust_1"] = 12
        self.mpc.bounds["upper", "_u", "thrust_2"] = 12
        self.mpc.bounds["upper", "_u", "thrust_3"] = 12

        self.mpc.bounds["lower", "_x", "roll"] = -pi / 5
        self.mpc.bounds["upper", "_x", "roll"] = pi / 5
        self.mpc.bounds["lower", "_x", "pitch"] = -pi / 5
        self.mpc.bounds["upper", "_x", "pitch"] = pi / 5
        self.mpc.bounds["lower", "_x", "yaw"] = -pi / 3
        self.mpc.bounds["upper", "_x", "yaw"] = pi / 3

        # Define the optimizer
        self.mpc.setup()


class MPCNode:
    def __init__(self) -> None:
        self.pose = PoseStamped()
        self.velocity = TwistStamped()
        self.imu = Imu()
        self.orientation = np.zeros(3)  # yaw, pitch, roll
        self.u0 = np.zeros(4)
        self.x0 = np.zeros(12)

        self.max_thrust = rospy.get_param("~max_thrust", 92.6)

        self.quadcopter_mpc = QuadcopterMPC()
        self.initialized = False

        self.pose_sub = Subscriber("/global_pose", PoseStamped, self.pose_callback)
        self.vel_sub = Subscriber(
            "/global_velocity",
            TwistStamped,
            self.velocity_callback,
        )
        self.imu_sub = Subscriber("/imu", Imu, self.imu_callback)
        self.target_attitude_pub = Publisher(
            "/target_attitude", AttitudeTarget, queue_size=1
        )

    def pose_callback(self, msg: PoseStamped) -> None:
        self.pose = msg
        pipe_orientation = Rotation.from_quat(
            [
                self.pose.pose.orientation.x,
                self.pose.pose.orientation.y,
                self.pose.pose.orientation.z,
                self.pose.pose.orientation.w,
            ]
        ).as_euler("ZYX")
        rospy.logerr(f"orientation: {self.orientation}")
        self.orientation[0] = pipe_orientation[0]

    def velocity_callback(self, msg: TwistStamped) -> None:
        self.velocity = msg

    def imu_callback(self, msg: Imu) -> None:
        self.imu = msg
        body_orientation = Rotation.from_quat(
            [
                self.imu.orientation.x,
                self.imu.orientation.y,
                self.imu.orientation.z,
                self.imu.orientation.w,
            ]
        ).as_euler("ZYX")
        self.orientation[1] = body_orientation[1]
        self.orientation[2] = body_orientation[2]

    def step(self) -> None:
        # Set initial state
        self.x0 = np.array(
            [
                self.pose.pose.position.x,
                self.pose.pose.position.y,
                self.pose.pose.position.z,
                self.velocity.twist.linear.x,
                self.velocity.twist.linear.y,
                self.velocity.twist.linear.z,
                self.orientation[2],
                self.orientation[1],
                self.orientation[0],
                self.imu.angular_velocity.x,
                self.imu.angular_velocity.y,
                self.imu.angular_velocity.z,
            ]
        )

        if not self.initialized:
            self.quadcopter_mpc.mpc.x0 = self.x0
            self.quadcopter_mpc.mpc.set_initial_guess()
            self.initialized = True

        # Solve MPC
        self.u0 = self.quadcopter_mpc.mpc.make_step(self.x0)

        # get optimal angular velocity
        self.target_attitude = AttitudeTarget()
        self.target_attitude.header.stamp = rospy.Time.now()

        time_horizon = 2

        self.target_attitude.body_rate.x = self.quadcopter_mpc.mpc.data.prediction(
            ("_x", "ang_vel_x", 0)
        )[0, time_horizon, 0]
        self.target_attitude.body_rate.y = self.quadcopter_mpc.mpc.data.prediction(
            ("_x", "ang_vel_y", 0)
        )[0, time_horizon, 0]
        self.target_attitude.body_rate.z = self.quadcopter_mpc.mpc.data.prediction(
            ("_x", "ang_vel_z", 0)
        )[0, time_horizon, 0]

        # get optimal thrust
        self.target_attitude.thrust = sqrt((
            self.quadcopter_mpc.mpc.data.prediction(("_u", "thrust_0", 0))[0, time_horizon, 0]
            + self.quadcopter_mpc.mpc.data.prediction(("_u", "thrust_1", 0))[0, time_horizon, 0]
            + self.quadcopter_mpc.mpc.data.prediction(("_u", "thrust_2", 0))[0, time_horizon, 0]
            + self.quadcopter_mpc.mpc.data.prediction(("_u", "thrust_3", 0))[0, time_horizon, 0]
        ) / self.max_thrust)

        self.target_attitude.type_mask = AttitudeTarget.IGNORE_ATTITUDE

        self.target_attitude_pub.publish(self.target_attitude)


if __name__ == "__main__":
    rospy.init_node("mpc_node", anonymous=True)

    rate = Rate(20)  # mpc rate
    mpc_node = MPCNode()

    while not rospy.is_shutdown():
        mpc_node.step()
        rate.sleep()
