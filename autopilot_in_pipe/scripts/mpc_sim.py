from cProfile import label
import numpy as np
import do_mpc
from casadi import *
import matplotlib.pyplot as plt
import matplotlib as mpl

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
            * (self.thrust_0 - self.thrust_1 + self.thrust_2 - self.thrust_3),
        )
        ### rotation matrix, ZYX euler angles
        self.R = vertcat(
            horzcat(
                cos(self.yaw) * cos(self.pitch),
                cos(self.yaw) * sin(self.pitch) * sin(self.roll)
                - sin(self.yaw) * cos(self.roll),
                cos(self.yaw) * sin(self.pitch) * cos(self.roll)
                + sin(self.yaw) * sin(self.roll),
            ),
            horzcat(
                sin(self.yaw) * cos(self.pitch),
                sin(self.yaw) * sin(self.pitch) * sin(self.roll)
                + cos(self.yaw) * cos(self.roll),
                sin(self.yaw) * sin(self.pitch) * cos(self.roll)
                - cos(self.yaw) * sin(self.roll),
            ),
            horzcat(
                -sin(self.pitch), cos(self.pitch) * sin(self.roll), cos(self.pitch)
            ),
        )

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
                        sin(self.roll) * tan(self.pitch),
                        cos(self.roll) * tan(self.pitch),
                    ),
                    horzcat(0, cos(self.roll), -sin(self.roll)),
                    horzcat(
                        0,
                        sin(self.roll) / cos(self.pitch),
                        cos(self.roll) / cos(self.pitch),
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
            "n_horizon": 15,
            "t_step": 0.05,
            "n_robust": 0,
            "store_full_solution": True,
            "suppress_ipopt_output": True,
        }
        self.mpc.set_param(**setup_mpc)

        self.mpc.settings.supress_ipopt_output()

        # Define the objective function
        lterm = (
            4 * (self.pos_x**2 + self.pos_y**2 + self.pos_z**2)
            + 0.2 * (self.vel_x**2 + self.vel_y**2 + self.vel_z**2)
            + self.yaw**2
            + 0.1 * (self.ang_vel_x**2 + self.ang_vel_y**2 + self.ang_vel_z**2)
        )

        self.mpc.set_objective(lterm=lterm, mterm=lterm)

        self.mpc.set_rterm(
            thrust_0=0.001, thrust_1=0.001, thrust_2=0.001, thrust_3=0.001
        )

        # Define constraints
        self.mpc.bounds["lower", "_u", "thrust_0"] = 0
        self.mpc.bounds["lower", "_u", "thrust_1"] = 0
        self.mpc.bounds["lower", "_u", "thrust_2"] = 0
        self.mpc.bounds["lower", "_u", "thrust_3"] = 0

        self.mpc.bounds["upper", "_u", "thrust_0"] = 10
        self.mpc.bounds["upper", "_u", "thrust_1"] = 10
        self.mpc.bounds["upper", "_u", "thrust_2"] = 10
        self.mpc.bounds["upper", "_u", "thrust_3"] = 10

        # Define the optimizer
        self.mpc.setup()

    class Simulator:
        def __init__(
            self, mpc: do_mpc.controller.MPC, model: do_mpc.model.Model
        ) -> None:
            self.mpc = mpc
            self.model = model
            self.simulator = do_mpc.simulator.Simulator(self.model)

            self.x0 = np.zeros((12, 1))
            self.x0[0] = 1
            self.x0[2] = -1
            self.u0 = np.zeros((4, 1))

            self.simulator.set_param(t_step=0.05)
            self.simulator.setup()

            self.simulator.x0 = self.x0
            self.mpc.x0 = self.x0

            self.mpc.set_initial_guess()

        def run(self):
            mpc_graphics = do_mpc.graphics.Graphics(self.mpc.data)
            sim_graphics = do_mpc.graphics.Graphics(self.simulator.data)

            fig, ax = plt.subplots(5, sharex=True, figsize=(16, 9))
            fig.align_ylabels()

            for g in [mpc_graphics, sim_graphics]:
                g.add_line(var_type="_x", var_name="pos_x", axis=ax[0], label="x")
                g.add_line(var_type="_x", var_name="pos_y", axis=ax[0], label="y")
                g.add_line(var_type="_x", var_name="pos_z", axis=ax[0], label="z")
                g.add_line(var_type="_x", var_name="vel_x", axis=ax[1], label="x")
                g.add_line(var_type="_x", var_name="vel_y", axis=ax[1], label="y")
                g.add_line(var_type="_x", var_name="vel_z", axis=ax[1], label="z")
                g.add_line(var_type="_x", var_name="roll", axis=ax[2], label="roll")
                g.add_line(var_type="_x", var_name="pitch", axis=ax[2], label="pitch")
                g.add_line(var_type="_x", var_name="yaw", axis=ax[2], label="yaw")
                g.add_line(
                    var_type="_x", var_name="ang_vel_x", axis=ax[3], label="ang_vel_x"
                )
                g.add_line(
                    var_type="_x", var_name="ang_vel_y", axis=ax[3], label="ang_vel_y"
                )
                g.add_line(
                    var_type="_x", var_name="ang_vel_z", axis=ax[3], label="ang_vel_z"
                )

                g.add_line(
                    var_type="_u", var_name="thrust_0", axis=ax[4], label="thrust_0"
                )
                g.add_line(
                    var_type="_u", var_name="thrust_1", axis=ax[4], label="thrust_1"
                )
                g.add_line(
                    var_type="_u", var_name="thrust_2", axis=ax[4], label="thrust_2"
                )
                g.add_line(
                    var_type="_u", var_name="thrust_3", axis=ax[4], label="thrust_3"
                )

            ax[0].set_ylabel("Position [m]")
            ax[1].set_ylabel("Velocity [m/s]")
            ax[2].set_ylabel("Orientation [rad]")
            ax[3].set_ylabel("Angular velocity [rad/s]")
            ax[4].set_ylabel("Thrust [N]")

            ax[0].legend()
            ax[1].legend()
            ax[2].legend()
            ax[3].legend()
            ax[4].legend()

            self.simulator.reset_history()
            self.simulator.x0 = self.x0
            self.mpc.reset_history()

            for i in range(100):
                self.u0 = self.mpc.make_step(self.x0)
                self.x0 = self.simulator.make_step(self.u0)

            mpc_graphics.plot_predictions()
            sim_graphics.plot_results()
            sim_graphics.reset_axes()
            plt.show()


if __name__ == "__main__":
    quadcopter_mpc = QuadcopterMPC()
    simulator = quadcopter_mpc.Simulator(quadcopter_mpc.mpc, quadcopter_mpc.model)
    simulator.run()
