#pragma once

#include <ct/core/core.h>

namespace ct
{
namespace core
{
constexpr size_t kStateDim = 6;
constexpr size_t kControlDim = 2;

//! Describes a drone system
/* This describes a general, non-linear dynamic system described by an Ordinary Differential Equation (ODE)

f(x,u,t) = [p_dot, q_dot, v_dot, w_dot]^T = [v, q * [0, w/2]^T, 1/m * q * f * q^-1 + g, 1/J * (tau - w x J * w)]^T

where x(t) is the state, u(t) the control input (thrust) and t the time.

*/

template <typename SCALAR = double>
class AdrienSystem : public ControlledSystem<kStateDim, kControlDim, SCALAR>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  AdrienSystem()
    : ControlledSystem<kStateDim, kControlDim, SCALAR>()
    , inertia_(Eigen::Matrix3d::Identity())
    , mass_(1.0)
    , torque_coefficient_(1.0)
    , gravity_(9.81)
  {
  }

  AdrienSystem(std::shared_ptr<Controller<kStateDim, kControlDim, SCALAR>> controller)
    : ControlledSystem<kStateDim, kControlDim, SCALAR>(controller)
  {
  }

  AdrienSystem(const AdrienSystem& arg) : ControlledSystem<kStateDim, kControlDim, SCALAR>(arg)
  {
  }

  ~AdrienSystem() override = default;

  AdrienSystem* clone() const override
  {
    return new AdrienSystem(*this);
  }

  virtual void computeControlledDynamics(const StateVector<kStateDim, SCALAR>& state, const time_t& t,
                                         const ControlVector<kControlDim, SCALAR>& control,
                                         StateVector<kStateDim, SCALAR>& derivative) override
  {
    // state: [p, q, v, w]^T
    // control: [thrust, torque]^T
    // derivative: [p_dot, q_dot, v_dot, w_dot]^T

    derivative[0] = state[7];  // p_dot = v
    derivative[1] = state[8];
    derivative[2] = state[9];

    Eigen::Quaterniond q(state[3], state[4], state[5], state[6]);
    Eigen::Vector3<SCALAR> w(state[10], state[11], state[12]);
    Eigen::Vector3<SCALAR> f(0, 0, control.sum());
    Eigen::Vector3<SCALAR> g(0, 0, -gravity_);

    Eigen::Quaterniond w_q = Eigen::Quaterniond(0, w[0] / 2, w[1] / 2, w[2] / 2);
    Eigen::Quaterniond q_dot = q * w_q;

    derivative[3] = q_dot.w();
    derivative[4] = q_dot.x();
    derivative[5] = q_dot.y();
    derivative[6] = q_dot.z();

    Eigen::Vector3<SCALAR> v_dot = 1.0 / mass_ * (q * f * q.inverse()) + g;

    derivative[7] = v_dot[0];
    derivative[8] = v_dot[1];
    derivative[9] = v_dot[2];

    Eigen::Vector3<SCALAR> tau(motor_position_[1] * (-control[0] - control[1] + control[2] + control[3]),
                               motor_position_[0] * (-control[0] + control[1] + control[2] - control[3]),
                               torque_coefficient_ * (control[0] - control[1] + control[2] - control[3]));

    Eigen::Vector3<SCALAR> w_dot = inertia_inv_ * (tau - w.cross(inertia_ * w));

    derivative[10] = w_dot[0];
    derivative[11] = w_dot[1];
    derivative[12] = w_dot[2];
  }

protected:
  Eigen::Matrix3d inertia_;
  Eigen::Vector3<SCALAR> inertia_inv_;
  SCALAR mass_;
  SCALAR torque_coefficient_;
  SCALAR gravity_;
  Eigen::Vector2d motor_position_;
};

}  // namespace core
}  // namespace ct