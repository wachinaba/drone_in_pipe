#ifndef MOVE_EKF_H
#define MOVE_EKF_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <kalman/SystemModel.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>

namespace MoveUKF
{
template <typename T>
class State : public Kalman::Vector<T, 3>
{
public:
  KALMAN_VECTOR(State, T, 3)

  //! Velocity in x-direction
  static constexpr size_t VEL_X = 0;
  //! Velocity in y-direction
  static constexpr size_t VEL_Y = 1;
  //! Velocity in z-direction
  static constexpr size_t VEL_Z = 2;

  T vel_x() const
  {
    return (*this)[VEL_X];
  }
  T vel_y() const
  {
    return (*this)[VEL_Y];
  }
  T vel_z() const
  {
    return (*this)[VEL_Z];
  }

  T& vel_x()
  {
    return (*this)[VEL_X];
  }
  T& vel_y()
  {
    return (*this)[VEL_Y];
  }
  T& vel_z()
  {
    return (*this)[VEL_Z];
  }
};

template <typename T>
class Control : public Kalman::Vector<T, 6>
{
public:
  KALMAN_VECTOR(Control, T, 6)

  //! Acceleration in x-direction
  static constexpr size_t ACC_X = 0;
  //! Acceleration in y-direction
  static constexpr size_t ACC_Y = 1;
  //! Acceleration in z-direction
  static constexpr size_t ACC_Z = 2;

  //! Angular velocity in x-direction
  static constexpr size_t ANGULAR_VEL_X = 3;
  //! Angular velocity in y-direction
  static constexpr size_t ANGULAR_VEL_Y = 4;
  //! Angular velocity in z-direction
  static constexpr size_t ANGULAR_VEL_Z = 5;

  T acc_x() const
  {
    return (*this)[ACC_X];
  }
  T acc_y() const
  {
    return (*this)[ACC_Y];
  }
  T acc_z() const
  {
    return (*this)[ACC_Z];
  }

  T angular_vel_x() const
  {
    return (*this)[ANGULAR_VEL_X];
  }
  T angular_vel_y() const
  {
    return (*this)[ANGULAR_VEL_Y];
  }
  T angular_vel_z() const
  {
    return (*this)[ANGULAR_VEL_Z];
  }

  T& acc_x()
  {
    return (*this)[ACC_X];
  }
  T& acc_y()
  {
    return (*this)[ACC_Y];
  }
  T& acc_z()
  {
    return (*this)[ACC_Z];
  }

  T& angular_vel_x()
  {
    return (*this)[ANGULAR_VEL_X];
  }
  T& angular_vel_y()
  {
    return (*this)[ANGULAR_VEL_Y];
  }
  T& angular_vel_z()
  {
    return (*this)[ANGULAR_VEL_Z];
  }
};

template <typename T, template <class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::SystemModel<State<T>, Control<T>, CovarianceBase>
{
public:
  //! State type shortcut definition
  typedef MoveUKF::State<T> S;
  //! Control type shortcut definition
  typedef MoveUKF::Control<T> C;

  S f(const S& x, const C& u) const override
  {
    S x_;

    Eigen::Quaternion<T> q_x_ = Eigen::Quaternion<T>(0, x.vel_x(), x.vel_y(), x.vel_z());

    auto angular_velocity = Eigen::Vector3d(u.angular_vel_x(), u.angular_vel_y(), u.angular_vel_z());
    T norm = angular_velocity.norm();
    T angle_half = norm / 2;

    if (norm > 1e-6) { // avoid division by zero
      Eigen::Quaternion<T> q_delta(cos(angle_half), sin(angle_half) * u.angular_vel_x() / norm,
                                  sin(angle_half) * u.angular_vel_y() / norm,
                                  sin(angle_half) * u.angular_vel_z() / norm);

      auto rotated_velocity = q_delta.conjugate() * q_x_ * q_delta;
      x_.vel_x() = rotated_velocity.x();
      x_.vel_y() = rotated_velocity.y();
      x_.vel_z() = rotated_velocity.z();
    }

    x_.vel_x() += u.acc_x();
    x_.vel_y() += u.acc_y();
    x_.vel_z() += u.acc_z();
    return x_;
  }
};

template <typename T>
class Measurement : public Kalman::Vector<T, 3>
{
public:
  KALMAN_VECTOR(Measurement, T, 3)

  //! Measured velocity in x-direction
  static constexpr size_t MEASURED_VEL_X = 0;
  //! Measured velocity in y-direction
  static constexpr size_t MEASURED_VEL_Y = 1;
  //! Measured velocity in z-direction
  static constexpr size_t MEASURED_VEL_Z = 2;

  T measured_vel_x() const
  {
    return (*this)[MEASURED_VEL_X];
  }
  T measured_vel_y() const
  {
    return (*this)[MEASURED_VEL_Y];
  }
  T measured_vel_z() const
  {
    return (*this)[MEASURED_VEL_Z];
  }

  T& measured_vel_x()
  {
    return (*this)[MEASURED_VEL_X];
  }
  T& measured_vel_y()
  {
    return (*this)[MEASURED_VEL_Y];
  }
  T& measured_vel_z()
  {
    return (*this)[MEASURED_VEL_Z];
  }
};

template <typename T, template <class> class CovarianceBase = Kalman::StandardBase>
class MeasurementModel : public Kalman::MeasurementModel<State<T>, Measurement<T>, CovarianceBase>
{
public:
  //! State type shortcut definition
  typedef MoveUKF::State<T> S;

  //! Measurement type shortcut definition
  typedef MoveUKF::Measurement<T> M;

  M h(const S& x) const override
  {
    M velocity_2d;
    M velocity_3d = x + angular_velocity_.cross(range_vector_);

    velocity_2d = velocity_3d - velocity_3d.dot(camera_z_axis_) * camera_z_axis_;
    return velocity_2d;
  }

  void setSensorStatus(const Kalman::Vector<T, 3>& angular_velocity, const Kalman::Vector<T, 3>& range_vector,
                       const Kalman::Vector<T, 3>& camera_z_axis)
  {
    angular_velocity_ = angular_velocity;
    range_vector_ = range_vector;
    camera_z_axis_ = camera_z_axis;
  }

protected:
  Kalman::Vector<T, 3> angular_velocity_;
  Kalman::Vector<T, 3> range_vector_;
  Kalman::Vector<T, 3> camera_z_axis_;
};

}  // namespace MoveUKF

#endif  // MOVE_EKF_H