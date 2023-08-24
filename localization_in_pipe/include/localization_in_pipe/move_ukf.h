#ifndef MOSE_EKF_H
#define MOSE_EKF_H

#include <ros/ros.h>
#include <Eigen/Dense>

#include <ukf_ros/core.h>
#include <ukf_ros/state_vector.h>
#include <ukf_ros/measurement_vector.h>
#include <ukf_ros/integrator.h>
#include <ukf_ros/core.h>

enum StateFields
{
  Velocity
};

enum MeasurementFields
{
  MeasuredVelocity
};

using MoveStateVector = UKF::StateVector<UKF::Field<Velocity, UKF::Vector<3>>>;

using MoveMeasurementVector = UKF::FixedMeasurementVector<UKF::Field<MeasuredVelocity, UKF::Vector<3>>>;

template <>
template <>
MoveStateVector MoveStateVector::derivative<UKF::Vector<3>>(const UKF::Vector<3>& acceleration) const
{
  MoveStateVector derivative;
  derivative.set_field<Velocity>(acceleration);
  return derivative;
}

template <>
template <>
UKF::Vector<3>
MoveMeasurementVector::expected_measurement<MoveStateVector, Velocity, UKF::Vector<3>, UKF::Vector<3>, UKF::Vector<3>>(
    const MoveStateVector& state, const UKF::Vector<3>& angular_velocity, const UKF::Vector<3>& range_vector,
    const UKF::Vector<3>& camera_z_axis)
{
  auto velocity_3d = angular_velocity.cross(range_vector) + state.get_field<Velocity>();
  return velocity_3d - velocity_3d.dot(camera_z_axis) * camera_z_axis;
}

using MoveUKFCore = UKF::Core<MoveStateVector, MoveMeasurementVector, UKF::IntegratorRK4>;

#endif  // MOSE_EKF_H