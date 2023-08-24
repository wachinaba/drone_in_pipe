#include <ros/ros.h>
#include <Eigen/Dense>

#include "localization_in_pipe/flow_range_integrator.h"

FlowRangeIntegrator::FlowRangeIntegrator(const Eigen::Isometry3d& transform, double flow_covariance)
  : transform_(transform), flow_covariance_constant_(flow_covariance), prev_flow_stamp_(ros::Time(0))
{
  default_covariance_ = Eigen::Matrix3d::Zero();
  default_covariance_(0, 0) = flow_covariance_constant_;  // optical flow x covariance
  default_covariance_(1, 1) = flow_covariance_constant_;  // optical flow y covariance

  // transform covariance to body frame
  default_covariance_ = transform_.rotation() * default_covariance_ * transform_.rotation().transpose();
}

Eigen::Vector3d FlowRangeIntegrator::calcVelocity(const Eigen::Vector2d& flow_rad, double range, ros::Time stamp)
{
  Eigen::Vector2d displacement_2d = flow_rad * range;
  Eigen::Vector3d displacement_3d(displacement_2d(0), displacement_2d(1), 0.0);

  double dt = (stamp - prev_flow_stamp_).toSec();
  prev_flow_stamp_ = stamp;

  if (stamp == ros::Time(0))  // first time
  {
    return Eigen::Vector3d::Zero();
  }
  else if (dt <= 0.0)  // dt should be positive
  {
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d velocity = transform_.rotation() * displacement_3d / dt;

  return transform_.rotation() * velocity;
}

Eigen::Matrix3d FlowRangeIntegrator::calcCovariance(const Eigen::Vector2d& flow_rad, double range,
                                                    double surface_quality)
{
  // TODO: consider surface quality, range, and flow covariance
  return default_covariance_;
}

Eigen::Vector3d FlowRangeIntegrator::calcRangeVector(double range)
{
  // transform range vector to body frame
  // consider range sensor position
  return transform_.rotation() * Eigen::Vector3d(0.0, 0.0, range) + transform_.translation();
}

Eigen::Vector3d FlowRangeIntegrator::getCameraZAxis()
{
  return transform_.rotation() * Eigen::Vector3d(0.0, 0.0, 1.0);
}
