#ifndef FLOW_RANGE_INTEGRATOR_H
#define FLOW_RANGE_INTEGRATOR_H

#include <ros/ros.h>
#include <Eigen/Dense>

class FlowRangeIntegrator
{
public:
  FlowRangeIntegrator(const Eigen::Isometry3d& transform, double flow_covariance, double px_to_m_constant);
  FlowRangeIntegrator();

  Eigen::Vector3d calcVelocity(const Eigen::Vector2d& delta_flow, double range, ros::Time stamp);
  Eigen::Matrix3d calcCovariance(const Eigen::Vector2d& delta_flow, double range, double surface_quality);
  Eigen::Vector3d calcRangeVector(double range);
  Eigen::Vector3d getCameraZAxis();

private:
  Eigen::Isometry3d transform_;
  double flow_covariance_constant_;
  double px_to_m_constant_;
  Eigen::Matrix3d default_covariance_;
  ros::Time prev_flow_stamp_;
};

#endif  // FLOW_RANGE_INTEGRATOR_H