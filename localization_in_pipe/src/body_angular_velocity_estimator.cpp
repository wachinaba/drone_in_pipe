#include <Eigen/Dense>

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <localization_in_pipe_msgs/CalibrateImu.h>

#include "localization_in_pipe/body_angular_velocity_estimator.h"

AngularVelocityCalibrator::AngularVelocityCalibrator()
  : angular_velocity_covariance_(Eigen::Matrix3d::Zero()), angular_velocity_bias_(Eigen::Vector3d::Zero())
{
}

bool AngularVelocityCalibrator::calibrate(const Eigen::Matrix3Xd& angular_velocity_history)
{
  if (angular_velocity_history.cols() <= 1)
  {
    return true;
  }

  // calc mean and variance
  Eigen::Vector3d angular_velocity_mean = angular_velocity_history.rowwise().mean();
  Eigen::Matrix3Xd angular_velocity_centered = angular_velocity_history.colwise() - angular_velocity_mean;
  angular_velocity_covariance_ =
      angular_velocity_centered * angular_velocity_centered.transpose() / double(angular_velocity_history.cols() - 1);

  // check variance of angular velocity is small enough
  if (angular_velocity_covariance_.norm() > 0.1)
  {
    ROS_WARN("Calibration failed because of large variance, angular_velocity_covariance.norm() = %f",
             angular_velocity_covariance_.norm());
    return false;  // calibration failed because of large variance due to motion, etc.
  }

  // calc and set angular velocity bias
  angular_velocity_bias_ = angular_velocity_mean;

  return true;
}

BodyAngularVelocityEstimator::BodyAngularVelocityEstimator(int history_size)
  : is_calibrated_(false)
  , enable_calibration_(false)
  , history_size_(history_size)
  , history_index_(0)
  , angular_velocity_history_(Eigen::Matrix3Xd::Zero(3, history_size))
{
}

void BodyAngularVelocityEstimator::reset(int history_size)
{
  is_calibrated_ = false;
  enable_calibration_ = false;
  history_index_ = 0;
  history_size_ = history_size;
  angular_velocity_history_ = Eigen::Matrix3Xd::Zero(3, history_size);
  angular_velocity_calibrator_ = AngularVelocityCalibrator();
  body_angular_velocity_ = Eigen::Vector3d::Zero();
}

void BodyAngularVelocityEstimator::update(const Eigen::Vector3d& angular_velocity)
{
  if (enable_calibration_)
  {
    angular_velocity_history_.col(history_index_) = angular_velocity;
    history_index_ = (history_index_ + 1) % history_size_;
    if (angular_velocity_calibrator_.calibrate(angular_velocity_history_.leftCols(history_index_)))
    {
      ROS_INFO_THROTTLE(0.5, "Calibrating Gyro, %d %% collected", int(100.0 * history_index_ / history_size_));

      if (history_index_ == 0)
      {
        is_calibrated_ = true;
        enable_calibration_ = false;
        ROS_INFO("Gyro Calibration succeeded");
        ROS_INFO("angular velocity bias: [%f, %f, %f]", angular_velocity_calibrator_.getAngularVelocityBias()(0),
                 angular_velocity_calibrator_.getAngularVelocityBias()(1),
                 angular_velocity_calibrator_.getAngularVelocityBias()(2));
      }
    }
    else
    {
      is_calibrated_ = false;
      reset(history_size_);
      enable_calibration_ = true;
      ROS_INFO("Gyro Calibration failed, angular velocity covariance norm: %f",
               angular_velocity_calibrator_.getAngularVelocityCovariance().norm());
    }
  }
  if (is_calibrated_)
  {
    body_angular_velocity_ = angular_velocity_calibrator_.calcBodyAngularVelocity(angular_velocity);
  }
  else
  {
    body_angular_velocity_ = Eigen::Vector3d::Zero();
  }
}

BodyAngularVelocityEstimatorNode::BodyAngularVelocityEstimatorNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh), angular_velocity_estimator_(100)
{
  body_angular_velocity_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("body_angular_velocity", 1);
  imu_sub_ = nh_.subscribe("imu", 1, &BodyAngularVelocityEstimatorNode::imuCallback, this);

  calibrate_imu_service_ =
      nh_.advertiseService("calibrate", &BodyAngularVelocityEstimatorNode::calibrateImuServiceCallback, this);
}

void BodyAngularVelocityEstimatorNode::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  angular_velocity_estimator_.update(
      Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z));
  geometry_msgs::Vector3Stamped body_angular_velocity_msg;
  body_angular_velocity_msg.header = msg->header;
  body_angular_velocity_msg.vector.x = angular_velocity_estimator_.getBodyAngularVelocity()(0);
  body_angular_velocity_msg.vector.y = angular_velocity_estimator_.getBodyAngularVelocity()(1);
  body_angular_velocity_msg.vector.z = angular_velocity_estimator_.getBodyAngularVelocity()(2);
  body_angular_velocity_pub_.publish(body_angular_velocity_msg);
}

bool BodyAngularVelocityEstimatorNode::calibrateImuServiceCallback(
    localization_in_pipe_msgs::CalibrateImu::Request& req, localization_in_pipe_msgs::CalibrateImu::Response& res)
{
  angular_velocity_estimator_.toggleCalibration(req.enable_calibration);
  ROS_INFO("Gyro Calibration %s", req.enable_calibration ? "enabled" : "disabled");
  return true;
}