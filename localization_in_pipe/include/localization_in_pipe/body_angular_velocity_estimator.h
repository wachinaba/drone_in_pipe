#ifndef BODY_ANGULAR_VELOCITY_ESTIMATOR_H
#define BODY_ANGULAR_VELOCITY_ESTIMATOR_H

#include <Eigen/Dense>

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <localization_in_pipe_msgs/CalibrateImu.h>

class AngularVelocityCalibrator
{
public:
  AngularVelocityCalibrator();

  bool calibrate(const Eigen::Matrix3Xd& angular_velocity_history);

  Eigen::Vector3d getAngularVelocityBias() const
  {
    return angular_velocity_bias_;
  }

  Eigen::Matrix3d getAngularVelocityCovariance() const
  {
    return angular_velocity_covariance_;
  }

  Eigen::Vector3d calcBodyAngularVelocity(const Eigen::Vector3d& angular_velocity) const
  {
    return angular_velocity - angular_velocity_bias_;
  }

private:
  Eigen::Matrix3d angular_velocity_covariance_;
  Eigen::Vector3d angular_velocity_bias_;
};

class BodyAngularVelocityEstimator
{
public:
  BodyAngularVelocityEstimator(int history_size_ = 100);
  bool isCalibrated() const
  {
    return is_calibrated_;
  }
  void reset(int history_size_ = 100);
  void update(const Eigen::Vector3d& angular_velocity);
  void toggleCalibration(bool enable)
  {
    enable_calibration_ = enable;
  }
  Eigen::Vector3d getBodyAngularVelocity() const
  {
    return body_angular_velocity_;
  }

private:
  bool is_calibrated_;
  bool enable_calibration_;
  int history_index_;
  int history_size_;
  Eigen::Matrix3Xd angular_velocity_history_;
  AngularVelocityCalibrator angular_velocity_calibrator_;
  Eigen::Vector3d body_angular_velocity_;
};

class BodyAngularVelocityEstimatorNode
{
public:
  BodyAngularVelocityEstimatorNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber imu_sub_;
  ros::Publisher body_angular_velocity_pub_;

  BodyAngularVelocityEstimator angular_velocity_estimator_;
  ros::ServiceServer calibrate_imu_service_;

  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  bool calibrateImuServiceCallback(localization_in_pipe_msgs::CalibrateImu::Request& req,
                                   localization_in_pipe_msgs::CalibrateImu::Response& res);
};

#endif  // BODY_ANGULAR_VELOCITY_ESTIMATOR_H