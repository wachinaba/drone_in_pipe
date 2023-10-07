#ifndef BODY_ACCEL_ESTIMATOR_H
#define BODY_ACCEL_ESTIMATOR_H

#include <Eigen/Dense>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <localization_in_pipe_msgs/CalibrateImu.h>

class AccelCalibrator
{
public:
  AccelCalibrator();

  bool calibrate(const Eigen::Matrix3Xd& accel_history, const Eigen::Matrix4Xd& orientation_history);

  Eigen::Vector3d getAccelBias() const
  {
    return accel_bias_;
  }

  Eigen::Vector3d getGravityVector() const
  {
    return gravity_vector_;
  }

  Eigen::Vector3d getAccelMean() const
  {
    return accel_mean_;
  }
  Eigen::Quaterniond getOrientationMean() const
  {
    return Eigen::Quaterniond(orientation_mean_(0), orientation_mean_(1), orientation_mean_(2), orientation_mean_(3));
  }
  Eigen::Matrix3d getAccelCovariance() const
  {
    return accel_covariance_;
  }
  Eigen::Matrix4d getOrientationCovariance() const
  {
    return orientation_covariance_;
  }

  Eigen::Vector3d calcBodyAccel(const Eigen::Vector3d& accel, const Eigen::Quaterniond& orientation) const
  {
    return -(accel - accel_bias_ - orientation.inverse() * gravity_vector_);
  }

private:
  Eigen::Vector3d accel_mean_;
  Eigen::Matrix3d accel_covariance_;
  Eigen::Vector4d orientation_mean_;
  Eigen::Matrix4d orientation_covariance_;

  Eigen::Vector3d accel_bias_;
  Eigen::Vector3d gravity_vector_;
};

class BodyAccelEstimator
{
public:
  explicit BodyAccelEstimator(int history_size = 100);
  bool isCalibrated() const
  {
    return is_calibrated_;
  }
  void reset(int history_size = 100);
  void update(const Eigen::Vector3d& accel, const Eigen::Quaterniond& orientation);
  void toggleCalibration(bool enable)
  {
    enable_calibration_ = enable;
  }
  Eigen::Vector3d getBodyAccel() const
  {
    return body_accel_;
  }
  Eigen::Vector3d getGravityVector() const
  {
    return accel_calibrator_.getGravityVector();
  }

private:
  bool is_calibrated_;
  bool enable_calibration_;
  int history_index_;
  int history_size_;
  Eigen::Matrix3Xd accel_history_;
  Eigen::Matrix4Xd orientation_history_;
  AccelCalibrator accel_calibrator_;
  Eigen::Vector3d body_accel_;
};

class BodyAccelEstimatorNode
{
public:
  BodyAccelEstimatorNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber imu_sub_;
  ros::Publisher body_accel_imu_pub_;
  ros::Publisher raw_imu_accel_pub_;
  ros::Publisher gravity_pub_;
  ros::Publisher body_accel_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string sensor_frame_;
  std::string body_frame_;
  Eigen::Isometry3d transform_;

  BodyAccelEstimator body_accel_estimator_;
  ros::ServiceServer calibrate_imu_service_;

  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  bool calibrateImuServiceCallback(localization_in_pipe_msgs::CalibrateImu::Request& req,
                                   localization_in_pipe_msgs::CalibrateImu::Response& res);
  void getFrameIds();
  void getTransform();
};

#endif  // BODY_ACCEL_ESTIMATOR_H