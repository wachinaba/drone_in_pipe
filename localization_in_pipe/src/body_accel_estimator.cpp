#include <Eigen/Dense>

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <localization_in_pipe_msgs/CalibrateImu.h>

#include "localization_in_pipe/body_accel_estimator.h"

AccelCalibrator::AccelCalibrator()
  : accel_mean_(Eigen::Vector3d::Zero())
  , accel_covariance_(Eigen::Matrix3d::Zero())
  , orientation_mean_(Eigen::Vector4d::Zero())
  , orientation_covariance_(Eigen::Matrix4d::Zero())
  , accel_bias_(Eigen::Vector3d::Zero())
  , gravity_vector_(Eigen::Vector3d::Zero())
{
}

bool AccelCalibrator::calibrate(const Eigen::Matrix3Xd& accel_history, const Eigen::Matrix4Xd& orientation_history)
{
  if (accel_history.cols() <= 1)
  {
    return true;
  }

  // calc mean and variance
  accel_mean_ = accel_history.rowwise().mean();
  Eigen::Matrix3Xd accel_centered = accel_history.colwise() - accel_mean_;
  accel_covariance_ = accel_centered * accel_centered.transpose() / double(accel_history.cols() - 1);

  orientation_mean_ = orientation_history.rowwise().mean();
  Eigen::Matrix4Xd orientation_centered = orientation_history.colwise() - orientation_mean_;
  orientation_covariance_ =
      orientation_centered * orientation_centered.transpose() / double(orientation_history.cols() - 1);

  // check variance of accel and orientation is small enough
  if (accel_covariance_.norm() > 0.1 || orientation_covariance_.norm() > 0.1)
  {
    ROS_WARN(
        "Calibration failed because of large variance, accel_covariance.norm() = %f, orientation_covariance.norm() = "
        "%f",
        accel_covariance_.norm(), orientation_covariance_.norm());
    ROS_WARN("Accel mean: [%f, %f, %f]", accel_mean_(0), accel_mean_(1), accel_mean_(2));
    return false;  // calibration failed because of large variance due to motion, etc.
  }

  gravity_vector_ = -Eigen::Vector3d::UnitZ() * accel_mean_.norm();

  // calc and set accel bias
  Eigen::Quaterniond gravity_quaternion =
      Eigen::Quaterniond(orientation_mean_(0), orientation_mean_(1), orientation_mean_(2), orientation_mean_(3))
          .inverse();
  accel_bias_ = accel_mean_ - gravity_quaternion * gravity_vector_;

  return true;
}

BodyAccelEstimator::BodyAccelEstimator(int history_size_)
  : is_calibrated_(false)
  , enable_calibration_(false)
  , history_index_(0)
  , history_size_(history_size_)
  , accel_history_(Eigen::Matrix3Xd::Zero(3, history_size_))
  , orientation_history_(Eigen::Matrix4Xd::Zero(4, history_size_))
  , accel_calibrator_()
  , body_accel_(Eigen::Vector3d::Zero())
{
}

void BodyAccelEstimator::reset(int history_size)
{
  is_calibrated_ = false;
  enable_calibration_ = false;
  history_index_ = 0;
  history_size_ = history_size;
  accel_history_ = Eigen::Matrix3Xd::Zero(3, history_size_);
  orientation_history_ = Eigen::Matrix4Xd::Zero(4, history_size_);
  accel_calibrator_ = AccelCalibrator();
  body_accel_ = Eigen::Vector3d::Zero();
}

void BodyAccelEstimator::update(const Eigen::Vector3d& accel, const Eigen::Quaterniond& orientation)
{
  if (enable_calibration_)
  {
    accel_history_.col(history_index_) = accel;
    orientation_history_.col(history_index_) << orientation.w(), orientation.x(), orientation.y(), orientation.z();
    history_index_ = (history_index_ + 1) % history_size_;
    if (accel_calibrator_.calibrate(accel_history_.leftCols(history_index_),
                                    orientation_history_.leftCols(history_index_)))
    {
      ROS_INFO_THROTTLE(0.5, "Calibrating IMU, %d %% collected", int(100.0 * history_index_ / history_size_));

      if (history_index_ == 0)
      {
        is_calibrated_ = true;
        enable_calibration_ = false;
        ROS_INFO("IMU Calibration succeeded");
        ROS_INFO("accel bias: [%f, %f, %f]", accel_calibrator_.getAccelBias()(0), accel_calibrator_.getAccelBias()(1),
                 accel_calibrator_.getAccelBias()(2));
        ROS_INFO("gravity vector: [%f, %f, %f]", accel_calibrator_.getGravityVector()(0),
                 accel_calibrator_.getGravityVector()(1), accel_calibrator_.getGravityVector()(2));
      }

    }  // else calibration failed
    else
    {
      is_calibrated_ = false;
      reset(history_size_);
      enable_calibration_ = true;
      ROS_INFO("IMU Calibration failed, accel covariance norm = %f, orientation covariance norm = %f",
               accel_calibrator_.getAccelCovariance().norm(), accel_calibrator_.getOrientationCovariance().norm());
    }
  }
  if (is_calibrated_)
  {
    body_accel_ = accel_calibrator_.calcBodyAccel(accel, orientation);
  }
  else
  {
    body_accel_ = Eigen::Vector3d::Zero();
  }
}

BodyAccelEstimatorNode::BodyAccelEstimatorNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh), body_accel_estimator_(), tf_listener_(tf_buffer_)
{
  ROS_INFO("Initializing BodyAccelEstimatorNode ...");
  getFrameIds();
  getTransform();

  imu_sub_ = nh_.subscribe("imu", 1, &BodyAccelEstimatorNode::imuCallback, this);
  body_accel_imu_pub_ = nh_.advertise<sensor_msgs::Imu>("body_accel_imu", 1);

  raw_imu_accel_pub_ = nh_.advertise<geometry_msgs::AccelStamped>("raw_imu_accel", 1);
  gravity_pub_ = nh_.advertise<geometry_msgs::AccelStamped>("gravity", 1);
  body_accel_pub_ = nh_.advertise<geometry_msgs::AccelStamped>("body_accel", 1);

  calibrate_imu_service_ =
      nh_.advertiseService("calibrate_imu", &BodyAccelEstimatorNode::calibrateImuServiceCallback, this);
}

void BodyAccelEstimatorNode::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  Eigen::Vector3d accel_raw(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  Eigen::Quaterniond orientation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

  // transform accel from sensor frame to body frame
  Eigen::Vector3d accel = transform_.linear() * accel_raw;

  body_accel_estimator_.update(accel, orientation);

  sensor_msgs::Imu body_accel_msg;
  body_accel_msg.header = msg->header;
  body_accel_msg.header.frame_id = body_frame_;
  body_accel_msg.linear_acceleration.x = body_accel_estimator_.getBodyAccel()(0);
  body_accel_msg.linear_acceleration.y = body_accel_estimator_.getBodyAccel()(1);
  body_accel_msg.linear_acceleration.z = body_accel_estimator_.getBodyAccel()(2);
  body_accel_msg.orientation = msg->orientation;
  body_accel_msg.angular_velocity = msg->angular_velocity;
  body_accel_imu_pub_.publish(body_accel_msg);

  // publish accels
  geometry_msgs::AccelStamped raw_imu_accel_msg;
  raw_imu_accel_msg.header = msg->header;
  raw_imu_accel_msg.header.frame_id = sensor_frame_;

  raw_imu_accel_msg.accel.linear.x = accel_raw(0);
  raw_imu_accel_msg.accel.linear.y = accel_raw(1);
  raw_imu_accel_msg.accel.linear.z = accel_raw(2);
  raw_imu_accel_pub_.publish(raw_imu_accel_msg);

  geometry_msgs::AccelStamped gravity_msg;
  gravity_msg.header = msg->header;
  gravity_msg.header.frame_id = body_frame_;

  Eigen::Vector3d gravity_vector = orientation.inverse() * body_accel_estimator_.getGravityVector();

  gravity_msg.accel.linear.x = gravity_vector(0);
  gravity_msg.accel.linear.y = gravity_vector(1);
  gravity_msg.accel.linear.z = gravity_vector(2);

  gravity_pub_.publish(gravity_msg);

  geometry_msgs::AccelStamped body_accel_msg_;
  body_accel_msg_.header = msg->header;
  body_accel_msg_.header.frame_id = body_frame_;

  body_accel_msg_.accel.linear.x = body_accel_estimator_.getBodyAccel()(0);
  body_accel_msg_.accel.linear.y = body_accel_estimator_.getBodyAccel()(1);
  body_accel_msg_.accel.linear.z = body_accel_estimator_.getBodyAccel()(2);

  body_accel_pub_.publish(body_accel_msg_);
}

bool BodyAccelEstimatorNode::calibrateImuServiceCallback(localization_in_pipe_msgs::CalibrateImu::Request& req,
                                                         localization_in_pipe_msgs::CalibrateImu::Response& res)
{
  body_accel_estimator_.toggleCalibration(req.enable_calibration);
  ROS_INFO("IMU Calibration %s", req.enable_calibration ? "enabled" : "disabled");
  return true;
}

void BodyAccelEstimatorNode::getFrameIds()
{
  if (!pnh_.getParam("sensor_frame", sensor_frame_))
  {
    ROS_ERROR("Failed to get param 'sensor_frame'");
    ros::shutdown();
  }
  if (!pnh_.getParam("body_frame", body_frame_))
  {
    ROS_WARN("Failed to get param 'body_frame'. Using 'base_link' instead");
    body_frame_ = "base_link";
  }
  ROS_INFO("[BodyAccelEstimatorNode] sensor_frame: %s, body_frame: %s", sensor_frame_.c_str(), body_frame_.c_str());
}

void BodyAccelEstimatorNode::getTransform()
{
  // lookup transform from sensor frame to body frame, and store it in transform_, for 5 seconds
  geometry_msgs::TransformStamped tf2_transform;
  try
  {
    tf2_transform = tf_buffer_.lookupTransform(body_frame_, sensor_frame_, ros::Time(0), ros::Duration(5.0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  transform_.translation() =
      Eigen::Vector3d(tf2_transform.transform.translation.x, tf2_transform.transform.translation.y,
                      tf2_transform.transform.translation.z);
  transform_.linear() = Eigen::Quaterniond(tf2_transform.transform.rotation.w, tf2_transform.transform.rotation.x,
                                           tf2_transform.transform.rotation.y, tf2_transform.transform.rotation.z)
                            .toRotationMatrix();
}