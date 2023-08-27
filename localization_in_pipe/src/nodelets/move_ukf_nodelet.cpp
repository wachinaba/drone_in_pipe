#include <Eigen/Dense>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include "localization_in_pipe_msgs/IntegratedFlow.h"

#include <kalman/UnscentedKalmanFilter.hpp>
#include "localization_in_pipe/move_ukf.h"

namespace localization_in_pipe
{
class move_ukf_nodelet : public nodelet::Nodelet
{
public:
  virtual void onInit();
  void integratedFlowCallback(const localization_in_pipe_msgs::IntegratedFlow::ConstPtr& msg);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Publisher velocity_pub_;
  ros::Subscriber integrated_flow_sub_;
  ros::Subscriber imu_sub_;

  typedef float T;
  MoveUKF::State<T> filter_state_;
  MoveUKF::Control<T> filter_control_;
  MoveUKF::SystemModel<T> filter_system_model_;
  MoveUKF::MeasurementModel<T> filter_measurement_model_;

  MoveUKF::Measurement<T> filter_measurement_;
  Eigen::Matrix<T, 3, 3> filter_covariance_matrix_;
  Eigen::Matrix<T, 3, 1> filter_angular_velocity_;
  Eigen::Matrix<T, 3, 1> filter_range_vector_;
  Eigen::Matrix<T, 3, 1> filter_camera_z_axis_;

  Kalman::UnscentedKalmanFilter<MoveUKF::State<T>> filter_;

  ros::Time prev_stamp_;
  geometry_msgs::TwistWithCovarianceStamped velocity_msg;
  sensor_msgs::Imu::ConstPtr imu_msg_;

  void publishVelocity(const localization_in_pipe_msgs::IntegratedFlow::ConstPtr& msg);
};

void move_ukf_nodelet::onInit()
{
  NODELET_INFO("Initializing move_ukf_nodelet ...");

  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();

  prev_stamp_ = ros::Time(0);

  // initialize filter
  filter_state_ = MoveUKF::State<T>::Zero();
  filter_control_ = MoveUKF::Control<T>::Zero();

  filter_system_model_ = MoveUKF::SystemModel<T>();
  filter_system_model_.setCovariance(Eigen::Matrix<T, 3, 3>::Identity() * 1e-5);  // process noise covariance

  filter_measurement_model_ = MoveUKF::MeasurementModel<T>();
  filter_measurement_model_.setCovariance(Eigen::Matrix<T, 3, 3>::Identity() * 1e-3);  // measurement covariance

  filter_ = Kalman::UnscentedKalmanFilter<MoveUKF::State<T>>(0.001, 2.0, 0.0);
  filter_.init(filter_state_);
  filter_.setCovariance(Eigen::Matrix<T, 3, 3>::Identity() * 1.0);

  velocity_msg = geometry_msgs::TwistWithCovarianceStamped();

  velocity_pub_ = pnh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("velocity", 1);
  integrated_flow_sub_ = pnh_.subscribe("integrated_flow", 10, &move_ukf_nodelet::integratedFlowCallback, this);
  imu_sub_ = pnh_.subscribe("imu", 10, &move_ukf_nodelet::imuCallback, this);
}

void move_ukf_nodelet::integratedFlowCallback(const localization_in_pipe_msgs::IntegratedFlow::ConstPtr& msg)
{
  double dt = (msg->header.stamp - prev_stamp_).toSec();
  prev_stamp_ = msg->header.stamp;

  if (msg->header.stamp == ros::Time(0))  // first time
  {
    return;
  }
  else if (dt <= 0.0)  // dt should be positive
  {
    return;
  }
  else if (dt > 1.0)  // dt should be less than 1.0
  {
    return;
  }

  filter_control_.acc_x() = imu_msg_->linear_acceleration.x * dt;
  filter_control_.acc_y() = imu_msg_->linear_acceleration.y * dt;
  filter_control_.acc_z() = imu_msg_->linear_acceleration.z * dt;

  filter_state_ = filter_.predict(filter_system_model_, filter_control_);

  // set covariance
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3 - i; j++)
    {
      filter_covariance_matrix_(i, j) = filter_covariance_matrix_(j, i) = filter_covariance_matrix_(i, j);
    }
  }

  // set measurement
  filter_measurement_model_.setCovariance(filter_covariance_matrix_);
  filter_angular_velocity_ << imu_msg_->angular_velocity.x, imu_msg_->angular_velocity.y, imu_msg_->angular_velocity.z;
  filter_range_vector_ << msg->range.x, msg->range.y, msg->range.z;
  filter_camera_z_axis_ << msg->camera_z_axis.x, msg->camera_z_axis.y, msg->camera_z_axis.z;

  filter_measurement_model_.setSensorStatus(filter_angular_velocity_, filter_range_vector_, filter_camera_z_axis_);

  filter_measurement_ << msg->measured_velocity.x, msg->measured_velocity.y, msg->measured_velocity.z;

  // update
  filter_state_ = filter_.update(filter_measurement_model_, filter_measurement_);

  publishVelocity(msg);
}

void move_ukf_nodelet::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu_msg_ = msg;
}

void move_ukf_nodelet::publishVelocity(const localization_in_pipe_msgs::IntegratedFlow::ConstPtr& msg)
{
  velocity_msg.header.stamp = msg->header.stamp;
  velocity_msg.header.frame_id = msg->header.frame_id;
  velocity_msg.twist.twist.linear.x = filter_state_.vel_x();
  velocity_msg.twist.twist.linear.y = filter_state_.vel_y();
  velocity_msg.twist.twist.linear.z = filter_state_.vel_z();

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      velocity_msg.twist.covariance[6 * i + j] = filter_.getCovariance()(i, j);
    }
  }

  velocity_pub_.publish(velocity_msg);
}

}  // namespace localization_in_pipe
