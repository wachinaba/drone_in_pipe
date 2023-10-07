#include <Eigen/Dense>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
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
  void bodyAccelCallback(const geometry_msgs::AccelStamped::ConstPtr& msg);
  void bodyAngularVelocityCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Publisher velocity_pub_;
  ros::Subscriber integrated_flow_sub_;
  ros::Subscriber body_accel_sub_;
  ros::Subscriber body_angular_velocity_sub_;

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
  geometry_msgs::AccelStamped::ConstPtr body_accel_msg_;
  geometry_msgs::Vector3Stamped::ConstPtr body_angular_velocity_msg_;

  void publishVelocity(const localization_in_pipe_msgs::IntegratedFlow::ConstPtr& msg);
};

void move_ukf_nodelet::onInit()
{
  NODELET_INFO("Initializing move_ukf_nodelet ...");

  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();

  prev_stamp_ = ros::Time(0);

  body_accel_msg_ = boost::make_shared<geometry_msgs::AccelStamped>();

  // initialize filter
  filter_state_ = MoveUKF::State<T>::Zero();
  filter_control_ = MoveUKF::Control<T>::Zero();

  filter_system_model_ = MoveUKF::SystemModel<T>();
  filter_system_model_.setCovariance(Eigen::Matrix<T, 3, 3>::Identity() * 1e-5);  // process noise covariance

  filter_measurement_model_ = MoveUKF::MeasurementModel<T>();
  filter_measurement_model_.setCovariance(Eigen::Matrix<T, 3, 3>::Identity() * 1e-3);  // measurement covariance

  filter_ = Kalman::UnscentedKalmanFilter<MoveUKF::State<T>>(1, 2.0, 0.0);
  filter_.init(filter_state_);
  filter_.setCovariance(Eigen::Matrix<T, 3, 3>::Identity() * 1.0);

  velocity_msg = geometry_msgs::TwistWithCovarianceStamped();
  body_accel_msg_ = boost::make_shared<geometry_msgs::AccelStamped>();
  body_angular_velocity_msg_ = boost::make_shared<geometry_msgs::Vector3Stamped>();

  velocity_pub_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("velocity", 1);
  integrated_flow_sub_ = nh_.subscribe("integrated_flow", 10, &move_ukf_nodelet::integratedFlowCallback, this);
  body_accel_sub_ = nh_.subscribe("body_accel", 10, &move_ukf_nodelet::bodyAccelCallback, this);
  body_angular_velocity_sub_ =
      nh_.subscribe("body_angular_velocity", 10, &move_ukf_nodelet::bodyAngularVelocityCallback, this);
}

void move_ukf_nodelet::integratedFlowCallback(const localization_in_pipe_msgs::IntegratedFlow::ConstPtr& msg)
{
  double dt = (msg->header.stamp - prev_stamp_).toSec();
  if (prev_stamp_ == ros::Time(0))  // first time
  {
    prev_stamp_ = msg->header.stamp;
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

  // set control
  filter_control_.acc_x() = body_accel_msg_->accel.linear.x * dt;
  filter_control_.acc_y() = body_accel_msg_->accel.linear.y * dt;
  filter_control_.acc_z() = body_accel_msg_->accel.linear.z * dt;

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

  filter_angular_velocity_ << body_angular_velocity_msg_->vector.x, body_angular_velocity_msg_->vector.y,
      body_angular_velocity_msg_->vector.z;
  filter_range_vector_ << msg->range.x, msg->range.y, msg->range.z;
  filter_camera_z_axis_ << msg->camera_z_axis.x, msg->camera_z_axis.y, msg->camera_z_axis.z;

  filter_measurement_model_.setSensorStatus(filter_angular_velocity_, filter_range_vector_, filter_camera_z_axis_);

  filter_measurement_ << msg->measured_velocity.x, msg->measured_velocity.y, msg->measured_velocity.z;

  // update
  filter_state_ = filter_.update(filter_measurement_model_, filter_measurement_);

  publishVelocity(msg);
}

void move_ukf_nodelet::bodyAccelCallback(const geometry_msgs::AccelStamped::ConstPtr& msg)
{
  body_accel_msg_ = msg;
}

void move_ukf_nodelet::bodyAngularVelocityCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  body_angular_velocity_msg_ = msg;
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
PLUGINLIB_EXPORT_CLASS(localization_in_pipe::move_ukf_nodelet, nodelet::Nodelet)
