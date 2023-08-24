#include <Eigen/Dense>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include "localization_in_pipe_msgs/IntegratedFlow.h"

#include "localization_in_pipe/move_ukf.h"

namespace localization_in_pipe
{
class move_ukf_nodelet : public nodelet::Nodelet
{
public:
  virtual void onInit();
  void integratedFlowCallback(const localization_in_pipe_msgs::IntegratedFlow::ConstPtr& msg);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Publisher velocity_pub_;
  ros::Subscriber integrated_flow_sub_;

  MoveUKFCore move_ukf_;

  ros::Time prev_stamp_;
  geometry_msgs::TwistWithCovarianceStamped velocity_msg;

  void publishVelocity(const localization_in_pipe_msgs::IntegratedFlow::ConstPtr& msg);
};

void move_ukf_nodelet::onInit()
{
  NODELET_INFO("Initializing move_ukf_nodelet ...");

  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();

  // initialize move_ukf
  move_ukf_.state.set_field<Velocity>(UKF::Vector<3>::Zero());
  move_ukf_.covariance = MoveStateVector::CovarianceMatrix::Identity();

  move_ukf_.process_noise_covariance = MoveStateVector::CovarianceMatrix::Identity() * 1e-5;
  move_ukf_.measurement_covariance = MoveMeasurementVector::CovarianceMatrix::Identity() * 1e-5;

  velocity_msg = geometry_msgs::TwistWithCovarianceStamped();

  velocity_pub_ = pnh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("velocity", 1);
  integrated_flow_sub_ = pnh_.subscribe("integrated_flow", 10, &move_ukf_nodelet::integratedFlowCallback, this);
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

  MoveMeasurementVector measurement;
  measurement.set_field<MeasuredVelocity>(
      UKF::Vector<3>(msg->measured_velocity.x, msg->measured_velocity.y, msg->measured_velocity.z));

  // set covariance
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3 - i; j++)
    {
      move_ukf_.measurement_covariance(i, j) = msg->covariance[3 * i + j];
    }
  }

  move_ukf_.step(dt, measurement);

  publishVelocity(msg);
}

void move_ukf_nodelet::publishVelocity(const localization_in_pipe_msgs::IntegratedFlow::ConstPtr& msg)
{
  velocity_msg.header.stamp = msg->header.stamp;
  velocity_msg.header.frame_id = msg->header.frame_id;
  velocity_msg.twist.twist.linear.x = move_ukf_.state.get_field<Velocity>()(0);
  velocity_msg.twist.twist.linear.y = move_ukf_.state.get_field<Velocity>()(1);
  velocity_msg.twist.twist.linear.z = move_ukf_.state.get_field<Velocity>()(2);

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      velocity_msg.twist.covariance[6 * i + j] = move_ukf_.covariance(i, j);
    }
  }

  velocity_pub_.publish(velocity_msg);
}

}  // namespace localization_in_pipe
