#include <Eigen/Dense>

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Range.h>
#include <optical_flow_msgs/OpticalFlowDelta.h>
#include <localization_in_pipe_msgs/IntegratedFlow.h>

#include "localization_in_pipe/flow_range_integrator.h"
#include "localization_in_pipe/flow_range_integrator_node.h"

FlowRangeIntegratorNode::FlowRangeIntegratorNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh)
  , pnh_(pnh)
  , tf_listener_(tf_buffer_)
  , flow_range_integrator_()
  , latest_range_(boost::make_shared<sensor_msgs::Range>())
{
  ROS_INFO("Initializing FlowRangeIntegratorNode ...");
  getFrameIds();
  getTransform();

  Eigen::Isometry3d transform;
  transform.translation() =
      Eigen::Vector3d(latest_transform_.transform.translation.x, latest_transform_.transform.translation.y,
                      latest_transform_.transform.translation.z);
  transform.linear() =
      Eigen::Quaterniond(latest_transform_.transform.rotation.w, latest_transform_.transform.rotation.x,
                         latest_transform_.transform.rotation.y, latest_transform_.transform.rotation.z)
          .toRotationMatrix();

  double flow_covariance = 0.1;
  if (!pnh_.getParam("flow_covariance", flow_covariance))
  {
    ROS_WARN("Failed to get param 'flow_covariance'. Using default value: 0.1");
  }

  double px_to_m_constant = 0.01;
  if (!pnh_.getParam("px_to_m_constant", px_to_m_constant))
  {
    ROS_WARN("Failed to get param 'px_to_m_constant'. Using default value: 0.01");
  }

  flow_range_integrator_ = FlowRangeIntegrator(transform, flow_covariance, px_to_m_constant);

  integrated_flow_pub_ = nh_.advertise<localization_in_pipe_msgs::IntegratedFlow>("integrated_flow", 1);
  integrated_flow_pub_passthrough_ =
      nh_.advertise<localization_in_pipe_msgs::IntegratedFlow>("integrated_flow_passthrough", 1);

  range_sub_ = nh_.subscribe("range", 1, &FlowRangeIntegratorNode::rangeCallback, this);
  flow_sub_ = nh_.subscribe("flow", 1, &FlowRangeIntegratorNode::flowCallback, this);
}

void FlowRangeIntegratorNode::getFrameIds()
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
  ROS_INFO("[FlowRangeIntegratorNode] sensor_frame: %s, body_frame: %s", sensor_frame_.c_str(), body_frame_.c_str());
}

void FlowRangeIntegratorNode::getTransform()
{
  // lookup transform from sensor frame to body frame, and store it in latest_transform_, for 5 seconds
  try
  {
    latest_transform_ = tf_buffer_.lookupTransform(body_frame_, sensor_frame_, ros::Time(0),
                                                   ros::Duration(5.0));  // timeout is 5 seconds
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    ROS_ERROR("Failed to get transform from %s to %s, frame id may be wrong.", sensor_frame_.c_str(),
              body_frame_.c_str());
    ros::shutdown();
  }
  catch (...)
  {
    ROS_ERROR("Unknown error in getTransform");
    ros::shutdown();
  }
}

void FlowRangeIntegratorNode::rangeCallback(const sensor_msgs::Range::ConstPtr& msg)
{
  latest_range_ = msg;
}

void FlowRangeIntegratorNode::flowCallback(const optical_flow_msgs::OpticalFlowDelta::ConstPtr& msg)
{
  latest_flow_ = msg;
  calcIntegratedFlow();

  // set header
  integrated_flow_msg_.header.stamp = msg->header.stamp;
  integrated_flow_msg_.header.frame_id = body_frame_;

  integrated_flow_pub_.publish(integrated_flow_msg_);
  integrated_flow_pub_passthrough_.publish(integrated_flow_msg_);
}

void FlowRangeIntegratorNode::calcIntegratedFlow()
{
  Eigen::Vector2d flow_delta(latest_flow_->delta_px, latest_flow_->delta_py);
  double range = latest_range_->range;
  // convert quality to double
  double quality = (static_cast<double>(latest_flow_->surface_quality)) / 255.0;

  Eigen::Vector3d velocity = flow_range_integrator_.calcVelocity(flow_delta, range, latest_flow_->header.stamp);
  Eigen::Matrix3d covariance = flow_range_integrator_.calcCovariance(flow_delta, range, quality);
  Eigen::Vector3d range_vector = flow_range_integrator_.calcRangeVector(range);
  Eigen::Vector3d camera_z_axis = flow_range_integrator_.getCameraZAxis();

  // set values to integrated_flow_msg_
  integrated_flow_msg_.measured_velocity.x = velocity.x();
  integrated_flow_msg_.measured_velocity.y = velocity.y();
  integrated_flow_msg_.measured_velocity.z = velocity.z();

  // set covariance, row major
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      integrated_flow_msg_.covariance[i * 3 + j] = covariance(i, j);
    }
  }

  integrated_flow_msg_.range.x = range_vector.x();
  integrated_flow_msg_.range.y = range_vector.y();
  integrated_flow_msg_.range.z = range_vector.z();

  integrated_flow_msg_.camera_z_axis.x = camera_z_axis.x();
  integrated_flow_msg_.camera_z_axis.y = camera_z_axis.y();
  integrated_flow_msg_.camera_z_axis.z = camera_z_axis.z();
}
