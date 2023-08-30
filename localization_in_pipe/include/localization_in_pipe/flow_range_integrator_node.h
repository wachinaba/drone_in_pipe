#ifndef FLOW_RANGE_INTEGRATOR_NODE_H
#define FLOW_RANGE_INTEGRATOR_NODE_H

#include <Eigen/Dense>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Range.h>
#include <mavros_msgs/OpticalFlowRad.h>
#include <localization_in_pipe_msgs/IntegratedFlow.h>

#include "localization_in_pipe/flow_range_integrator.h"

class FlowRangeIntegratorNode
{
public:
  explicit FlowRangeIntegratorNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  void rangeCallback(const sensor_msgs::Range::ConstPtr& msg);
  void flowCallback(const mavros_msgs::OpticalFlowRad::ConstPtr& msg);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Publisher integrated_flow_pub_;
  ros::Subscriber range_sub_;
  ros::Subscriber flow_sub_;

  FlowRangeIntegrator flow_range_integrator_;

  geometry_msgs::TransformStamped latest_transform_;
  sensor_msgs::Range::ConstPtr latest_range_;
  mavros_msgs::OpticalFlowRad::ConstPtr latest_flow_;

  localization_in_pipe_msgs::IntegratedFlow integrated_flow_msg_;

  std::string sensor_frame_;
  std::string body_frame_;

  void getFrameIds();
  void getTransform();

  void calcIntegratedFlow();
};

#endif  // FLOW_RANGE_INTEGRATOR_NODE_H