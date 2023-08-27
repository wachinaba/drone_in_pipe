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

namespace localization_in_pipe
{
class flow_range_integrator_nodelet : public nodelet::Nodelet
{
public:
  flow_range_integrator_nodelet();

private:
  virtual void onInit();
  void rangeCallback(const sensor_msgs::Range::ConstPtr& msg);
  void flowCallback(const mavros_msgs::OpticalFlowRad::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  tf2_ros::Buffer tf_buffer_;
  boost::shared_ptr<tf2_ros::TransformListener> tf_listener_ptr_;

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

void flow_range_integrator_nodelet::onInit()
{
  NODELET_INFO("Initializing flow_range_integrator_nodelet ...");

  latest_range_ = boost::make_shared<sensor_msgs::Range>();

  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();

  getFrameIds();

  tf_listener_ptr_ = boost::make_shared<tf2_ros::TransformListener>(tf_buffer_);
  getTransform();

  // convert transform to Eigen::Transform
  Eigen::Isometry3d transform;
  tf2::fromMsg(latest_transform_.transform, transform);
  // TODO: get flow covariance from param
  flow_range_integrator_ = FlowRangeIntegrator(transform, 0.1);

  range_sub_ = nh_.subscribe("range", 10, &flow_range_integrator_nodelet::rangeCallback, this);
  flow_sub_ = nh_.subscribe("flow", 10, &flow_range_integrator_nodelet::flowCallback, this);

  integrated_flow_pub_ = nh_.advertise<localization_in_pipe_msgs::IntegratedFlow>("integrated_flow", 1);
}

void flow_range_integrator_nodelet::getFrameIds()
{
  if (!pnh_.getParam("sensor_frame", sensor_frame_))
  {
    NODELET_ERROR("Failed to get param 'sensor_frame'");
    ros::shutdown();
  }
  if (!pnh_.getParam("body_frame", body_frame_))
  {
    NODELET_WARN("Failed to get param 'body_frame'. Using 'base_link' instead");
    body_frame_ = "base_link";
  }
}

void flow_range_integrator_nodelet::getTransform()
{
  // lookup transform from sensor frame to body frame, and store it in latest_transform_, for 5 seconds
  try
  {
    latest_transform_ = tf_buffer_.lookupTransform(body_frame_, sensor_frame_, ros::Time(0),
                                                   ros::Duration(5.0));  // timeout is 5 seconds
  }
  catch (tf2::TransformException& ex)
  {
    NODELET_ERROR("%s", ex.what());
    NODELET_ERROR("Failed to get transform from %s to %s, frame id may be wrong.", sensor_frame_.c_str(),
                  body_frame_.c_str());
    ros::shutdown();
  }
}

void flow_range_integrator_nodelet::rangeCallback(const sensor_msgs::Range::ConstPtr& msg)
{
  latest_range_ = msg;
}

void flow_range_integrator_nodelet::flowCallback(const mavros_msgs::OpticalFlowRad::ConstPtr& msg)
{
  latest_flow_ = msg;
  calcIntegratedFlow();

  // set header
  integrated_flow_msg_.header.stamp = msg->header.stamp;
  integrated_flow_msg_.header.frame_id = body_frame_;

  integrated_flow_pub_.publish(integrated_flow_msg_);
}

void flow_range_integrator_nodelet::calcIntegratedFlow()
{
  Eigen::Vector2d flow_rad(latest_flow_->integrated_x, latest_flow_->integrated_y);
  double range = latest_range_->range;
  // convert quality to double
  double quality = (static_cast<double>(latest_flow_->quality)) / 255.0;

  Eigen::Vector3d velocity = flow_range_integrator_.calcVelocity(flow_rad, range, latest_flow_->header.stamp);
  Eigen::Matrix3d covariance = flow_range_integrator_.calcCovariance(flow_rad, range, quality);
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

}  // namespace localization_in_pipe

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(localization_in_pipe::flow_range_integrator_nodelet, nodelet::Nodelet)