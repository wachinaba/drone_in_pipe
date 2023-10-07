#include <Eigen/Dense>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Range.h>
#include <mavros_msgs/OpticalFlowRad.h>
#include <localization_in_pipe_msgs/IntegratedFlow.h>

#include "localization_in_pipe/flow_range_integrator_node.h"

namespace localization_in_pipe
{
class flow_range_integrator_nodelet : public nodelet::Nodelet
{
public:
  virtual void onInit()
  {
    ros::NodeHandle nh = getNodeHandle();
    ros::NodeHandle pnh = getPrivateNodeHandle();
    try
    {
      NODELET_INFO("Initializing flow_range_integrator_nodelet ...");
      flow_range_integrator_node_ = std::shared_ptr<FlowRangeIntegratorNode>(new FlowRangeIntegratorNode(nh, pnh));
    }
    catch (const char* e)
    {
      NODELET_FATAL_STREAM("Error in FlowRangeIntegratorNode: " << e);
    }
    catch (...)
    {
      NODELET_FATAL_STREAM("Unknown error in flow_range_integrator_nodelet");
    }
  }

private:
  std::shared_ptr<FlowRangeIntegratorNode> flow_range_integrator_node_;
};

}  // namespace localization_in_pipe

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(localization_in_pipe::flow_range_integrator_nodelet, nodelet::Nodelet)