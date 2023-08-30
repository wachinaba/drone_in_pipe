#include <Eigen/Dense>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "localization_in_pipe/body_accel_estimator.h"

namespace localization_in_pipe
{
class body_accel_estimator_nodelet : public nodelet::Nodelet
{
public:
  virtual void onInit()
  {
    ros::NodeHandle nh = getNodeHandle();
    ros::NodeHandle pnh = getPrivateNodeHandle();

    try
    {
      NODELET_INFO("Initializing body_accel_estimator_nodelet ...");
      body_accel_estimator_ = std::shared_ptr<BodyAccelEstimatorNode>(new BodyAccelEstimatorNode(nh, pnh));
    }
    catch (const char* e)
    {
      NODELET_FATAL_STREAM("Error in BodyAccelEstimatorNode: " << e);
    }
    catch (...)
    {
      NODELET_FATAL_STREAM("Unknown error in body_accel_estimator_nodelet");
    }
  }

protected:
  std::shared_ptr<BodyAccelEstimatorNode> body_accel_estimator_;
};
}  // namespace localization_in_pipe

PLUGINLIB_EXPORT_CLASS(localization_in_pipe::body_accel_estimator_nodelet, nodelet::Nodelet)
