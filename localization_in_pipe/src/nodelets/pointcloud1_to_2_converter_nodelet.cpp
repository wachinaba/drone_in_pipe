#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

namespace localization_in_pipe
{
class pointcloud1_to_2_converter_nodelet : public nodelet::Nodelet
{
public:
  pointcloud1_to_2_converter_nodelet()
  {
  }
  ~pointcloud1_to_2_converter_nodelet()
  {
  }

private:
  virtual void onInit()
  {
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& pnh = getPrivateNodeHandle();

    // Subscribe to input point cloud topic
    sub_ = nh.subscribe("input_point_cloud", 1, &pointcloud1_to_2_converter_nodelet::pointCloud1Callback, this);

    // Advertise output point cloud topic
    pub_ = nh.advertise<sensor_msgs::PointCloud2>("output_point_cloud", 1);
  }

  void pointCloud1Callback(const sensor_msgs::PointCloudConstPtr& cloud1_msg)
  {
    // Convert PointCloud to PointCloud2
    sensor_msgs::PointCloud2 cloud2_msg;
    sensor_msgs::convertPointCloudToPointCloud2(*cloud1_msg, cloud2_msg);
    cloud2_msg.header = cloud1_msg->header;
    if (cloud1_msg->header.stamp == ros::Time(0))
    {
      cloud2_msg.header.stamp = ros::Time::now();
    }

    // Publish PointCloud2
    pub_.publish(cloud2_msg);
  }

  ros::Subscriber sub_;
  ros::Publisher pub_;
};

}  // namespace localization_in_pipe

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(localization_in_pipe::pointcloud1_to_2_converter_nodelet, nodelet::Nodelet)
