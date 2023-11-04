#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

namespace localization_in_pipe
{
class pointcloud_transform_nodelet : public nodelet::Nodelet
{
public:
  pointcloud_transform_nodelet()
  {
  }
  virtual ~pointcloud_transform_nodelet() override
  {
  }

private:
  virtual void onInit()
  {
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& pnh = getPrivateNodeHandle();

    // Get the target frame from rosparam
    if (!pnh.getParam("target_frame", target_frame_))
    {
      ROS_ERROR("Failed to get target_frame parameter");
      return;
    }

    // Subscribe to the input point cloud topic
    sub_ = nh.subscribe("input", 1, &pointcloud_transform_nodelet::pointCloudCallback, this);

    // Advertise the output point cloud topic
    pub_ = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

    // Initialize the transform listener
    tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));

    ROS_INFO("pointcloud_transform_nodelet initialized");
  }

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud)
  {
    // Get the transform from the input cloud frame to the target frame
    geometry_msgs::TransformStamped transform;
    try
    {
      transform =
          tf_buffer_.lookupTransform(target_frame_, input_cloud->header.frame_id, ros::Time(0), ros::Duration(1.0));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      return;
    }

    // Transform the input cloud to the target frame
    sensor_msgs::PointCloud2 output_cloud;
    tf2::doTransform(*input_cloud, output_cloud, transform);
    output_cloud.header.frame_id = target_frame_;
    output_cloud.header.stamp = input_cloud->header.stamp;
    output_cloud.fields = input_cloud->fields;
    output_cloud.is_bigendian = input_cloud->is_bigendian;
    output_cloud.point_step = input_cloud->point_step;
    output_cloud.row_step = input_cloud->row_step;
    output_cloud.is_dense = input_cloud->is_dense;
    output_cloud.height = input_cloud->height;
    output_cloud.width = input_cloud->width;

    // Publish the transformed cloud
    pub_.publish(output_cloud);
  }

  ros::Subscriber sub_;
  ros::Publisher pub_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  tf2_ros::Buffer tf_buffer_;
  std::string target_frame_;
};
}  // namespace localization_in_pipe

PLUGINLIB_EXPORT_CLASS(localization_in_pipe::pointcloud_transform_nodelet, nodelet::Nodelet);
