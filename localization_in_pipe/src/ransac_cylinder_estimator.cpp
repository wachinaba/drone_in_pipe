#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/Dense>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

class CylinderFitting
{
public:
  CylinderFitting()
  {
    // ROS subscriber to get the point cloud data
    sub = nh.subscribe("input", 1, &CylinderFitting::cloudCallback, this);
    // ROS publisher for the drone pose
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
    // ROS publisher for the drone velocity
    velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("velocity", 1);
    // ROS publisher for the Marker
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Get RANSAC parameters from rosparam
    ros::NodeHandle pnh("~");
    pnh.param<double>("normal_distance_weight", normal_distance_weight, 0.1);
    pnh.param<int>("max_iterations", max_iterations, 1000);
    pnh.param<double>("distance_threshold", distance_threshold, 0.2);
    pnh.param<double>("min_radius", min_radius, 0.25);
    pnh.param<double>("max_radius", max_radius, 1.5);
    pnh.param<double>("normal_search_radius", normal_search_radius, 0.1);
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(normal_search_radius);
    ne.compute(*cloud_normals);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(normal_distance_weight);
    seg.setMaxIterations(max_iterations);
    seg.setDistanceThreshold(distance_threshold);
    seg.setRadiusLimits(min_radius, max_radius);
    seg.setInputCloud(cloud);
    seg.setInputNormals(cloud_normals);

    // Obtain the cylinder inliers and coefficients
    pcl::ModelCoefficients::Ptr coefficients_cylinder = boost::make_shared<pcl::ModelCoefficients>();
    pcl::PointIndices::Ptr inliers_cylinder = boost::make_shared<pcl::PointIndices>();
    seg.segment(*inliers_cylinder, *coefficients_cylinder);

    // Write the cylinder inliers to disk
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    extract.filter(*cloud_cylinder);

    if (cloud_cylinder->points.empty())
    {
      ROS_ERROR_STREAM("Can't find the cylindrical component.");
    }
    else
    {
      std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

      // calculate the cylinder origin
      Eigen::Vector3f coefficients_cylinder_position(coefficients_cylinder->values[0], coefficients_cylinder->values[1],
                                                     coefficients_cylinder->values[2]);
      Eigen::Vector3f coefficients_cylinder_direction(
          coefficients_cylinder->values[3], coefficients_cylinder->values[4], coefficients_cylinder->values[5]);

      /// check if the cylinder is pointing backwards (X negative)
      if (coefficients_cylinder_direction[0] < 0)
      {
        coefficients_cylinder_direction = -coefficients_cylinder_direction;
      }

      Eigen::Vector3f cylinder_origin =
          coefficients_cylinder_position -
          coefficients_cylinder_position.dot(coefficients_cylinder_direction) * coefficients_cylinder_direction;

      // Publish the Marker
      visualization_msgs::Marker marker;
      marker.header.frame_id = cloud_msg->header.frame_id;
      marker.header.stamp = ros::Time::now();
      marker.ns = "cylinder";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD;
      marker.points.resize(2);
      marker.points[0].x = cylinder_origin[0] - coefficients_cylinder_direction[0];
      marker.points[0].y = cylinder_origin[1] - coefficients_cylinder_direction[1];
      marker.points[0].z = cylinder_origin[2] - coefficients_cylinder_direction[2];
      marker.points[1].x = cylinder_origin[0] + coefficients_cylinder_direction[0];
      marker.points[1].y = cylinder_origin[1] + coefficients_cylinder_direction[1];
      marker.points[1].z = cylinder_origin[2] + coefficients_cylinder_direction[2];
      marker.scale.x = marker.scale.y = 2 * coefficients_cylinder->values[6];  // diameter
      marker.scale.z = 0.0001;                                                 // height
      marker.color.a = 0.3;                                                    // alpha
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;

      marker_pub.publish(marker);

      // Publish the cylinder origin as a transform
      geometry_msgs::TransformStamped transformStamped;
      transformStamped.header.stamp = ros::Time::now();
      transformStamped.header.frame_id = cloud_msg->header.frame_id;
      transformStamped.child_frame_id = "pipe_frame";
      transformStamped.transform.translation.x = cylinder_origin[0];
      transformStamped.transform.translation.y = cylinder_origin[1];
      transformStamped.transform.translation.z = cylinder_origin[2];

      // calculate the cylinder orientation, cylinder X = cylinder direction
      Eigen::Vector3f cylinder_x_axis = coefficients_cylinder_direction;
      Eigen::Vector3f cylinder_y_axis = Eigen::Vector3f::UnitZ().cross(cylinder_x_axis);
      Eigen::Vector3f cylinder_z_axis = cylinder_x_axis.cross(cylinder_y_axis);
      Eigen::Matrix3f cylinder_rotation;
      cylinder_rotation << cylinder_x_axis[0], cylinder_y_axis[0], cylinder_z_axis[0], cylinder_x_axis[1],
          cylinder_y_axis[1], cylinder_z_axis[1], cylinder_x_axis[2], cylinder_y_axis[2], cylinder_z_axis[2];
      Eigen::Quaternionf cylinder_quaternion(cylinder_rotation);

      transformStamped.transform.rotation.x = cylinder_quaternion.x();
      transformStamped.transform.rotation.y = cylinder_quaternion.y();
      transformStamped.transform.rotation.z = cylinder_quaternion.z();
      transformStamped.transform.rotation.w = cylinder_quaternion.w();
      broadcaster.sendTransform(transformStamped);

      // Publish the drone pose
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = "pipe_frame";

      // calculate the drone pose
      pose.pose.position.x = -transformStamped.transform.translation.x;
      pose.pose.position.y = -transformStamped.transform.translation.y;
      pose.pose.position.z = -transformStamped.transform.translation.z;

      pose.pose.orientation.x = -transformStamped.transform.rotation.x;
      pose.pose.orientation.y = -transformStamped.transform.rotation.y;
      pose.pose.orientation.z = -transformStamped.transform.rotation.z;
      pose.pose.orientation.w = transformStamped.transform.rotation.w;

      pose_pub.publish(pose);

      // Publish the drone velocity
      geometry_msgs::TwistStamped velocity;
      velocity.header.stamp = ros::Time::now();
      velocity.header.frame_id = "pipe_frame";

      // calculate the drone velocity
      float dt = (pose.header.stamp - last_pose.header.stamp).toSec();
      velocity.twist.linear.x = (pose.pose.position.x - last_pose.pose.position.x) / dt;
      velocity.twist.linear.y = (pose.pose.position.y - last_pose.pose.position.y) / dt;
      velocity.twist.linear.z = (pose.pose.position.z - last_pose.pose.position.z) / dt;

      velocity_pub.publish(velocity);

      // update the last pose
      last_pose = pose;
    }
  }

private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher pose_pub;
  ros::Publisher velocity_pub;
  ros::Publisher marker_pub;
  tf2_ros::TransformBroadcaster broadcaster;

  // RANSAC parameters
  double normal_distance_weight;
  int max_iterations;
  double distance_threshold;
  double min_radius;
  double max_radius;
  double normal_search_radius;

  // pose
  geometry_msgs::PoseStamped last_pose;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cylinder_fitting");
  CylinderFitting cf;
  ros::spin();
  return 0;
}