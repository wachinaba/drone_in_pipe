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

class CylinderFitting
{
public:
  CylinderFitting()
  {
    // ROS subscriber to get the point cloud data
    sub = nh.subscribe("input", 1, &CylinderFitting::cloudCallback, this);
    // ROS publisher for the Marker
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Get RANSAC parameters from rosparam
    ros::NodeHandle pnh("~");
    pnh.param<double>("normal_distance_weight", normal_distance_weight, 0.1);
    pnh.param<int>("max_iterations", max_iterations, 1000);
    pnh.param<double>("distance_threshold", distance_threshold, 0.2);
    pnh.param<double>("min_radius", min_radius, 0.5);
    pnh.param<double>("max_radius", max_radius, 3.0);
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
    }
  }

private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher marker_pub;

  // RANSAC parameters
  double normal_distance_weight;
  int max_iterations;
  double distance_threshold;
  double min_radius;
  double max_radius;
  double normal_search_radius;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cylinder_fitting");
  CylinderFitting cf;
  ros::spin();
  return 0;
}