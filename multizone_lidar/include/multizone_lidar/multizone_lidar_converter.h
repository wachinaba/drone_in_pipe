#ifndef MULTIZONE_LIDAR_CONVERTER_H
#define MULTIZONE_LIDAR_CONVERTER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>
#include <multizone_lidar_msgs/MultizoneRange.h>

namespace MultizoneLidarConverter
{
void convertToPointCloud2(const multizone_lidar_msgs::MultizoneRange::ConstPtr& msg,
                          sensor_msgs::PointCloud2& pointcloud);

float convertToRange(const multizone_lidar_msgs::MultizoneRange::ConstPtr& msg);
float convertToRange(const sensor_msgs::PointCloud2::ConstPtr& pointcloud);

template <typename T>
T calculateMean(const std::vector<T>& values);

template <typename T>
T calculateMeanWithoutIQROutliers(const std::vector<T>& values);
}  // namespace MultizoneLidarConverter

#endif  // MULTIZONE_LIDAR_CONVERTER_H