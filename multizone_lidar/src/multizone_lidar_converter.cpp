#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>
#include <multizone_lidar_msgs/MultizoneRange.h>

#include "multizone_lidar/multizone_lidar_converter.h"

namespace MultizoneLidarConverter
{
void convertToPointCloud2(const multizone_lidar_msgs::MultizoneRange::ConstPtr& msg,
                          sensor_msgs::PointCloud2& pointcloud)
{
  pointcloud.height = 1;
  pointcloud.width = msg->horizontal_samples * msg->vertical_samples;
  pointcloud.is_bigendian = false;
  pointcloud.is_dense = false;
  pointcloud.point_step = 12;
  pointcloud.row_step = pointcloud.point_step * pointcloud.width;
  pointcloud.fields.resize(3);
  pointcloud.fields[0].name = "x";
  pointcloud.fields[0].offset = 0;
  pointcloud.fields[0].datatype = 7;
  pointcloud.fields[0].count = 1;
  pointcloud.fields[1].name = "y";
  pointcloud.fields[1].offset = 4;
  pointcloud.fields[1].datatype = 7;
  pointcloud.fields[1].count = 1;
  pointcloud.fields[2].name = "z";
  pointcloud.fields[2].offset = 8;
  pointcloud.fields[2].datatype = 7;
  pointcloud.fields[2].count = 1;

  pointcloud.data.resize(pointcloud.row_step * pointcloud.height);

  float horizontal_increment = msg->horizontal_fov / msg->horizontal_samples;
  float vertical_increment = msg->vertical_fov / msg->vertical_samples;

  float horizontal_angle = -(float)(msg->horizontal_samples - 1) / 2 * horizontal_increment;
  float vertical_angle = -(float)(msg->vertical_samples - 1) / 2 * vertical_increment;

  for (int i_v = 0; i_v < msg->vertical_samples; i_v++)
  {
    for (int i_h = 0; i_h < msg->horizontal_samples; i_h++)
    {
      float range = msg->ranges[i_v * msg->horizontal_samples + i_h];  // range in meters
      float x = range;
      float y = range * -tan(horizontal_angle);
      float z = range * -tan(vertical_angle);

      memcpy(&pointcloud.data[(i_v * msg->horizontal_samples + i_h) * pointcloud.point_step], &x, 4);
      memcpy(&pointcloud.data[(i_v * msg->horizontal_samples + i_h) * pointcloud.point_step + 4], &y, 4);
      memcpy(&pointcloud.data[(i_v * msg->horizontal_samples + i_h) * pointcloud.point_step + 8], &z, 4);
      horizontal_angle += horizontal_increment;
    }
    vertical_angle += vertical_increment;
    horizontal_angle = -(float)(msg->horizontal_samples - 1) / 2 * horizontal_increment;
  }
}

template <typename T>
T calculateMean(const std::vector<T>& values)
{
  T sum = 0;
  for (auto& value : values)
  {
    sum += value;
  }
  return sum / values.size();
}

template <typename T>
T calculateMeanWithoutIQROutliers(const std::vector<T>& values)
{
  std::vector<T> sorted_values = values;
  std::sort(sorted_values.begin(), sorted_values.end());

  T iqr_range = sorted_values[sorted_values.size() * 3 / 4] - sorted_values[sorted_values.size() / 4];
  T lower_bound = sorted_values[sorted_values.size() / 4] - 1.5 * iqr_range;
  T upper_bound = sorted_values[sorted_values.size() * 3 / 4] + 1.5 * iqr_range;

  sorted_values.erase(
      std::remove_if(sorted_values.begin(), sorted_values.end(),
                     [lower_bound, upper_bound](T value) { return value < lower_bound || value > upper_bound; }),
      sorted_values.end());
  return calculateMean(sorted_values);
}

float convertToRange(const multizone_lidar_msgs::MultizoneRange::ConstPtr& msg)
{
  return calculateMeanWithoutIQROutliers(msg->ranges);
}

float convertToRange(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  std::vector<float> ranges;
  ranges.reserve(msg->width * msg->height);
  for (int i = 0; i < msg->width * msg->height; i++)
  {
    float x;
    memcpy(&x, &msg->data[i * msg->point_step], 4);
    ranges.push_back(x);
  }
  return calculateMeanWithoutIQROutliers(ranges);
}

}  // namespace MultizoneLidarConverter