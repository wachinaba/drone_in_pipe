#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class LevelPlaneTfBroadcaster
{
public:
  LevelPlaneTfBroadcaster()
  {
    // Initialize ROS node
    ros::NodeHandle nh;

    // Subscribe to IMU data topic
    imu_sub_ = nh.subscribe("mavros/imu/data", 10, &LevelPlaneTfBroadcaster::imuCallback, this);

    // Initialize transform broadcaster
    tf_broadcaster_ = new tf2_ros::TransformBroadcaster();
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
  {
    // Extract pitch and roll angles from IMU message
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Create transform message
    geometry_msgs::TransformStamped transform_msg;
    transform_msg.header.stamp = ros::Time::now();
    transform_msg.header.frame_id = "base_link";
    transform_msg.child_frame_id = "level_plane";

    // Set translation to zero
    transform_msg.transform.translation.x = 0.0;
    transform_msg.transform.translation.y = 0.0;
    transform_msg.transform.translation.z = 0.0;

    // Set rotation to correct for pitch and roll angles
    tf2::Quaternion level_q;
    level_q.setRPY(-roll, -pitch, 0.0);
    transform_msg.transform.rotation.x = level_q.x();
    transform_msg.transform.rotation.y = level_q.y();
    transform_msg.transform.rotation.z = level_q.z();
    transform_msg.transform.rotation.w = level_q.w();

    // Broadcast transform
    tf_broadcaster_->sendTransform(transform_msg);
  }

private:
  ros::Subscriber imu_sub_;
  tf2_ros::TransformBroadcaster* tf_broadcaster_;
};

int main(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "level_plane_tf_broadcaster");

  // Create LevelPlaneTfBroadcaster object
  LevelPlaneTfBroadcaster broadcaster;

  // Spin ROS node
  ros::spin();

  return 0;
}
