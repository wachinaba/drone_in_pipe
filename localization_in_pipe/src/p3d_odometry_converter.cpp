#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ros::Publisher pub;

void convertWorldToBaseLinkVelocity(const geometry_msgs::Vector3& world_linear,
                                    const geometry_msgs::Vector3& world_angular,
                                    const geometry_msgs::Quaternion& base_link_orientation,
                                    geometry_msgs::Vector3& base_link_linear,
                                    geometry_msgs::Vector3& base_link_angular)
{
    // Convert the world frame linear velocity to base_link frame
    tf2::Quaternion q;
    tf2::fromMsg(base_link_orientation, q);
    tf2::Matrix3x3 m(q);
    tf2::Vector3 v(world_linear.x, world_linear.y, world_linear.z);
    tf2::Vector3 v_base_link = m.inverse() * v;
    base_link_linear.x = v_base_link.x();
    base_link_linear.y = v_base_link.y();
    base_link_linear.z = v_base_link.z();

    // Convert the world frame angular velocity to base_link frame
    tf2::Vector3 w(world_angular.x, world_angular.y, world_angular.z);
    tf2::Vector3 w_base_link = m.inverse() * w;
    base_link_angular.x = w_base_link.x();
    base_link_angular.y = w_base_link.y();
    base_link_angular.z = w_base_link.z();

}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::TwistStamped base_link_twist;
    base_link_twist.header.stamp = msg->header.stamp;
    base_link_twist.header.frame_id = "base_link";

    // Convert the world frame linear velocity and angular velocity to base_link frame
    convertWorldToBaseLinkVelocity(msg->twist.twist.linear, msg->twist.twist.angular, msg->pose.pose.orientation, base_link_twist.twist.linear, base_link_twist.twist.angular);

    // Publish the transformed twist message
    pub.publish(base_link_twist);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gazebo_twist_converter_node");

    ros::NodeHandle nh("~");
    pub = nh.advertise<geometry_msgs::TwistStamped>("twist", 1);
    ros::Subscriber sub = nh.subscribe("/trackers/base_link/joint_state", 1, odometryCallback);

    ros::spin();

    return 0;
}