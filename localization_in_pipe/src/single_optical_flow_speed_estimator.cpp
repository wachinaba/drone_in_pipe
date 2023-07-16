#include <string>

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <gazebo_ros_optical_flow/OpticalFlow.h>


class SingleOpticalFlowSpeedEstimatorNode {
    public:
        SingleOpticalFlowSpeedEstimatorNode(std::string optical_flow_topic, std::string imu_topic, std::string range_topic) : 
            nh_("~"), 
            optical_flow_msg_(gazebo_ros_optical_flow::OpticalFlow()), 
            imu_msg_(sensor_msgs::Imu()), 
            range_msg_(sensor_msgs::Range()) {
            // init subscribers
            optical_flow_sub_ = nh_.subscribe(optical_flow_topic, 1, &SingleOpticalFlowSpeedEstimatorNode::OpticalFlowCallback, this);
            imu_sub_ = nh_.subscribe(imu_topic, 1, &SingleOpticalFlowSpeedEstimatorNode::ImuCallback, this);
            range_sub_ = nh_.subscribe(range_topic, 1, &SingleOpticalFlowSpeedEstimatorNode::RangeCallback, this);
        };
        ~SingleOpticalFlowSpeedEstimatorNode() {};

    private: 
        ros::NodeHandle nh_;

        // subscribers
        ros::Subscriber optical_flow_sub_;
        ros::Subscriber imu_sub_;
        ros::Subscriber range_sub_;

        // messages
        gazebo_ros_optical_flow::OpticalFlow optical_flow_msg_;
        sensor_msgs::Imu imu_msg_;
        sensor_msgs::Range range_msg_;
        

        void OpticalFlowCallback(const gazebo_ros_optical_flow::OpticalFlow::ConstPtr& msg) {
            optical_flow_msg_ = *msg;
        };
        void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
            imu_msg_ = *msg;
        };
        void RangeCallback(const sensor_msgs::Range::ConstPtr& msg){
            range_msg_ = *msg;
        };
};

class AngularVelocityEstimator {
    public:
        explicit AngularVelocityEstimator(Eigen::Isometry3d imu_transform) : 
            imu_header_(std_msgs::Header()), 
            angular_velocity_vector_(Eigen::Vector3d(0,0,0)) {
            imu_transform_ = imu_transform;
        }
        ~AngularVelocityEstimator() {};

        void update_imu(sensor_msgs::Imu imu_msg);
        Eigen::Vector3d get_angular_velocity_vector() {return angular_velocity_vector_;};
        std_msgs::Header get_imu_header() {return imu_header_;};

    private:
        Eigen::Isometry3d imu_transform_;
        std_msgs::Header imu_header_;
        Eigen::Vector3d angular_velocity_vector_;
        
};

void AngularVelocityEstimator::update_imu(sensor_msgs::Imu imu_msg) {
    imu_header_ = imu_msg.header;
    angular_velocity_vector_ = imu_transform_ * Eigen::Vector3d(imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z);
}


class RangeVectorEstimator {
    public:
        explicit RangeVectorEstimator(Eigen::Isometry3d optical_flow_transform) : 
            range_header_(std_msgs::Header()), 
            range_vector_(Eigen::Vector3d(0,0,0)) {
            optical_flow_transform_ = optical_flow_transform;
        }
        ~RangeVectorEstimator() {};

        void update_range(sensor_msgs::Range range_msg);
        Eigen::Vector3d get_range_vector() {return range_vector_;};
        std_msgs::Header get_range_header() {return range_header_;};

    private:
        Eigen::Isometry3d optical_flow_transform_;
        std_msgs::Header range_header_;
        Eigen::Vector3d range_vector_;
        
};

void RangeVectorEstimator::update_range(sensor_msgs::Range range_msg) {
    range_header_ = range_msg.header;
    range_vector_ = optical_flow_transform_ * Eigen::Vector3d(0,0,range_msg.range);
}


class VectorDerivator {
    public:
        explicit VectorDerivator() : 
            previous_vector_(Eigen::Vector3d(0,0,0)), 
            current_vector_(Eigen::Vector3d(0,0,0)), 
            previous_header_(std_msgs::Header()), 
            current_header_(std_msgs::Header()) {};
        ~VectorDerivator() {};

        void update_vector(Eigen::Vector3d vector, std_msgs::Header header);
        Eigen::Vector3d get_vector_diff();

    private:
        Eigen::Vector3d previous_vector_;
        Eigen::Vector3d current_vector_;
        std_msgs::Header previous_header_;
        std_msgs::Header current_header_;
        
};

void VectorDerivator::update_vector(Eigen::Vector3d vector, std_msgs::Header header) {
    previous_vector_ = current_vector_;
    current_vector_ = vector;
    previous_header_ = current_header_;
    current_header_ = header;
}

Eigen::Vector3d VectorDerivator::get_vector_diff() {
    float dt = (current_header_.stamp - previous_header_.stamp).toSec();
    if (dt == 0) {
        return Eigen::Vector3d(0,0,0);
    }
    return (current_vector_ - previous_vector_) / dt;
}


class OpticalFlowVectorEstimator {
    public:
        explicit OpticalFlowVectorEstimator(Eigen::Isometry3d optical_flow_transform) : 
            optical_flow_transform_(optical_flow_transform), 
            optical_flow_vector_(Eigen::Vector3d(0,0,0)) {};
        ~OpticalFlowVectorEstimator() {};

        void update_optical_flow(gazebo_ros_optical_flow::OpticalFlow optical_flow_msg);
        Eigen::Vector3d update_range(float range) {range_ = range;}
        Eigen::Vector3d get_optical_flow_vector() {return optical_flow_vector_;};
        std_msgs::Header get_optical_flow_header() {return optical_flow_header_;};

    private:
        Eigen::Isometry3d optical_flow_transform_;
        gazebo_ros_optical_flow::OpticalFlow optical_flow_msg_;
        Eigen::Vector3d optical_flow_vector_;
        std_msgs::Header optical_flow_header_;

        float range_;
        sensor_msgs::Range range_msg_;
        Eigen::Vector3d calc_optical_flow_vector();
        
};

void OpticalFlowVectorEstimator::update_optical_flow(gazebo_ros_optical_flow::OpticalFlow optical_flow_msg) {
    optical_flow_msg_ = optical_flow_msg;
    optical_flow_header_ = optical_flow_msg.header;
    optical_flow_vector_ = calc_optical_flow_vector();
}

Eigen::Vector3d OpticalFlowVectorEstimator::calc_optical_flow_vector() {
    Eigen::Vector3d optical_flow_vector = Eigen::Vector3d(optical_flow_msg_.integrated_x, optical_flow_msg_.integrated_y, 0).array().tan() * range_ / (optical_flow_msg_.integration_time_us * 1e-6);
    return optical_flow_transform_ * optical_flow_vector;
}



class SingleOpticalFlowSpeedEstimator {
    public:
        SingleOpticalFlowSpeedEstimator() {};
        ~SingleOpticalFlowSpeedEstimator() {};

    private:
        // messages
        gazebo_ros_optical_flow::OpticalFlow optical_flow_;
        Eigen::Vector3d angular_velocity_;
        Eigen::Vector3d range_;
        Eigen::Vector3d range_diff_;
        geometry_msgs::TransformStamped optical_flow_tf_;

        Eigen::Vector3d get_circumference_speed();
        Eigen::Vector3d get_linear_velocity();
};

Eigen::Vector3d SingleOpticalFlowSpeedEstimator::get_circumference_speed() {
    // calc cross product of angular velocity and range
    return angular_velocity_.cross(range_);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "single_optical_flow_speed_estimator");
    ros::NodeHandle nh;

    SingleOpticalFlowSpeedEstimatorNode estimator_node();
}

  