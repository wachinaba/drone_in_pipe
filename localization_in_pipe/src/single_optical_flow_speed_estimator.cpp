#include <string>
#include <memory>

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <gazebo_ros_optical_flow/OpticalFlow.h>


class AngularVelocityEstimator {
    public:
        AngularVelocityEstimator() : 
            imu_header_(std_msgs::Header()), 
            angular_velocity_vector_(Eigen::Vector3d(0,0,0)) {
        }
        ~AngularVelocityEstimator() {};

        void update_imu(const sensor_msgs::Imu::ConstPtr& imu_msg, const std::shared_ptr<const Eigen::Isometry3d>& transform);
        Eigen::Vector3d get_angular_velocity_vector() {return angular_velocity_vector_;};
        std_msgs::Header get_imu_header() {return imu_header_;};

    private:
        std_msgs::Header imu_header_;
        Eigen::Vector3d angular_velocity_vector_;
        
};

void AngularVelocityEstimator::update_imu(const sensor_msgs::Imu::ConstPtr& imu_msg, const std::shared_ptr<const Eigen::Isometry3d>& transform) {
    imu_header_ = imu_msg->header;
    angular_velocity_vector_ = transform->rotation() * Eigen::Vector3d(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
}


class RangeVectorEstimator {
    public:
        RangeVectorEstimator() : 
            range_header_(std_msgs::Header()), 
            range_vector_(Eigen::Vector3d(0,0,0)) {
        }
        ~RangeVectorEstimator() {};

        void update_range(const sensor_msgs::Range::ConstPtr& range_msg, const std::shared_ptr<const Eigen::Isometry3d>& transform);
        Eigen::Vector3d get_range_vector() {return range_vector_;};
        std_msgs::Header get_range_header() {return range_header_;};

    private:
        std_msgs::Header range_header_;
        Eigen::Vector3d range_vector_;
        
};

void RangeVectorEstimator::update_range(const sensor_msgs::Range::ConstPtr& range_msg, const std::shared_ptr<const Eigen::Isometry3d>& transform) {
    range_header_ = range_msg->header;
    range_vector_ = transform->rotation() * Eigen::Vector3d(0,0,range_msg->range);
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
        explicit OpticalFlowVectorEstimator(Eigen::Matrix2f optical_flow_covariance_matrix) : 
            optical_flow_vector_(Eigen::Vector3d(0,0,0)),
            optical_flow_covariance_matrix_(optical_flow_covariance_matrix),
            optical_flow_vector_covariance_matrix_(Eigen::Matrix3f::Zero()) {};
        ~OpticalFlowVectorEstimator() {};

        void update_optical_flow(const gazebo_ros_optical_flow::OpticalFlow::ConstPtr& optical_flow_msg, const std::shared_ptr<const Eigen::Isometry3d>& transform);
        void update_range(float range) {range_ = range;}
        Eigen::Vector3d get_optical_flow_vector() {return optical_flow_vector_;};
        std_msgs::Header get_optical_flow_header() {return optical_flow_header_;};

    private:
        gazebo_ros_optical_flow::OpticalFlow optical_flow_msg_;
        Eigen::Matrix2f optical_flow_covariance_matrix_;
        Eigen::Vector3d optical_flow_vector_;
        Eigen::Matrix3f optical_flow_vector_covariance_matrix_;
        std_msgs::Header optical_flow_header_;

        float range_;
        sensor_msgs::Range range_msg_;
        Eigen::Vector3d calc_optical_flow_vector(const std::shared_ptr<const Eigen::Isometry3d>& transform);
        
};

void OpticalFlowVectorEstimator::update_optical_flow(const gazebo_ros_optical_flow::OpticalFlow::ConstPtr& optical_flow_msg, const std::shared_ptr<const Eigen::Isometry3d>& transform) {
    optical_flow_msg_ = *optical_flow_msg;
    optical_flow_header_ = optical_flow_msg->header;
    optical_flow_vector_ = calc_optical_flow_vector(transform);
}

Eigen::Vector3d OpticalFlowVectorEstimator::calc_optical_flow_vector(const std::shared_ptr<const Eigen::Isometry3d>& transform) {
    Eigen::Vector3d optical_flow_vector = Eigen::Vector3d(optical_flow_msg_.integrated_x, optical_flow_msg_.integrated_y, 0).array().tan() * range_ / (optical_flow_msg_.integration_time_us * 1e-6);
    return transform->rotation() * optical_flow_vector;
}


class SingleOpticalFlowSpeedEstimator {
    public:
        SingleOpticalFlowSpeedEstimator(std::shared_ptr<AngularVelocityEstimator> angular_velocity_estimator, std::shared_ptr<RangeVectorEstimator> range_vector_estimator, std::shared_ptr<OpticalFlowVectorEstimator> optical_flow_vector_estimator) : 
            angular_velocity_estimator_(angular_velocity_estimator), 
            range_vector_estimator_(range_vector_estimator), 
            optical_flow_vector_estimator_(optical_flow_vector_estimator) {};
        ~SingleOpticalFlowSpeedEstimator() {};

        Eigen::Vector3d get_circumference_velocity();
        Eigen::Vector3d get_linear_velocity();

    private:
        // estimators
        std::shared_ptr<AngularVelocityEstimator> angular_velocity_estimator_;
        std::shared_ptr<RangeVectorEstimator> range_vector_estimator_;
        std::shared_ptr<OpticalFlowVectorEstimator> optical_flow_vector_estimator_;

};

Eigen::Vector3d SingleOpticalFlowSpeedEstimator::get_circumference_velocity() {
    Eigen::Vector3d angular_velocity_vector = angular_velocity_estimator_->get_angular_velocity_vector();
    Eigen::Vector3d range_vector = range_vector_estimator_->get_range_vector();
    Eigen::Vector3d circumference_velocity = angular_velocity_vector.cross(range_vector);
    return circumference_velocity;
}

Eigen::Vector3d SingleOpticalFlowSpeedEstimator::get_linear_velocity() {
    Eigen::Vector3d optical_flow_vector = optical_flow_vector_estimator_->get_optical_flow_vector();
    Eigen::Vector3d circumference_velocity = get_circumference_velocity();
    Eigen::Vector3d linear_velocity = optical_flow_vector - circumference_velocity;
    return linear_velocity;
}


class SingleOpticalFlowSpeedEstimatorNode {
    public:
        SingleOpticalFlowSpeedEstimatorNode(std::string optical_flow_topic, std::string imu_topic, std::string range_topic, std::string base_link, std::string optical_flow_frame, std::string imu_frame, Eigen::Matrix2f optical_flow_covariance_matrix);
        ~SingleOpticalFlowSpeedEstimatorNode() {};

    private: 
        ros::NodeHandle nh_;

        // subscribers
        ros::Subscriber optical_flow_sub_;
        ros::Subscriber imu_sub_;
        ros::Subscriber range_sub_;

        // publishers(for visualization)
        ros::Publisher linear_velocity_pub_;
        ros::Publisher circumference_velocity_pub_;

        // messages
        gazebo_ros_optical_flow::OpticalFlow optical_flow_msg_;
        sensor_msgs::Imu imu_msg_;
        sensor_msgs::Range range_msg_;

        // tf
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        // transforms
        std::shared_ptr<Eigen::Isometry3d> imu_transform_;
        std::shared_ptr<Eigen::Isometry3d> optical_flow_transform_;

        // estimators
        std::shared_ptr<AngularVelocityEstimator> angular_velocity_estimator_;
        std::shared_ptr<RangeVectorEstimator> range_vector_estimator_;
        std::shared_ptr<OpticalFlowVectorEstimator> optical_flow_vector_estimator_;

        // single optical flow speed estimator
        SingleOpticalFlowSpeedEstimator single_optical_flow_speed_estimator_;
        

        void OpticalFlowCallback(const gazebo_ros_optical_flow::OpticalFlow::ConstPtr& msg);
        void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg);
        void RangeCallback(const sensor_msgs::Range::ConstPtr& msg);
};

SingleOpticalFlowSpeedEstimatorNode::SingleOpticalFlowSpeedEstimatorNode(std::string optical_flow_topic, std::string imu_topic, std::string range_topic, std::string base_link, std::string optical_flow_frame, std::string imu_frame, Eigen::Matrix2f optical_flow_covariance_matrix) : 
            nh_("~"), 
            optical_flow_msg_(gazebo_ros_optical_flow::OpticalFlow()), 
            imu_msg_(sensor_msgs::Imu()), 
            range_msg_(sensor_msgs::Range()),
            tf_buffer_(),
            tf_listener_(tf_buffer_),
            imu_transform_(std::make_shared<Eigen::Isometry3d>(Eigen::Isometry3d::Identity())),
            optical_flow_transform_(std::make_shared<Eigen::Isometry3d>(Eigen::Isometry3d::Identity())),
            angular_velocity_estimator_(std::make_shared<AngularVelocityEstimator>()),
            range_vector_estimator_(std::make_shared<RangeVectorEstimator>()),
            optical_flow_vector_estimator_(std::make_shared<OpticalFlowVectorEstimator>(optical_flow_covariance_matrix)),
            single_optical_flow_speed_estimator_(SingleOpticalFlowSpeedEstimator(angular_velocity_estimator_, range_vector_estimator_, optical_flow_vector_estimator_)),
            linear_velocity_pub_(nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("linear_velocity", 1)),
            circumference_velocity_pub_(nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("circumference_velocity", 1)) {
            // init subscribers
            optical_flow_sub_ = nh_.subscribe(optical_flow_topic, 1, &SingleOpticalFlowSpeedEstimatorNode::OpticalFlowCallback, this);
            imu_sub_ = nh_.subscribe(imu_topic, 1, &SingleOpticalFlowSpeedEstimatorNode::ImuCallback, this);
            range_sub_ = nh_.subscribe(range_topic, 1, &SingleOpticalFlowSpeedEstimatorNode::RangeCallback, this);
            // get transforms
            geometry_msgs::TransformStamped imu_transform_msg = tf_buffer_.lookupTransform(base_link, imu_frame, ros::Time(0), ros::Duration(5.0));
            geometry_msgs::TransformStamped optical_flow_transform_msg = tf_buffer_.lookupTransform(base_link, optical_flow_frame, ros::Time(0), ros::Duration(5.0));
            // convert transforms to eigen::isometry3d
            *imu_transform_ = tf2::transformToEigen(imu_transform_msg.transform);
            *optical_flow_transform_ = tf2::transformToEigen(optical_flow_transform_msg.transform);
            
        };

void SingleOpticalFlowSpeedEstimatorNode::ImuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    imu_msg_ = *msg;
    angular_velocity_estimator_->update_imu(msg, imu_transform_);
}

void SingleOpticalFlowSpeedEstimatorNode::RangeCallback(const sensor_msgs::Range::ConstPtr& msg) {
    range_msg_ = *msg;
    range_vector_estimator_->update_range(msg, optical_flow_transform_);
}

void SingleOpticalFlowSpeedEstimatorNode::OpticalFlowCallback(const gazebo_ros_optical_flow::OpticalFlow::ConstPtr& msg) {
    optical_flow_msg_ = *msg;
    optical_flow_vector_estimator_->update_optical_flow(msg, optical_flow_transform_);
    Eigen::Vector3d linear_velocity = single_optical_flow_speed_estimator_.get_linear_velocity();
    Eigen::Vector3d circumference_velocity = single_optical_flow_speed_estimator_.get_circumference_velocity();
    ROS_INFO_STREAM("linear velocity: " << linear_velocity.transpose());
    ROS_INFO_STREAM("circumference speed: " << circumference_velocity.transpose());
    // publish linear velocity
    geometry_msgs::TwistWithCovarianceStamped linear_velocity_msg;
    linear_velocity_msg.header = optical_flow_vector_estimator_->get_optical_flow_header();
    linear_velocity_msg.twist.twist.linear.x = linear_velocity.x();
    linear_velocity_msg.twist.twist.linear.y = linear_velocity.y();
    linear_velocity_msg.twist.twist.linear.z = linear_velocity.z();
    linear_velocity_pub_.publish(linear_velocity_msg);
    // publish circumference velocity
    geometry_msgs::TwistWithCovarianceStamped circumference_velocity_msg;
    circumference_velocity_msg.header = optical_flow_vector_estimator_->get_optical_flow_header();
    circumference_velocity_msg.twist.twist.linear.x = circumference_velocity.x();
    circumference_velocity_msg.twist.twist.linear.y = circumference_velocity.y();
    circumference_velocity_msg.twist.twist.linear.z = circumference_velocity.z();
    circumference_velocity_pub_.publish(circumference_velocity_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "single_optical_flow_speed_estimator");
    SingleOpticalFlowSpeedEstimatorNode single_optical_flow_speed_estimator_node("/adrien/optical_flow_bottom_link/optical_flow", "/mavros/imu/data", "/single_point_lidar_bottom_link/range", "base_link", "optical_flow_bottom_link", "single_point_lidar_bottom_link", Eigen::Matrix2f::Identity());
    ros::spin();
}

  