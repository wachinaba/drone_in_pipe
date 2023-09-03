#include <string>
#include <memory>

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <mavros_msgs/OpticalFlowRad.h>

class AngularVelocityEstimator
{
public:
  AngularVelocityEstimator() : imu_header_(std_msgs::Header()), angular_velocity_vector_(Eigen::Vector3d(0, 0, 0))
  {
  }
  ~AngularVelocityEstimator(){};

  void update_imu(const sensor_msgs::Imu::ConstPtr& imu_msg, const std::shared_ptr<const Eigen::Isometry3d>& transform);
  Eigen::Vector3d get_angular_velocity_vector()
  {
    return angular_velocity_vector_;
  };
  std_msgs::Header get_imu_header()
  {
    return imu_header_;
  };

private:
  std_msgs::Header imu_header_;
  Eigen::Vector3d angular_velocity_vector_;
};

void AngularVelocityEstimator::update_imu(const sensor_msgs::Imu::ConstPtr& imu_msg,
                                          const std::shared_ptr<const Eigen::Isometry3d>& transform)
{
  imu_header_ = imu_msg->header;
  angular_velocity_vector_ =
      transform->rotation() *
      Eigen::Vector3d(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
}

class RangeVectorEstimator
{
public:
  RangeVectorEstimator() : range_header_(std_msgs::Header()), range_vector_(Eigen::Vector3d(0, 0, 0))
  {
  }
  ~RangeVectorEstimator(){};

  void update_range(const sensor_msgs::Range::ConstPtr& range_msg,
                    const std::shared_ptr<const Eigen::Isometry3d>& transform);
  Eigen::Vector3d get_range_vector()
  {
    return range_vector_;
  };
  std_msgs::Header get_range_header()
  {
    return range_header_;
  };

private:
  std_msgs::Header range_header_;
  Eigen::Vector3d range_vector_;
};

void RangeVectorEstimator::update_range(const sensor_msgs::Range::ConstPtr& range_msg,
                                        const std::shared_ptr<const Eigen::Isometry3d>& transform)
{
  range_header_ = range_msg->header;
  range_vector_ = transform->rotation() * Eigen::Vector3d(0, 0, range_msg->range);
}

class VectorDerivator
{
public:
  explicit VectorDerivator()
    : previous_vector_(Eigen::Vector3d(0, 0, 0))
    , current_vector_(Eigen::Vector3d(0, 0, 0))
    , previous_header_(std_msgs::Header())
    , current_header_(std_msgs::Header()){};
  ~VectorDerivator(){};

  void update_vector(Eigen::Vector3d vector, std_msgs::Header header);
  Eigen::Vector3d get_vector_diff();

private:
  Eigen::Vector3d previous_vector_;
  Eigen::Vector3d current_vector_;
  std_msgs::Header previous_header_;
  std_msgs::Header current_header_;
};

void VectorDerivator::update_vector(Eigen::Vector3d vector, std_msgs::Header header)
{
  previous_vector_ = current_vector_;
  current_vector_ = vector;
  previous_header_ = current_header_;
  current_header_ = header;
}

Eigen::Vector3d VectorDerivator::get_vector_diff()
{
  float dt = (current_header_.stamp - previous_header_.stamp).toSec();
  if (dt == 0)
  {
    return Eigen::Vector3d(0, 0, 0);
  }
  return (current_vector_ - previous_vector_) / dt;
}

class OpticalFlowVectorEstimator
{
public:
  explicit OpticalFlowVectorEstimator(Eigen::Matrix2d optical_flow_covariance_matrix)
    : optical_flow_vector_(Eigen::Vector3d(0, 0, 0))
    , optical_flow_covariance_matrix_(optical_flow_covariance_matrix)
    , optical_flow_vector_covariance_matrix_(Eigen::Matrix3d::Zero()){};
  ~OpticalFlowVectorEstimator(){};

  void update_optical_flow(const mavros_msgs::OpticalFlowRad::ConstPtr& optical_flow_msg,
                           const std::shared_ptr<const Eigen::Isometry3d>& transform, float range);
  Eigen::Vector3d get_optical_flow_vector()
  {
    return optical_flow_vector_;
  };
  std_msgs::Header get_optical_flow_header()
  {
    return optical_flow_header_;
  };
  Eigen::Matrix3d get_optical_flow_vector_covariance_matrix()
  {
    return optical_flow_vector_covariance_matrix_;
  };

private:
  mavros_msgs::OpticalFlowRad optical_flow_msg_;
  Eigen::Matrix2d optical_flow_covariance_matrix_;
  Eigen::Vector3d optical_flow_vector_;
  Eigen::Matrix3d optical_flow_vector_covariance_matrix_;
  std_msgs::Header optical_flow_header_;
  float average_fps_ = 60;

  Eigen::Vector3d calc_optical_flow_vector(const std::shared_ptr<const Eigen::Isometry3d>& transform, float range);
};

void OpticalFlowVectorEstimator::update_optical_flow(const mavros_msgs::OpticalFlowRad::ConstPtr& optical_flow_msg,
                                                     const std::shared_ptr<const Eigen::Isometry3d>& transform,
                                                     float range)
{
  optical_flow_msg_ = *optical_flow_msg;
  optical_flow_header_ = optical_flow_msg->header;
  optical_flow_vector_ = calc_optical_flow_vector(transform, range);
  // update optical flow vector covariance matrix
  Eigen::Matrix3d covariance_matrix;
  covariance_matrix.block<2, 2>(0, 0) = optical_flow_covariance_matrix_;
  covariance_matrix(2, 2) = 100;  // optical flow sensors can't measure z velocity
  // rotate covariance matrix
  optical_flow_vector_covariance_matrix_ =
      (transform->rotation()) * covariance_matrix * (transform->rotation().transpose());
}

Eigen::Vector3d OpticalFlowVectorEstimator::calc_optical_flow_vector(
    const std::shared_ptr<const Eigen::Isometry3d>& transform, float range)
{
  // calculate optical flow vector
  Eigen::Vector3d optical_flow_vector =
      Eigen::Vector3d(-optical_flow_msg_.integrated_x, -optical_flow_msg_.integrated_y, 0).array().tan() * range;

  // update average fps
  float integration_period = optical_flow_msg_.integration_time_us * 1e-6;
  if (integration_period == 0)
  {
    return Eigen::Vector3d(0, 0, 0);
  }

  float current_fps = 1.0 / integration_period;

  average_fps_ = 0.9 * average_fps_ + 0.1 * current_fps;
  ROS_INFO("average fps: %f, current fps: %f", average_fps_, current_fps);

  // calculate optical flow vector derivative
  optical_flow_vector = optical_flow_vector * average_fps_;

  return transform->rotation() * optical_flow_vector;
}

class SingleOpticalFlowSpeedEstimator
{
public:
  SingleOpticalFlowSpeedEstimator(std::shared_ptr<AngularVelocityEstimator> angular_velocity_estimator,
                                  std::shared_ptr<RangeVectorEstimator> range_vector_estimator,
                                  std::shared_ptr<OpticalFlowVectorEstimator> optical_flow_vector_estimator)
    : angular_velocity_estimator_(angular_velocity_estimator)
    , range_vector_estimator_(range_vector_estimator)
    , optical_flow_vector_estimator_(optical_flow_vector_estimator){};
  ~SingleOpticalFlowSpeedEstimator(){};

  Eigen::Vector3d get_circumference_velocity();
  Eigen::Vector3d get_linear_velocity();

private:
  // estimators
  std::shared_ptr<AngularVelocityEstimator> angular_velocity_estimator_;
  std::shared_ptr<RangeVectorEstimator> range_vector_estimator_;
  std::shared_ptr<OpticalFlowVectorEstimator> optical_flow_vector_estimator_;
};

Eigen::Vector3d SingleOpticalFlowSpeedEstimator::get_circumference_velocity()
{
  Eigen::Vector3d angular_velocity_vector = angular_velocity_estimator_->get_angular_velocity_vector();
  Eigen::Vector3d range_vector = range_vector_estimator_->get_range_vector();
  Eigen::Vector3d circumference_velocity = angular_velocity_vector.cross(range_vector);
  return circumference_velocity;
}

Eigen::Vector3d SingleOpticalFlowSpeedEstimator::get_linear_velocity()
{
  Eigen::Vector3d optical_flow_vector = optical_flow_vector_estimator_->get_optical_flow_vector();
  Eigen::Vector3d circumference_velocity = get_circumference_velocity();
  Eigen::Vector3d linear_velocity = optical_flow_vector - circumference_velocity;
  // limit linear velocity to 100 m/s
  if (linear_velocity.norm() > 100)
  {
    linear_velocity = linear_velocity.normalized() * 100;
  }
  return linear_velocity;
}

class SingleOpticalFlowSpeedEstimatorNode
{
public:
  SingleOpticalFlowSpeedEstimatorNode(std::string optical_flow_topic, std::string imu_topic, std::string range_topic,
                                      std::string base_link, std::string optical_flow_frame, std::string imu_frame,
                                      Eigen::Matrix2d optical_flow_covariance_matrix);
  ~SingleOpticalFlowSpeedEstimatorNode(){};

private:
  ros::NodeHandle nh_;

  // subscribers
  ros::Subscriber optical_flow_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber range_sub_;

  // publishers
  ros::Publisher optical_flow_velocity_pub_;
  ros::Publisher linear_velocity_pub_;
  ros::Publisher circumference_velocity_pub_;
  ros::Publisher angular_velocity_pub_;

  // publishers(for visualization)
  ros::Publisher linear_velocity_vis_pub_;
  ros::Publisher circumference_velocity_vis_pub_;
  ros::Publisher angular_velocity_vis_pub_;

  // messages
  mavros_msgs::OpticalFlowRad optical_flow_msg_;
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

  void OpticalFlowCallback(const mavros_msgs::OpticalFlowRad::ConstPtr& msg);
  void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void RangeCallback(const sensor_msgs::Range::ConstPtr& msg);

  void publish_velocities();
  void publish_visualization_markers();
};

SingleOpticalFlowSpeedEstimatorNode::SingleOpticalFlowSpeedEstimatorNode(
    std::string optical_flow_topic, std::string imu_topic, std::string range_topic, std::string base_link,
    std::string optical_flow_frame, std::string imu_frame, Eigen::Matrix2d optical_flow_covariance_matrix)
  : nh_("~")
  , optical_flow_msg_(mavros_msgs::OpticalFlowRad())
  , imu_msg_(sensor_msgs::Imu())
  , range_msg_(sensor_msgs::Range())
  , tf_buffer_()
  , tf_listener_(tf_buffer_)
  , imu_transform_(std::make_shared<Eigen::Isometry3d>(Eigen::Isometry3d::Identity()))
  , optical_flow_transform_(std::make_shared<Eigen::Isometry3d>(Eigen::Isometry3d::Identity()))
  , angular_velocity_estimator_(std::make_shared<AngularVelocityEstimator>())
  , range_vector_estimator_(std::make_shared<RangeVectorEstimator>())
  , optical_flow_vector_estimator_(std::make_shared<OpticalFlowVectorEstimator>(optical_flow_covariance_matrix))
  , single_optical_flow_speed_estimator_(SingleOpticalFlowSpeedEstimator(
        angular_velocity_estimator_, range_vector_estimator_, optical_flow_vector_estimator_))
  , optical_flow_velocity_pub_(nh_.advertise<geometry_msgs::TwistStamped>("optical_flow_velocity", 1))
  , linear_velocity_pub_(nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("linear_velocity", 1))
  , circumference_velocity_pub_(nh_.advertise<geometry_msgs::TwistStamped>("circumference_velocity", 1))
  , angular_velocity_pub_(nh_.advertise<geometry_msgs::TwistStamped>("angular_velocity", 1))
  , linear_velocity_vis_pub_(nh_.advertise<visualization_msgs::Marker>("linear_velocity_marker", 1))
  , circumference_velocity_vis_pub_(nh_.advertise<visualization_msgs::Marker>("circumference_velocity_marker", 1))
  , angular_velocity_vis_pub_(nh_.advertise<visualization_msgs::Marker>("angular_velocity_marker", 1))
{
  // init subscribers
  optical_flow_sub_ =
      nh_.subscribe(optical_flow_topic, 1, &SingleOpticalFlowSpeedEstimatorNode::OpticalFlowCallback, this);
  imu_sub_ = nh_.subscribe(imu_topic, 1, &SingleOpticalFlowSpeedEstimatorNode::ImuCallback, this);
  range_sub_ = nh_.subscribe(range_topic, 1, &SingleOpticalFlowSpeedEstimatorNode::RangeCallback, this);
  // get transforms
  geometry_msgs::TransformStamped imu_transform_msg =
      tf_buffer_.lookupTransform(base_link, imu_frame, ros::Time(0), ros::Duration(5.0));
  geometry_msgs::TransformStamped optical_flow_transform_msg =
      tf_buffer_.lookupTransform(base_link, optical_flow_frame, ros::Time(0), ros::Duration(5.0));
  // convert transforms to eigen::isometry3d
  *imu_transform_ = tf2::transformToEigen(imu_transform_msg.transform);
  *optical_flow_transform_ = tf2::transformToEigen(optical_flow_transform_msg.transform);
};

void SingleOpticalFlowSpeedEstimatorNode::ImuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu_msg_ = *msg;
  angular_velocity_estimator_->update_imu(msg, imu_transform_);
}

void SingleOpticalFlowSpeedEstimatorNode::RangeCallback(const sensor_msgs::Range::ConstPtr& msg)
{
  range_msg_ = *msg;
  range_vector_estimator_->update_range(msg, optical_flow_transform_);
}

void SingleOpticalFlowSpeedEstimatorNode::OpticalFlowCallback(const mavros_msgs::OpticalFlowRad::ConstPtr& msg)
{
  optical_flow_msg_ = *msg;
  optical_flow_vector_estimator_->update_optical_flow(msg, optical_flow_transform_, range_msg_.range);
  Eigen::Vector3d linear_velocity = single_optical_flow_speed_estimator_.get_linear_velocity();
  Eigen::Vector3d circumference_velocity = single_optical_flow_speed_estimator_.get_circumference_velocity();
  Eigen::Matrix3d optical_flow_vector_covariance_matrix =
      optical_flow_vector_estimator_->get_optical_flow_vector_covariance_matrix();

  // publish velocities
  geometry_msgs::TwistStamped optical_flow_velocity_msg;
  optical_flow_velocity_msg.header = optical_flow_vector_estimator_->get_optical_flow_header();
  optical_flow_velocity_msg.header.frame_id = "base_link";
  optical_flow_velocity_msg.twist.linear.x = optical_flow_vector_estimator_->get_optical_flow_vector().x();
  optical_flow_velocity_msg.twist.linear.y = optical_flow_vector_estimator_->get_optical_flow_vector().y();
  optical_flow_velocity_msg.twist.linear.z = optical_flow_vector_estimator_->get_optical_flow_vector().z();
  optical_flow_velocity_pub_.publish(optical_flow_velocity_msg);

  geometry_msgs::TwistWithCovarianceStamped linear_velocity_msg;
  linear_velocity_msg.header = optical_flow_vector_estimator_->get_optical_flow_header();
  linear_velocity_msg.header.frame_id = "base_link";
  linear_velocity_msg.twist.twist.linear.x = linear_velocity.x();
  linear_velocity_msg.twist.twist.linear.y = linear_velocity.y();
  linear_velocity_msg.twist.twist.linear.z = linear_velocity.z();
  Eigen::Matrix<double, 6, 6> covariance_matrix = Eigen::Matrix<double, 6, 6>::Zero();
  covariance_matrix.block<3, 3>(0, 0) = optical_flow_vector_covariance_matrix;
  std::copy(covariance_matrix.data(), covariance_matrix.data() + covariance_matrix.size(),
            linear_velocity_msg.twist.covariance.begin());
  linear_velocity_pub_.publish(linear_velocity_msg);

  geometry_msgs::TwistStamped circumference_velocity_msg;
  circumference_velocity_msg.header = optical_flow_vector_estimator_->get_optical_flow_header();
  circumference_velocity_msg.header.frame_id = "base_link";
  circumference_velocity_msg.twist.linear.x = circumference_velocity.x();
  circumference_velocity_msg.twist.linear.y = circumference_velocity.y();
  circumference_velocity_msg.twist.linear.z = circumference_velocity.z();
  circumference_velocity_pub_.publish(circumference_velocity_msg);

  geometry_msgs::TwistStamped angular_velocity_msg;
  angular_velocity_msg.header = angular_velocity_estimator_->get_imu_header();
  angular_velocity_msg.header.frame_id = "base_link";
  angular_velocity_msg.twist.angular.x = angular_velocity_estimator_->get_angular_velocity_vector().x();
  angular_velocity_msg.twist.angular.y = angular_velocity_estimator_->get_angular_velocity_vector().y();
  angular_velocity_msg.twist.angular.z = angular_velocity_estimator_->get_angular_velocity_vector().z();
  angular_velocity_pub_.publish(angular_velocity_msg);

  publish_visualization_markers();
}

void SingleOpticalFlowSpeedEstimatorNode::publish_velocities()
{
  Eigen::Vector3d angular_velocity = angular_velocity_estimator_->get_angular_velocity_vector();
  Eigen::Vector3d linear_velocity = single_optical_flow_speed_estimator_.get_linear_velocity();
  Eigen::Vector3d circumference_velocity = single_optical_flow_speed_estimator_.get_circumference_velocity();

  // publish linear velocity
  geometry_msgs::TwistWithCovarianceStamped linear_velocity_msg;
  linear_velocity_msg.header = optical_flow_vector_estimator_->get_optical_flow_header();
  linear_velocity_msg.header.frame_id = "base_link";
  linear_velocity_msg.twist.twist.linear.x = linear_velocity.x();
  linear_velocity_msg.twist.twist.linear.y = linear_velocity.y();
  linear_velocity_msg.twist.twist.linear.z = linear_velocity.z();
}

void SingleOpticalFlowSpeedEstimatorNode::publish_visualization_markers()
{
  // publish linear velocity as marker
  visualization_msgs::Marker linear_velocity_msg;
  linear_velocity_msg.header = optical_flow_vector_estimator_->get_optical_flow_header();
  // set frame id to base_link
  linear_velocity_msg.header.frame_id = "base_link";
  linear_velocity_msg.ns = "linear_velocity";
  linear_velocity_msg.id = 0;
  linear_velocity_msg.type = visualization_msgs::Marker::ARROW;
  linear_velocity_msg.action = visualization_msgs::Marker::ADD;
  linear_velocity_msg.pose.position.x = 0;
  linear_velocity_msg.pose.position.y = 0;
  linear_velocity_msg.pose.position.z = 0;
  linear_velocity_msg.pose.orientation.x = 0;
  linear_velocity_msg.pose.orientation.y = 0;
  linear_velocity_msg.pose.orientation.z = 0;
  linear_velocity_msg.pose.orientation.w = 1;
  linear_velocity_msg.scale.x = 0.1;
  linear_velocity_msg.scale.y = 0.1;
  linear_velocity_msg.scale.z = 0.1;
  linear_velocity_msg.color.a = 1.0;
  linear_velocity_msg.color.r = 0.0;
  linear_velocity_msg.color.g = 1.0;
  linear_velocity_msg.color.b = 0.0;
  linear_velocity_msg.points.resize(2);
  linear_velocity_msg.points[0].x = 0;
  linear_velocity_msg.points[0].y = 0;
  linear_velocity_msg.points[0].z = 0;
  linear_velocity_msg.points[1].x = single_optical_flow_speed_estimator_.get_linear_velocity().x();
  linear_velocity_msg.points[1].y = single_optical_flow_speed_estimator_.get_linear_velocity().y();
  linear_velocity_msg.points[1].z = single_optical_flow_speed_estimator_.get_linear_velocity().z();
  linear_velocity_vis_pub_.publish(linear_velocity_msg);

  // publish circumference velocity as marker
  visualization_msgs::Marker circumference_velocity_msg;
  circumference_velocity_msg.header = optical_flow_vector_estimator_->get_optical_flow_header();
  // set frame id to base_link
  circumference_velocity_msg.header.frame_id = "base_link";
  circumference_velocity_msg.ns = "circumference_velocity";
  circumference_velocity_msg.id = 0;
  circumference_velocity_msg.type = visualization_msgs::Marker::ARROW;
  circumference_velocity_msg.action = visualization_msgs::Marker::ADD;
  circumference_velocity_msg.pose.position.x = 0;
  circumference_velocity_msg.pose.position.y = 0;
  circumference_velocity_msg.pose.position.z = 0;
  circumference_velocity_msg.pose.orientation.x = 0;
  circumference_velocity_msg.pose.orientation.y = 0;
  circumference_velocity_msg.pose.orientation.z = 0;
  circumference_velocity_msg.pose.orientation.w = 1;
  circumference_velocity_msg.scale.x = 0.1;
  circumference_velocity_msg.scale.y = 0.1;
  circumference_velocity_msg.scale.z = 0.1;
  circumference_velocity_msg.color.a = 1.0;
  circumference_velocity_msg.color.r = 0.0;
  circumference_velocity_msg.color.g = 0.0;
  circumference_velocity_msg.color.b = 1.0;
  circumference_velocity_msg.points.resize(2);
  circumference_velocity_msg.points[0].x = 0;
  circumference_velocity_msg.points[0].y = 0;
  circumference_velocity_msg.points[0].z = 0;
  circumference_velocity_msg.points[1].x = single_optical_flow_speed_estimator_.get_circumference_velocity().x();
  circumference_velocity_msg.points[1].y = single_optical_flow_speed_estimator_.get_circumference_velocity().y();
  circumference_velocity_msg.points[1].z = single_optical_flow_speed_estimator_.get_circumference_velocity().z();
  circumference_velocity_vis_pub_.publish(circumference_velocity_msg);

  // publish angular velocity as marker
  visualization_msgs::Marker angular_velocity_msg;
  angular_velocity_msg.header = angular_velocity_estimator_->get_imu_header();
  // set frame id to base_link
  angular_velocity_msg.header.frame_id = "base_link";
  angular_velocity_msg.ns = "angular_velocity";
  angular_velocity_msg.id = 0;
  angular_velocity_msg.type = visualization_msgs::Marker::ARROW;
  angular_velocity_msg.action = visualization_msgs::Marker::ADD;
  angular_velocity_msg.pose.position.x = 0;
  angular_velocity_msg.pose.position.y = 0;
  angular_velocity_msg.pose.position.z = 0;
  angular_velocity_msg.pose.orientation.x = 0;
  angular_velocity_msg.pose.orientation.y = 0;
  angular_velocity_msg.pose.orientation.z = 0;
  angular_velocity_msg.pose.orientation.w = 1;
  angular_velocity_msg.scale.x = 0.1;
  angular_velocity_msg.scale.y = 0.1;
  angular_velocity_msg.scale.z = 0.1;
  angular_velocity_msg.color.a = 1.0;
  angular_velocity_msg.color.r = 1.0;
  angular_velocity_msg.color.g = 0.0;
  angular_velocity_msg.color.b = 0.0;
  angular_velocity_msg.points.resize(2);
  angular_velocity_msg.points[0].x = 0;
  angular_velocity_msg.points[0].y = 0;
  angular_velocity_msg.points[0].z = 0;
  angular_velocity_msg.points[1].x = angular_velocity_estimator_->get_angular_velocity_vector().x();
  angular_velocity_msg.points[1].y = angular_velocity_estimator_->get_angular_velocity_vector().y();
  angular_velocity_msg.points[1].z = angular_velocity_estimator_->get_angular_velocity_vector().z();
  angular_velocity_vis_pub_.publish(angular_velocity_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "single_optical_flow_speed_estimator");
  SingleOpticalFlowSpeedEstimatorNode single_optical_flow_speed_estimator_node(
      "/adrien/optical_flow_bottom_link/optical_flow", "/mavros/imu/data", "/single_point_lidar_bottom_link/range",
      "base_link", "optical_flow_bottom_link", "imu_link", Eigen::Matrix2d::Identity());
  ros::spin();
}
