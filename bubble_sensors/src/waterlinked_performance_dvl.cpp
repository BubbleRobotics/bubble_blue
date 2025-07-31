/*
waterlinked_performance_dvl.cpp V1.0
Author: Henry Adam
Purpose:
This file is meant to act as the node representing the waterlinked performance DVL
for the Blue simulation used by Bubble Robotics. It follows the referred specs of the
DVL at this link: https://waterlinked.com/datasheets/dvl-a50
*/

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <random>

// for the transformations:
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class PerformanceDVL : public rclcpp::Node
{
public:
  PerformanceDVL()
      : Node("performance_dvl"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_)
  {
    // create/read the parameters for the sensor in the constructor:
    //  standard deviation
    this->declare_parameter<double>("noise_stddev", 0.05);
    this->get_parameter("noise_stddev", noise_stddev_);

    // defining which measurements will be noisy
    this->declare_parameter<double>("bias", 0.05);
    this->get_parameter("bias", bias_);

    this->declare_parameter<double>("long_term_inaccuracy_percentage", 0.1);
    this->get_parameter("long_term_inaccuracy_percentage", long_term_inaccuracies_);

    this->declare_parameter<double>("resolution_mms", 0.1);
    this->get_parameter("resolution_mms", resolution_mms_);

    this->declare_parameter<bool>("angular_twist", false);
    this->get_parameter("angular_twist", measure_ang_twist_);

    // Define the incoming and outgoing topics
    this->declare_parameter("input_topic", "/model/bluerov2/odometry");
    this->get_parameter("input_topic", input_topic_name_);

    this->declare_parameter("output_topic", "/dvl/velocity");
    this->get_parameter("output_topic", output_topic_name_);

    odometry_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
        input_topic_name_,
        10,
        std::bind(&PerformanceDVL::odom_callback, this, std::placeholders::_1));

    measurement_publisher = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
        output_topic_name_,
        10);

    // Declare the covariance
    cov_ = noise_stddev_ * noise_stddev_ + bias_ * bias_ + (long_term_inaccuracies_ / 100) * (long_term_inaccuracies_ / 100);
    resolution_ = resolution_mms_ / 1000;
    // seed the random generator:
    std::srand(std::time(0)); // seed w/ start time of program

    RCLCPP_INFO(this->get_logger(), "Noisy odometry relay started.");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", input_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic_name_.c_str());
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry &msg)
  {
    // Transform the odometry into a velocity at the DVL frame
    try
    {
      tf2::Vector3 v_dvl = get_dvl_velocity(msg);

      // Transform the velocity at the dvl frame into a noisy TwistWithCovariance Message
      geometry_msgs::msg::TwistWithCovarianceStamped noisy_msg = get_noisy_measurement(v_dvl,
                                                                                       msg.header);

      // publish the message
      measurement_publisher->publish(noisy_msg);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
    }
  }

  tf2::Vector3 get_dvl_velocity(const nav_msgs::msg::Odometry &msg)
  {

    geometry_msgs::msg::Vector3 transformed_velocity;

    geometry_msgs::msg::TransformStamped dvl_base_transform_stamped =
        tf_buffer_.lookupTransform("dvl_link", "base_link", tf2::TimePointZero);

    // testing has shown that the velocities being output from the /model/bluerov2/odometry topic are in base_link
    //  even though it says it's in map frame. smh. This transformation will translate from map to dvl frame.
    tf2::Vector3 v_base(msg.twist.twist.linear.x,
                        msg.twist.twist.linear.y,
                        msg.twist.twist.linear.z);

    tf2::Vector3 omega_base(msg.twist.twist.angular.x,
                            msg.twist.twist.angular.y,
                            msg.twist.twist.angular.z);

    // now translate this velocity into the velocity at the dvl link in the base link frame:
    tf2::Vector3 r_dvl_base(dvl_base_transform_stamped.transform.translation.x,
                            dvl_base_transform_stamped.transform.translation.y,
                            dvl_base_transform_stamped.transform.translation.z);

    tf2::Vector3 v_dvl_base = v_base + omega_base.cross(r_dvl_base);

    // finally, transform the velocity of the dvl in the base_link frame into the dvl_link frame
    tf2::Quaternion q_dvl_base(
        dvl_base_transform_stamped.transform.rotation.x,
        dvl_base_transform_stamped.transform.rotation.y,
        dvl_base_transform_stamped.transform.rotation.z,
        dvl_base_transform_stamped.transform.rotation.w);

    tf2::Matrix3x3 rot_dvl_base(q_dvl_base);

    tf2::Vector3 v_dvl = rot_dvl_base * v_dvl_base;
    return v_dvl;
  }

  geometry_msgs::msg::TwistWithCovarianceStamped get_noisy_measurement(
      const tf2::Vector3 &dvl_velocity, const std_msgs::msg::Header &msg_header)
  {

    geometry_msgs::msg::TwistWithCovarianceStamped noisy_msg;

    // create a gaussian distribution
    std::normal_distribution diff(0.0, noise_stddev_);

    // first add the header
    noisy_msg.header.stamp = msg_header.stamp;
    noisy_msg.header.frame_id = "dvl_link"; // make sure it registers that it's in the right link

    // compensate for long-term inaccuracies in the twist measurement.
    inaccurate_linear_twist_.x = dvl_velocity.x() *
                                 (1 + long_term_inaccuracies_ * (double)std::rand() / RAND_MAX);
    inaccurate_linear_twist_.y = dvl_velocity.y() *
                                 (1 + long_term_inaccuracies_ * (double)std::rand() / RAND_MAX);
    inaccurate_linear_twist_.z = dvl_velocity.z() *
                                 (1 + long_term_inaccuracies_ * (double)std::rand() / RAND_MAX);

    // now add some noise
    noisy_msg.twist.twist.linear.x = std::round((inaccurate_linear_twist_.x +
                                                 diff(rng_generator_)) /
                                                resolution_) *
                                     resolution_;
    noisy_msg.twist.twist.linear.y = std::round((inaccurate_linear_twist_.y +
                                                 diff(rng_generator_)) /
                                                resolution_) *
                                     resolution_;
    noisy_msg.twist.twist.linear.z = std::round((inaccurate_linear_twist_.z +
                                                 diff(rng_generator_)) /
                                                resolution_) *
                                     resolution_;

    // add covariance for state estimation pipeline. Since there is an EKF on the sensor itself, this should be provided for us
    noisy_msg.twist.covariance[0] = cov_;
    noisy_msg.twist.covariance[7] = cov_;
    noisy_msg.twist.covariance[14] = cov_;

    return noisy_msg;
  }

  // Declaring the ROS nodes
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr measurement_publisher;

  // Parameters
  double noise_stddev_;
  double long_term_inaccuracies_;
  double bias_;
  double cov_;
  double resolution_;
  double resolution_mms_;
  bool measure_ang_twist_;
  std::string input_topic_name_;
  std::string output_topic_name_;

  // ancillary variables
  std::default_random_engine rng_generator_;
  geometry_msgs::msg::Vector3 inaccurate_linear_twist_;

  // for getting the correct transformations
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

// define the main function which actually spins the sensor up
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerformanceDVL>());
  rclcpp::shutdown();
  return 0;
}
