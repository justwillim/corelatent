#include "simulator/rigid_jerk_angular_velocity_object.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;

class RigidJerkAngularVelocityObjectNode : public rclcpp::Node
{
public:
    RigidJerkAngularVelocityObjectNode()
        : Node("rigid_jerk_angular_velocity_object_node")
    {
        // initialize the object
        object_ = std::make_unique<RigidJerkAngularVelocityObject>();
        // initialize the timer
        timer_ = this->create_wall_timer(50ms, std::bind(&RigidJerkAngularVelocityObjectNode::timer_callback, this));
        // initialize the publisher
        state_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("rigid_jerk_angular_velocity_object", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

private:
    void timer_callback()
    {
        // step the object
        Eigen::VectorXd control = Eigen::VectorXd::Random(6);
        Eigen::VectorXd state = object_->step(control, 0.05);
        // publish the state
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = state(0, 3);
        pose.pose.position.y = state(1, 3);
        pose.pose.position.z = state(2, 3);
        auto q = Eigen::Quaterniond(state.block<3, 3>(0, 0));
        pose.pose.orientation.w = q.w();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        state_publisher_->publish(pose);

        // publish the tf
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "rigid_jerk_angular_velocity_object";
        transform.transform.translation.x = state(0, 3);
        transform.transform.translation.y = state(1, 3);
        transform.transform.translation.z = state(2, 3);
        auto q = Eigen::Quaterniond(state.block<3, 3>(0, 0));
        transform.transform.rotation.w = q.w();
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        tf_broadcaster_->sendTransform(transform);
    }

private:
    // data structure
    std::unique_ptr<RigidJerkAngularVelocityObject> object_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr state_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};