#include "simulator/2d_jerk_object.hpp"
#include "simulator/jerk_2d_controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;

class Jerk2DObjectNode : public rclcpp::Node
{
public:
    Jerk2DObjectNode()
        : Node("jerk_2d_object_node")
    {
        // initialize the object
        object_ = std::make_unique<Jerk2DObject>();

        // controller
        Jerk2DController::Pid3D pid;
        pid << 0.5, 0.005, 0.01, // Kp, Ki, Kd for p
            2.0, 0.005, 0.01,    // Kp, Ki, Kd for v
            2.0, 0.005, 0.01;    // Kp, Ki, Kd for a
        controller_ = std::make_unique<Jerk2DController>(pid.transpose(), 1.0);

        // initialize the timer
        timer_ = this->create_wall_timer(50ms, std::bind(&Jerk2DObjectNode::timer_callback, this));

        // initialize the publisher
        std::string ns = this->get_namespace();
        std::string name = this->get_name();

        state_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(ns + name + "/pose", 10);
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Point>(ns + name + "/velocity", 10);
        acceleration_publisher_ = this->create_publisher<geometry_msgs::msg::Point>(ns + name + "/acceleration", 10);
        error_x_publisher_ = this->create_publisher<geometry_msgs::msg::Point>(ns + name + "/error_x", 10);
        error_y_publisher_ = this->create_publisher<geometry_msgs::msg::Point>(ns + name + "/error_y", 10);
        intt_x_publisher_ = this->create_publisher<geometry_msgs::msg::Point>(ns + name + "/intt_x", 10);
        intt_y_publisher_ = this->create_publisher<geometry_msgs::msg::Point>(ns + name + "/intt_y", 10);

        // tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

private:
    void timer_callback()
    {
        // header
        auto header = std_msgs::msg::Header();
        header.frame_id = "map";
        header.stamp = this->now();

        // step the object
        Eigen::VectorXd target = Eigen::VectorXd::Random(6);
        target << 3.0, 3.0, // p
            0.0, 0.0,       // v
            0.0, 0.0;       // a
        // state p | v | a
        auto state_p = object_->state();
        auto state_v = object_->velocity();
        auto state_a = object_->acceleration();
        Eigen::VectorXd state = Eigen::VectorXd::Zero(6);
        state << state_p, state_v, state_a;
        // RCLCPP_INFO_STREAM(this->get_logger(), "state: " << state.transpose());
    
        auto control = controller_->control(state, target);
        state.block<2, 1>(0, 0) = object_->step(control, 0.05);
        state.block<2, 1>(2, 0) = object_->velocity();
        state.block<2, 1>(4, 0) = object_->acceleration();

        // publish the state
        geometry_msgs::msg::PoseStamped pose;
        pose.header = header;
        pose.pose.position.x = state(0);
        pose.pose.position.y = state(1);
        // RCLCPP_INFO_STREAM(this->get_logger(), "state: " << state);
        pose.pose.position.z = 0.0;
        auto q = Eigen::Quaterniond(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()));
        pose.pose.orientation.w = q.w();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        state_publisher_->publish(pose);

        // publish the velocity
        geometry_msgs::msg::Point velocity;
        velocity.x = state(2);
        velocity.y = state(3);
        velocity.z = 0.0;
        velocity_publisher_->publish(velocity);

        // publish the acceleration
        geometry_msgs::msg::Point acceleration;
        acceleration.x = state(4);
        acceleration.y = state(5);
        acceleration.z = 0.0;
        acceleration_publisher_->publish(acceleration);

        // publish the error_x
        geometry_msgs::msg::Point error_x;
        error_x.x = controller_->prev_error_x()(0);
        error_x.y = controller_->prev_error_x()(1);
        error_x.z = controller_->prev_error_x()(2);
        error_x_publisher_->publish(error_x);

        // publish the error_y
        geometry_msgs::msg::Point error_y;
        error_y.x = controller_->prev_error_y()(0);
        error_y.y = controller_->prev_error_y()(1);
        error_y.z = controller_->prev_error_y()(2);
        error_y_publisher_->publish(error_y);

        // publish the intt_x
        geometry_msgs::msg::Point intt_x;
        intt_x.x = controller_->intt_error_x()(0);
        intt_x.y = controller_->intt_error_x()(1);
        intt_x.z = controller_->intt_error_x()(2);
        intt_x_publisher_->publish(intt_x);

        // publish the intt_y
        geometry_msgs::msg::Point intt_y;
        intt_y.x = controller_->intt_error_y()(0);
        intt_y.y = controller_->intt_error_y()(1);
        intt_y.z = controller_->intt_error_y()(2);
        intt_y_publisher_->publish(intt_y);
    }

private:
    // data structure
    std::unique_ptr<Jerk2DObject> object_;
    std::unique_ptr<Jerk2DController> controller_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr state_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr velocity_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr acceleration_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr error_x_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr error_y_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr intt_x_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr intt_y_publisher_;
    // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Jerk2DObjectNode>());
    rclcpp::shutdown();
    return 0;
}