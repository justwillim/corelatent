// use ros path to visualize the polynomial 3d trajectory
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include "interface/polynomial_3d_trajectory.hpp"

using namespace std::chrono_literals;

class Polynomial3dTrajectoryNode : public rclcpp::Node
{
public:
    Polynomial3dTrajectoryNode()
        : Node("polynomial_3d_trajectory_node")
    {
        // initialize the trajectory
        Eigen::MatrixXd coeff = Eigen::MatrixXd::Random(3, 7);
        poly3d_ = Polynomial3dTrajectory(coeff);

        // initialize the path publisher
        //! @todo vis curvature / velocity / acceleration / jerk etc.
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("polynomial_3d_trajectory", 10);

        // initialize the timer
        timer_ = this->create_wall_timer(500ms, std::bind(&Polynomial3dTrajectoryNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // random a new trajectory
        Eigen::MatrixXd coeff = Eigen::MatrixXd::Random(3, 7);
        poly3d_ = Polynomial3dTrajectory(coeff);
        // sample the trajectory
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        for (double t = 0; t < 1; t += 0.01)
        {
            Eigen::VectorXd sample = poly3d_.sample(t);
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = sample(0);
            pose.pose.position.y = sample(1);
            pose.pose.position.z = sample(2);
            path.poses.push_back(pose);
        }
        path_publisher_->publish(path);
    }

private:
    // data structure
    Polynomial3dTrajectory poly3d_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Polynomial3dTrajectoryNode>());
    rclcpp::shutdown();
    return 0;
}