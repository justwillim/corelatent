// use ros path to visualize the polynomial 3d trajectory
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include "interface/polynomial_trajectory.hpp"
#include "interface/piecewise_trajectory.hpp"

using namespace std::chrono_literals;

class Polynomial3dTrajectoryNode : public rclcpp::Node
{
public:
    Polynomial3dTrajectoryNode()
        : Node("polynomial_3d_trajectory_node")
    {
        generate();
        // initialize the path publisher
        //! @todo vis curvature / velocity / acceleration / jerk etc.
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("polynomial_3d_trajectory", 10);

        // initialize the timer
        timer_ = this->create_wall_timer(500ms, std::bind(&Polynomial3dTrajectoryNode::timer_callback, this));
    }

private:
    void generate(){
        // initialize the piecewise trajectory
        //! @todo make it smooth
        piecewise_ = std::make_unique<PiecewiseTrajectory>();
        for(int i = 0; i < 3; i++)
        {
            Eigen::MatrixXd coeff = Eigen::MatrixXd::Random(3, 7);
            auto poly3d = std::make_unique<PolynomialTrajectory>(coeff);
            piecewise_->insert_piece(i, std::move(poly3d), 1.0);
        }
    }

private:
    void timer_callback()
    {
        // random a new trajectory
        generate();
        // sample the trajectory
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        for (double t = 0; t < 3; t += 0.01)
        {
            Eigen::VectorXd sample = piecewise_->sample(t);
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
    std::unique_ptr<PiecewiseTrajectory> piecewise_;
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