#include <rclcpp/rclcpp.hpp>

#include "interface/msg/polynomial.hpp"

using namespace std::chrono_literals;

class PolynomialPublisher : public rclcpp::Node
{
public:
  PolynomialPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<interface::msg::Polynomial>("topic", 10); 
    timer_ = this->create_wall_timer(
      500ms, std::bind(&PolynomialPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = interface::msg::Polynomial();
    message.coefficients = {(double)count_, 2, 3, 4, 5};                             
    count_++;                                        
    RCLCPP_INFO(this->get_logger(), "Publishing: '%f' size: %ld", message.coefficients[0], message.coefficients.size());    
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<interface::msg::Polynomial>::SharedPtr publisher_;         
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PolynomialPublisher>());
  rclcpp::shutdown();
  return 0;
}