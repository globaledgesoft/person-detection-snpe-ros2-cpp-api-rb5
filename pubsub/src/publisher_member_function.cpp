#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "custom_msg/msg/b_box.hpp"     // CHANGE

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<custom_msg::msg::BBox>("topic", 10);    // CHANGE
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = custom_msg::msg::BBox();                               // CHANGE
    message.timestamp = "this is now";    // CHANGE
    message.probability = 44.0;
    message.xmin = 1;
    message.ymin = 2;
    message.xmax = 100;
    message.ymax = 200;
    message.id = 22;
    message.cls = "COCO_CLASS";    // CHANGE
    message.collision_warning_status = 1;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.id);    // CHANGE
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<custom_msg::msg::BBox>::SharedPtr publisher_;         // CHANGE
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
