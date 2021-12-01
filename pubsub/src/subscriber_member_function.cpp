#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "custom_msg/msg/b_box.hpp"     // CHANGE
using std::placeholders::_1;

/*
string timestamp
float64 probability
int64 xmin
int64 ymin
int64 xmax
int64 ymax
int16 id
string cls
int64 collision_warning_status
*/

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<custom_msg::msg::BBox>(          // CHANGE
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const custom_msg::msg::BBox::SharedPtr msg) const       // CHANGE
  {
    RCLCPP_INFO(this->get_logger(), "Total Pedestrain: '%d'\n", msg->id);              // CHANGE
    RCLCPP_INFO(this->get_logger(), "Class Name: '%s'\n", msg->cls.c_str());              // CHANGE
    RCLCPP_INFO(this->get_logger(), "Received Event on: '%s'\n\n", msg->timestamp.c_str());              // CHANGE
  }
  rclcpp::Subscription<custom_msg::msg::BBox>::SharedPtr subscription_;       // CHANGE
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
