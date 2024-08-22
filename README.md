# ROS2_Basic
- For ROBOCON 2024 Member
# ROS2 Workspace
> [!TIP]
> Create First workspace
```
mkdir ros2_ws
cd ros2_ws
mkdir src
colcon build
```
# ROS2 Programming
> [!TIP]
> C++ Example pub/sub 
```
ros2 pkg create --build-type ament_cmake --license Apache-2.0 cpp_pubsub
```
### Create Demo Publisher
```
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
      : Node("cpp_node"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic_toxic_cpp", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```
### Create Demo Subscriber
```
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
      : Node("cpp_node")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic_toxic_cpp", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```
> [!NOTE]
### run script file
```
ros2 run cpp_pubsub talker
```
```
ros2 run cpp_pubsub listener
```
> [!TIP]
> Python Example pub/sub 
```
ros2 pkg create --build-type ament_python --license Apache-2.0 py_pubsub
```
