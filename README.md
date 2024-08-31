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
### Create Demo Publisher CPP
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
### Create Demo Subscriber CPP
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
> [!TIP]
> run script file
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
### Create Demo Publisher PY
```
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic_toxic_py', 10)
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
### Create Demo Subscriber PY
```
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(String,'topic_toxic_py',self.listener_callback,10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().warning('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
### Create Demo Subscribe data from twist
```
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class TwistSubscriber : public rclcpp::Node
{
public:
    TwistSubscriber() : Node("twist_subscriber")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&TwistSubscriber::twist_callback, this, std::placeholders::_1));
    }

private:
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Received Twist message:");
        RCLCPP_INFO(this->get_logger(), "Linear X: %.2f, Y: %.2f, Z: %.2f",
                    msg->linear.x, msg->linear.y, msg->linear.z);
        RCLCPP_INFO(this->get_logger(), "Angular X: %.2f, Y: %.2f, Z: %.2f",
                    msg->angular.x, msg->angular.y, msg->angular.z);
    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistSubscriber>());
    rclcpp::shutdown();
    return 0;
}
```
