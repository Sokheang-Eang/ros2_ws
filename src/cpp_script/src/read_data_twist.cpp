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
