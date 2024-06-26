#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher() : 
        Node("number_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::UInt32>("number", 10); // Erstellung des Publishers für 'number'
        timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::UInt32(); // Erstellung einer UInt32-Nachricht
        message.set__data(count_++);

        RCLCPP_INFO(this->get_logger(), "Publishing: '%i'", message.data); // Loggen der veröffentlichten Nachricht
        publisher_->publish(message); // Veröffentlichen der Nachricht auf 'number'
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>()); // Erstellung eines Objekts der Klasse MinimalPublisher
    rclcpp::shutdown();
    return 0;
}