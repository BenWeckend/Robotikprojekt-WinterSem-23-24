#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"

using namespace std::chrono_literals;

class MinPub : public rclcpp::Node {
public:
    MinPub() : Node("number_publisher"), count_(0) {
        publisher_ = this->create_publisher<std_msgs::msg::UInt32>("number", 10);

        // Erstellung von timer, der die timer_callback-Funktion im Intervall von 1s aufruft
        timer_ = this->create_wall_timer(1000s, std::bind(&MinPub::timer_callback, this));
    }
private:
    // Callback-Funktion
    void timer_callback() {
        auto message = std_msgs::msg::UInt32();
        message.set__data(count_++);

        RCLCPP_INFO(this->get_logger(), "Publishing: '%i'", message.data); // Loggen der veröffentlichten Nachricht
        publisher_->publish(message);  // Veröffentlichen der Nachricht
    }

   
    rclcpp::TimerBase::SharedPtr timer_;  // Member-Variablen der Klasse
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char *argv[]) {
    // Initialisierung von ROS 2
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinPub>()); // Erstellung eines Objekts der Klasse MinPub

    rclcpp::shutdown();

    return 0;
}
