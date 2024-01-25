#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("my_node")
  {
    // Erstellen Sie ein QoS-Objekt mit BEST_EFFORT-Zuverlässigkeit
    rclcpp::QoS qos(rclcpp::KeepLast(1)); // KeepLast(1) entspricht BEST_EFFORT

    // Subscriber für den Lidar-Sensor mit QoS-Einstellungen
    lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", qos, std::bind(&MyNode::lidarCallback, this, std::placeholders::_1)
    );
  }

private:
  void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Hier können Sie den Lidar-Datenverarbeitungscode einfügen
    // Beispiel: Ausgabe der Anzahl der Lidar-Messungen
    RCLCPP_INFO(get_logger(), "Anzahl der Messungen: %zu", msg->ranges.size());
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyNode>());
  rclcpp::shutdown();
  return 0;
}
