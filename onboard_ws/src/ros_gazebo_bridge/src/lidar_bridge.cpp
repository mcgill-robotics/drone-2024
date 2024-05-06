#include <functional>

#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <gz/msgs.hh>
#include <gz/transport.hh>


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher(gz::transport::Node& node)
    : Node("minimal_publisher")
    {
      MinimalPublisher::publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("laser_scan", 10);
      if (!node.Subscribe(this->gz_topic, std::bind(&MinimalPublisher::callback, this, std::placeholders::_1))) {
        std::cerr << "Error Subscribing to topic [" << this->gz_topic << "]" << std::endl;
        return;
      }
      
    }

  private:
    void callback(const gz::msgs::LaserScan& msg) {
      std::cout << msg.DebugString() << std::endl;
      auto ros_msg = sensor_msgs::msg::LaserScan();
      
    }
    std::string gz_topic = "/lidar"; 
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  gz::transport::Node node;
  rclcpp::spin(std::make_shared<MinimalPublisher>(node));
  gz::transport::waitForShutdown();
  rclcpp::shutdown();
  return 0;
}
