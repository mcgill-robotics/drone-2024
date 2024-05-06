#include <functional>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;

// This node subscribes to gazebo topics relevant to lidar readings and lidar
// position it then publshes the lidar readings under a ros topic and sets a
// frame for it ("map" by default)
class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher(gz::transport::Node &node)
      : Node("minimal_publisher"), tf_broadcaster_(this) {
    this->declare_parameter("gz_lidar_topic", "/lidar");
    this->declare_parameter("gz_pose_topic", "/world/default/pose/info");
    this->declare_parameter("gz_lidar_link_name", "lidar_link");

    this->gz_lidar_topic = this->get_parameter("gz_lidar_topic").as_string();
    this->gz_pose_topic = this->get_parameter("gz_pose_topic").as_string();
    this->gz_lidar_link_name =
        this->get_parameter("gz_lidar_link_name").as_string();

    MinimalPublisher::publisher_ =
        this->create_publisher<sensor_msgs::msg::LaserScan>("laser_scan", 10);
    std::function<void(const gz::msgs::LaserScan &)> lidar_func_cb = std::bind(
        &MinimalPublisher::lidar_callback, this, std::placeholders::_1);

    // lidar publisher
    if (!node.Subscribe(this->gz_lidar_topic, lidar_func_cb)) {
      RCLCPP_ERROR(this->get_logger(), "Error Subscribing to topic [%s]",
                   this->gz_lidar_topic.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(),
                "Subscribing to lidar topic [%s] successfull!",
                this->gz_lidar_topic.c_str());

    // Pose (frame) publisher
    std::function<void(const gz::msgs::Pose_V &)> pose_func_cb = std::bind(
        &MinimalPublisher::pose_callback, this, std::placeholders::_1);

    if (!node.Subscribe(this->gz_pose_topic, pose_func_cb)) {
      RCLCPP_ERROR(this->get_logger(), "Error Subscribing to topic [%s]",
                   this->gz_pose_topic.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(),
                "Subscribing to pose topic [%s] successfull!",
                this->gz_pose_topic.c_str());
  }

private:
  void lidar_callback(const gz::msgs::LaserScan &msg) {
    // std::cout << msg.DebugString() << std::endl;
    auto ros_msg = sensor_msgs::msg::LaserScan();
    // HEADERS
    ros_msg.header.frame_id = "map";
    ros_msg.header.stamp.sec = msg.header().stamp().sec();
    ros_msg.header.stamp.nanosec = msg.header().stamp().nsec();
    // THE MEAT
    ros_msg.angle_min = msg.angle_min();
    ros_msg.angle_max = msg.angle_max();
    ros_msg.angle_increment = msg.angle_step();
    ros_msg.time_increment = 0;
    ros_msg.scan_time = 0;
    ros_msg.range_min = msg.range_min();
    ros_msg.range_max = msg.range_max();
    std::vector<float> ranges;
    for (int i = 0; i < msg.ranges_size(); i++) {
      ranges.push_back(msg.ranges(i));
    }
    ros_msg.set__ranges(ranges);
    std::vector<float> intensities;
    for (int i = 0; i < msg.intensities_size(); i++) {
      intensities.push_back(msg.intensities(i));
    }
    ros_msg.set__intensities(intensities);
    // PUBLISH
    this->publisher_->publish(ros_msg);
  }

  void pose_callback(const gz::msgs::Pose_V &msg) {
    // std::cout << msg.DebugString() << std::endl;
    gz::msgs::Pose gz_pose;
    int i = 0;
    for (i = 0; i < msg.pose_size(); i++) {
      gz_pose = msg.pose(i);
      if (gz_pose.name() == this->gz_lidar_link_name)
        break;
    }
    if (i >= msg.pose_size()) {
      std::cerr << "Couldn't find pose of object with name ["
                << this->gz_lidar_link_name << "]" << std::endl;
      return;
    }
    geometry_msgs::msg::TransformStamped tf;
    rclcpp::Time time = get_clock()->now();
    // HEADER and FRAME NAME
    tf.header.stamp = time;
    tf.header.frame_id = "/map";
    tf.child_frame_id = "/base_link";
    // POSITION
    tf.transform.translation.x = gz_pose.position().x();
    tf.transform.translation.y = gz_pose.position().y();
    tf.transform.translation.z = gz_pose.position().z();
    // ROTATION
    tf.transform.rotation.x = gz_pose.orientation().x();
    tf.transform.rotation.y = gz_pose.orientation().y();
    tf.transform.rotation.z = gz_pose.orientation().z();
    tf.transform.rotation.w = gz_pose.orientation().w();
    // PUBLISH
    this->tf_broadcaster_.sendTransform(tf);
  }

  tf2_ros::TransformBroadcaster tf_broadcaster_;
  std::string gz_lidar_topic = "/lidar";
  std::string gz_pose_topic = "/world/default/pose/info";
  std::string gz_lidar_link_name = "lidar_link";
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  gz::transport::Node node;
  rclcpp::spin(std::make_shared<MinimalPublisher>(node));
  gz::transport::waitForShutdown();
  rclcpp::shutdown();
  return 0;
}
