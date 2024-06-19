#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include <rclcpp/subscription.hpp>

class GuiNode : public rclcpp::Node
{
	public:
		GuiNode();
		void publish_message(std::string message);

	private:
		// Publications
		rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr start_publisher;
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_lap_publisher;
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr survey_zone_publisher;
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_boundary_publisher;
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_lap_tolerance_publisher;
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_targets_publisher;
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_time_publisher;

		// Subscriptions
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_status_sub;
		
};
