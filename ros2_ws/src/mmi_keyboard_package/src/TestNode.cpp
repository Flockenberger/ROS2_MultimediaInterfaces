#include "TestNode.h"

MinimalPublisher::MinimalPublisher() : Node("minimal_publisher"), count_(0)
{
	publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
	auto timer_callback =
		[this]() -> void {
		auto message = std_msgs::msg::String();
		message.data = "Hello, world! " + std::to_string(this->count_++);
		RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
		this->publisher_->publish(message);
	};
	timer_ = this->create_wall_timer(500ms, timer_callback);
}


int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MinimalPublisher>());
	rclcpp::shutdown();
	return 0;
}
