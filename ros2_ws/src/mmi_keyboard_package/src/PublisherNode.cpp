#include "PublisherNode.h"
#include "SerialNode.h"

#include <stdio.h>
#include <conio.h>

mmi::PublisherNode::PublisherNode() : Node("mmi_publisher"), m_Count(0)
{
	RCLCPP_INFO(this->get_logger(), "Spinning up PublisherNode");

	m_Publisher = this->create_publisher<std_msgs::msg::String>(mmi::SerialNode::SerialTopic, 1);
	auto timer_callback =
		[this]() -> void
	{
		mmi::Char keyboardInput = getch();
		
		auto message = std_msgs::msg::String();
		message.data = keyboardInput;
		
		RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
		
		this->m_Publisher->publish(message);
		m_Count++;
	};

	m_Timer = this->create_wall_timer(100ms, timer_callback);
}


