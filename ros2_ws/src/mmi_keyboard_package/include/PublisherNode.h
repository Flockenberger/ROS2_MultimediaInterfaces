#pragma once
#include "MMIPredefine.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
using namespace std::chrono_literals;


MMI_NAMESPACE_BEGIN(mmi)

class PublisherNode : public rclcpp::Node
{
private:
	rclcpp::TimerBase::SharedPtr m_Timer;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_Publisher;
	size_t m_Count;

public:
	PublisherNode();
};

MMI_NAMESPACE_END(mmi)