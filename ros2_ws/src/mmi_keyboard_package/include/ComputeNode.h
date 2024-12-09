#pragma once

#include "MMIPredefine.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

MMI_NAMESPACE_BEGIN(mmi)

/// <summary>
/// ComputeNode:
/// A very simple node which has one subscriber and expects a mmi::SerialProtocol::Value message on the 
/// topic mmi_serial_compute_topic
/// </summary>
class ComputeNode : public rclcpp::Node
{

private:
	//rclcpp 
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_Subscriber;
private:
	void OnCompute(const std_msgs::msg::String::SharedPtr msg);

public:
	ComputeNode();
};

MMI_NAMESPACE_END(mmi)