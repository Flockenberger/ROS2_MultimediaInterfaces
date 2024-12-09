#include "ComputeNode.h"
#include "SerialInterface.h"


mmi::ComputeNode::ComputeNode() : Node("mmi_compute_node")
{
	m_Subscriber =
		this->create_subscription<std_msgs::msg::String>("mmi_serial_compute_topic", 10, std::bind(&ComputeNode::OnCompute, this, std::placeholders::_1));

	RCLCPP_INFO(this->get_logger(), "Spinning up ComputeNode");

}

void mmi::ComputeNode::OnCompute(const std_msgs::msg::String::SharedPtr msg)
{
	std::string result = msg->data;
	std::string prettyResult = result.substr(0, result.length() - 1);

	std::vector<mmi::Float32> values = mmi::SerialProtocol::ParseValueProtocol(prettyResult);
	
	RCLCPP_INFO(this->get_logger(), "Received Value Protocol in ComputeNode with: %i values!", values.size());

	mmi::Float32 Calculated = 0;
	for (mmi::Float32 value : values)
	{
		Calculated += value;
	}

	RCLCPP_INFO(this->get_logger(), "All Values Added Up: %f", Calculated);
}