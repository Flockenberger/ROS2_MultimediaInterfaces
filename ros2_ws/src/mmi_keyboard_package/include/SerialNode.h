#pragma once
#include "SerialInterface.h"

#include "serial/serial.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

MMI_NAMESPACE_BEGIN(mmi)

/// <summary>
/// SerialConnection.
/// This struct contains the Port and Baud for the serial connection.
/// 
/// </summary>
struct SerialConnection
{
	/// <summary>
	/// If this is enabled the serial node
	/// will try to detect this project's specific arduino
	/// by sending `mmi` to the arduino
	/// </summary>
	mmi::Bool AutoDetectPort;

	/// <summary>
	/// The port to open the connection on
	/// </summary>
	std::string Port;

	/// <summary>
	/// The baud rate
	/// </summary>
	mmi::SerialBaud Baud;
};


/// <summary>
/// SerialNode.
/// The serial node is responsible to send and receive
/// data from a given serial port at a given baud
/// </summary>
class SerialNode : public rclcpp::Node
{

private:
	/// <summary>
	/// The serial port for our current connection
	/// </summary>
	serial::Serial m_SerialPort;
	mmi::SerialConnection m_Connection;
	std::vector<std::string> m_PortList;

	//rclcpp 

	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_Subscriber;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_Publisher;
	rclcpp::TimerBase::SharedPtr m_Timer;

private:

	///Callback functions

	/// <summary>
	/// Sends the received message to the serial connection.
	/// If the connection is not available it does nothing.
	/// </summary>
	/// <param name="msg">The received message</param>
	void SendToSerialCallback(const std_msgs::msg::String::SharedPtr msg);

	/// <summary>
	/// Processes any messages received from the openened serial connection
	/// </summary>
	void ReceiveSerialCallback();

	/// <summary>
	/// Creates the initial port list to check for an arduino running the mmi protocol
	/// </summary>
	void CreateInitialPortList();

	/// <summary>
	/// Checks each port from the port list created by CreateInitiailPortList()
	/// if the attached device is an arduino supporting the mmi protocol.
	/// Note: It does not check for vendor id or of the sorts!
	/// </summary>
	void OpenOrDetectArduinoPort();

	/// <summary>
	/// Tests the connection for the mmi protocol and valid response
	/// </summary>
	/// <returns>True if the device is a valid arduino supporting the mmi protocol, otherwise false</returns>
	mmi::Bool ProtocolInitializeConnection();

	/// <summary>
	/// Tries to open the port given by the parameter 'port'.
	/// </summary>
	/// <param name="port">The port's string representation to try to open</param>
	/// <returns>True if the port was openened successfully, otherwise false</returns>
	mmi::Bool OpenPort(const std::string& port);

	/// <summary>
	/// Formats the given message by appending mmi::SerialProtocol::MessageEnd if not already present
	/// </summary>
	/// <param name="msg">The message to format</param>
	/// <returns>The same formatted message (reference!)</returns>
	inline std::string& FormatProtocolMessage(std::string& msg);

public:
	SerialNode(const mmi::SerialConnection& connection);

	/// <summary>
	/// The ros topic used to by this node to receive data and forward it to the opened connection
	/// </summary>
	static const std::string SerialTopic;
};




MMI_NAMESPACE_END(mmi)