#pragma once
#include "SerialInterface.h"

#include "serial/serial.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

MMI_NAMESPACE_BEGIN(mmi)

/// <summary>
/// SerialNode.
/// The serial node is responsible to send and receive
/// data from a given serial port at a given baud
/// </summary>
class SerialNode : public rclcpp::Node 
{
private:
        
    void SendToSerialCallback(const std_msgs::msg::String::SharedPtr msg);
    void CreateInitialPortList();
    void OpenOrDetectArduinoPort();

    mmi::Bool ProtocolInitializeConnection();
    mmi::Bool OpenPort(const std::string& port);

    serial::Serial m_SerialPort;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_Subscriber;

    mmi::SerialConnection m_Connection;
    std::vector<std::string> m_PortList;

public:
    SerialNode(const mmi::SerialConnection& connection);
};

MMI_NAMESPACE_END(mmi)