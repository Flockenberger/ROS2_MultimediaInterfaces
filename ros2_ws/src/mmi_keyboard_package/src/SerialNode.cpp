#include "SerialNode.h"

#ifdef MMI_PLATFORM_WINDOWS
#include <windows.h>
#endif

const std::string mmi::SerialNode::SerialTopic = "mmi_serial_topic";

mmi::SerialNode::SerialNode(const mmi::SerialConnection& connection)
	: Node("mmi_serial_node"), m_Connection(connection)
{

	RCLCPP_INFO(this->get_logger(), "Spinning up SerialNode");

	CreateInitialPortList();
	OpenOrDetectArduinoPort();

	// Subscriber to the topic
	m_Subscriber =
		this->create_subscription<std_msgs::msg::String>(mmi::SerialNode::SerialTopic, 10, std::bind(&SerialNode::SendToSerialCallback, this, std::placeholders::_1));
	// Create a publisher for serial data
	m_Publisher = this->create_publisher<std_msgs::msg::String>("mmi_serial_compute_topic", 10);

	// Create a timer to periodically check for data from the serial port
	m_Timer = this->create_wall_timer(
		std::chrono::milliseconds(100), std::bind(&SerialNode::ReceiveSerialCallback, this));
}

void mmi::SerialNode::CreateInitialPortList()
{
	if (m_Connection.AutoDetectPort)
	{
		RCLCPP_INFO(this->get_logger(), "Scanning Ports...");

#ifdef MMI_PLATFORM_WINDOWS
		for (int i = 1; i <= 256; i++)
		{
			std::string port_name = "COM" + std::to_string(i);
			HANDLE hSerial = CreateFile(port_name.c_str(), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
			if (hSerial != INVALID_HANDLE_VALUE)
			{
				m_PortList.push_back(port_name);
				CloseHandle(hSerial);
			}
		}
#else
		//Note: Not tested as I don't have a linux system at hand!
		// For Linux, use "/dev/ttyACMx" or "/dev/ttyUSBx"
		for (int i = 0; i < 10; i++)
		{
			std::string port_name = "/dev/ttyACM" + std::to_string(i);
			m_PortList.push_back(port_name);
		}
#endif
	}
}

mmi::Bool mmi::SerialNode::OpenPort(const std::string& port)
{
	try
	{
		if (m_SerialPort.isOpen())
		{
			m_SerialPort.close();
		}

		m_SerialPort.setPort(port);
		m_SerialPort.setBaudrate((mmi::UInt64)m_Connection.Baud);
		m_SerialPort.setTimeout(serial::Timeout::simpleTimeout(1000));
		m_SerialPort.open();
	}
	catch (const serial::IOException& e)
	{
		RCLCPP_ERROR(this->get_logger(), "Unable to open port");
		return false;
	}

	if (m_SerialPort.isOpen())
	{
		return true;
	}

	else
	{
		return false;
	}
}

mmi::Bool mmi::SerialNode::ProtocolInitializeConnection()
{
	mmi::UInt64 data = m_SerialPort.write(mmi::SerialProtocol::Initialize + mmi::SerialProtocol::MessageEnd);
	std::string response = m_SerialPort.readline();

	//	RCLCPP_INFO(this->get_logger(), "Protocol Response: %s", response);

	if (!response.empty())
	{
		if (response == (mmi::SerialProtocol::Ok + mmi::SerialProtocol::MessageEnd))
		{
			return true;
		}
	}
	return false;
}

void mmi::SerialNode::OpenOrDetectArduinoPort()
{
	if (m_Connection.AutoDetectPort)
	{
		mmi::Bool foundArduino = false;
		if (m_PortList.size() > 0)
		{
			for (auto& port : m_PortList)
			{
				if (OpenPort(port))
				{
					RCLCPP_INFO(this->get_logger(), "Testing port for mmi Ok: %s", port);
					if (ProtocolInitializeConnection())
					{
						RCLCPP_INFO(this->get_logger(), "Found Arduino at port: %s", port);
						foundArduino = true;
						break;
					}

					//					else
					//					{
					//						RCLCPP_INFO(this->get_logger(), "Port %s is not an mmi Arduino: ", port);
					//					}
				}
			}
		}

		if (!foundArduino)
		{
			RCLCPP_INFO(this->get_logger(), "Could not auto-detect arduino port... falling back to given port.");
			if (!OpenPort(m_Connection.Port))
			{
				if (m_SerialPort.isOpen())
				{
					m_SerialPort.close();
				}
			}
		}
	}
	else
	{
		if (OpenPort(m_Connection.Port))
		{
			if (ProtocolInitializeConnection())
			{
				RCLCPP_INFO(this->get_logger(), "Established Arduino connection!");
			}
		}
	}
}

std::string& mmi::SerialNode::FormatProtocolMessage(std::string& msg)
{
	if (msg.empty() || msg.back() != mmi::SerialProtocol::MessageEnd)
	{
		msg += mmi::SerialProtocol::MessageEnd;
	}
	return msg;
}

void mmi::SerialNode::SendToSerialCallback(const std_msgs::msg::String::SharedPtr msg)
{

	std::string data = msg->data;

	//ensure we add a \n to our message as that is required by protocol
	//FormatProtocolMessage(data);
	
	if (m_SerialPort.isOpen())
	{

		//TODO: Change to something better and more useful
		//Maybe do something similar as we did on the arduino and
		//create a proper registry system etc.
		if (data == "0")
		{
			data = mmi::SerialProtocol::DisableLedBuiltin; RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Sending to serial:" << mmi::SerialProtocol::DisableLedBuiltin);
			m_SerialPort.write(FormatProtocolMessage(data));
		}
		else if (data == "1")
		{
			data = mmi::SerialProtocol::EnableLedBuiltin; RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Sending to serial:" << mmi::SerialProtocol::EnableLedBuiltin);
			m_SerialPort.write(FormatProtocolMessage(data));
		}

		//Enable Joystick
		else if (data == "2")
		{
			data = mmi::SerialProtocol::EnableJoystick; RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Sending to serial:" << mmi::SerialProtocol::EnableJoystick);
			m_SerialPort.write(FormatProtocolMessage(data));
		}
		//Dsiable Joystick
		else if (data == "3")
		{
			data = mmi::SerialProtocol::DisableJoystick; RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Sending to serial:" << mmi::SerialProtocol::DisableJoystick);
			m_SerialPort.write(FormatProtocolMessage(data));
		}

		//Enable HC-SR04
		else if (data == "4")
		{
			data = mmi::SerialProtocol::EnableSR04; RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Sending to serial:" << mmi::SerialProtocol::EnableSR04);
			m_SerialPort.write(FormatProtocolMessage(data));
		}

		//Disable HC-SR04
		else if (data == "5")
		{
			data = mmi::SerialProtocol::DisableSR04; RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Sending to serial: " << mmi::SerialProtocol::DisableSR04);
			m_SerialPort.write(FormatProtocolMessage(data));
		}
		
	}
	else
	{
		RCLCPP_ERROR(this->get_logger(), "Serial port is not open. Cannot send data.");
	}

}

void mmi::SerialNode::ReceiveSerialCallback()
{
	if (m_SerialPort.available())
	{
		// Read all available data from the serial port
		std::string result = m_SerialPort.readline();
		std::string prettyResult = result.substr(0, result.length() - 1);

		RCLCPP_INFO(this->get_logger(), "Received: %s from Arduino!", prettyResult.c_str());

		if (result.find(mmi::SerialProtocol::Value) != std::string::npos)
		{
			RCLCPP_INFO(this->get_logger(), "Forwarding Protocol!");
			
			auto message = std_msgs::msg::String();
			message.data = result;
			m_Publisher->publish(message);
		}
	}
}