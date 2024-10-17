#include "SerialNode.h"

#ifdef MMI_PLATFORM_WINDOWS
#include <windows.h>
#endif


mmi::SerialNode::SerialNode(const mmi::SerialConnection& connection)
	: Node("mmi_serial_node"), m_Connection(connection)
{

	RCLCPP_INFO(this->get_logger(), "Spinning up SerialNode");

	CreateInitialPortList();
	OpenOrDetectArduinoPort();

	// Subscriber to the topic
	m_Subscriber =
		this->create_subscription<std_msgs::msg::String>(mmi::SerialProtocol::SerialTopic, 10, std::bind(&SerialNode::SendToSerialCallback, this, std::placeholders::_1));

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
				RCLCPP_INFO(this->get_logger(), "Found port: %s", port_name);

				m_PortList.push_back(port_name);
				CloseHandle(hSerial);
			}
		}
#else
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
	RCLCPP_INFO(this->get_logger(), "Trying to establish initial connection...");
	mmi::UInt64 data = m_SerialPort.write(mmi::SerialProtocol::Initialize);
	std::string response = m_SerialPort.readline();
	
	if (!response.empty())
	{
		if (response == mmi::SerialProtocol::Ok)
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

					else
					{
						RCLCPP_INFO(this->get_logger(), "Port %s is not an mmi Arduino: ", port);
					}
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

void mmi::SerialNode::SendToSerialCallback(const std_msgs::msg::String::SharedPtr msg)
{

	std::string data = msg->data;
	RCLCPP_INFO(this->get_logger(), "Sending to serial: %s", data.c_str());

	if (m_SerialPort.isOpen())
	{
		m_SerialPort.write(data);
	}
	else
	{
		RCLCPP_ERROR(this->get_logger(), "Serial port is not open. Cannot send data.");
	}

}