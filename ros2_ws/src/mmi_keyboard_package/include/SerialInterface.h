#pragma once
#include "MMIPredefine.h"

MMI_NAMESPACE_BEGIN(mmi)

/// <summary>
/// SerialBaud.
/// The serial baud rates available for arduino
/// </summary>
enum class SerialBaud : mmi::UInt64
{
    BAUD_300 = 300,
    BAUD_600 = 600,
    BAUD_1200 = 1200,
    BAUD_2400 = 2400,
    BAUD_4800 = 4800,
    BAUD_9600 = 9600,
    BAUD_14400 = 14400,
    BAUD_19200 = 19200,
    BAUD_28800 = 28800,
    BAUD_31250 = 31250,
    BAUD_38400 = 38400,
    BAUD_57600 = 57600,
    BAUD_115200 = 115200
};

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

class SerialProtocol
{

public:
    // Public static constant strings
    static const std::string Initialize;
    static const std::string Ok;
    static const std::string Fail;

    static const std::string SerialTopic;
};
MMI_NAMESPACE_END(mmi)