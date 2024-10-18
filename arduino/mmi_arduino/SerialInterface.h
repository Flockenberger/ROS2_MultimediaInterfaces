#pragma once
#include "MMIPredefine.h"

MMI_NAMESPACE_BEGIN(mmi)

/// <summary>
/// SerialBaud.
/// The serial baud rates available for arduino
/// </summary>
enum class SerialBaud : mmi::UInt64 {
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
/// The serial protocol which describes the mmi protocol.
/// </summary>
class SerialProtocol
{

public:

    /// <summary>
    /// Initialize.
    /// Used to initialize and establish the connection to the arduino device.
    /// After this is sent, the device should respond with mmi::SerialProtocol::Ok to signal
    /// that this connection is valid.
    /// </summary>
    static const String Initialize;

    /// <summary>
    /// Ok.
    /// Indicates that whatever message was received or written was successful.
    /// </summary>
    static const String Ok;

    /// <summary>
    /// Fail.
    /// Indicates that whatever message was received or written was not successful.
    /// </summary>
    static const String Fail;

    /// <summary>
    /// MessageEnd.
    /// This character indicates that the current message has ended and should be processed.
    /// </summary>
    static const mmi::Char mmi::SerialProtocol::MessageEnd;

    /// <summary>
    /// EnableLedBuiltin.
    /// Enables the builtin LED of the arduino.
    /// </summary>
    static const String EnableLedBuiltin;

    /// <summary>
    /// DisableLedBuiltin.
    /// Disables the builtin LED of the arduino.
    /// </summary>
    static const String DisableLedBuiltin;

};
MMI_NAMESPACE_END(mmi)