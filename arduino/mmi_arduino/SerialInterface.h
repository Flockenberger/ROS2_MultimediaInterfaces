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


MMI_NAMESPACE_BEGIN(SerialProtocol)

/// The serial protocol which describes the mmi protocol.

/// <summary>
/// Initialize.
/// Used to initialize and establish the connection to the arduino device.
/// After this is sent, the device should respond with mmi::SerialProtocol::Ok to signal
/// that this connection is valid.
/// </summary>
MMI_STATIC MMI_CONST mmi::String Initialize = "mmi_p://initialize";

/// <summary>
/// Ok.
/// Indicates that whatever message was received or written was successful.
/// </summary>
MMI_STATIC MMI_CONST  mmi::String Ok = "Ok";

/// <summary>
/// Fail.
/// Indicates that whatever message was received or written was not successful.
/// </summary>
MMI_STATIC MMI_CONST  mmi::String Fail = "Fail";

/// <summary>
/// MessageEnd.
/// This character indicates that the current message has ended and should be processed.
/// </summary>
MMI_STATIC MMI_CONST mmi::Char MessageEnd = '\n';

//Test protocols for functionality

/// <summary>
/// EnableLedBuiltin.
/// Enables the builtin LED of the arduino.
/// </summary>
MMI_STATIC MMI_CONST  mmi::String EnableLedBuiltin = "mmi_p://event/builtin_enable";

/// <summary>
/// DisableLedBuiltin.
/// Disables the builtin LED of the arduino.
/// </summary>
MMI_STATIC MMI_CONST  mmi::String DisableLedBuiltin = "mmi_p://event/builtin_disable";

/// <summary>
/// EnableJoystick.
/// Enables the Joystick
/// </summary>
MMI_STATIC MMI_CONST  mmi::String EnableJoystick = "mmi_p://event/ena_joystick";

/// <summary>
/// DisableJoystick.
/// Disables the Joystick
/// </summary>
MMI_STATIC MMI_CONST  mmi::String DisableJoystick = "mmi_p://event/dis_joystick";

/// <summary>
/// EnableSR04.
/// Enables the HC-SR04 sensor
/// </summary>
MMI_STATIC MMI_CONST  mmi::String EnableSR04 = "mmi_p://event/ena_hcsr04";

/// <summary>
/// DisableSR04.
/// Disables the HC-SR04 sensor
/// </summary>
MMI_STATIC MMI_CONST  mmi::String DisableSR04 = "mmi_p://event/dis_hcsr04";


/// <summary>
/// Values
/// Used to send values.
/// The protocol expects the following:
/// mmi_p://event/value/X/[V/]
/// Where X denotes the amount of values and V a single value
/// Example:
/// mmi_p://event/value/2/123.0/123.2
/// </summary>
MMI_STATIC MMI_CONST  mmi::String Value = "mmi_p://event/value";

#if defined(MMI_PLATFORM_WINDOWS)
MMI_STATIC std::vector<mmi::Float32> ParseValueProtocol(const std::string& protocolString)
{
    std::vector<mmi::Float32> values;
    std::string baseProto = Value + "/";
    // Check if the protocol string starts with the base protocol
    if (protocolString.find(baseProto) != 0)
    {
        return values;
    }

    std::string remaining = protocolString.substr(baseProto.size());
    std::replace(remaining.begin(), remaining.end(), ',', '.');

    std::vector<std::string> parts;
    std::istringstream stream(remaining);
    std::string part;

    while (std::getline(stream, part, '/')) 
    {
        parts.push_back(part);
    }

    if (parts.size() < 1) 
    {
        return values;
    }

    int numValues = std::stoi(parts[0]);

    if (parts.size() - 1 != static_cast<size_t>(numValues)) 
    {
        return values;
    }

    
    for (size_t i = 1; i < parts.size(); ++i) 
    {
        values.push_back(std::stof(parts[i]));
    }

    return values;
}
#elif defined(MMI_PLATFORM_ARDUINO)
MMI_STATIC mmi::Int32 ParseValueProtocol(MMI_CONST mmi::Char* protocolString, mmi::Float32* values, mmi::Int32 maxValues) 
{
    MMI_CONST mmi::Char* baseProto = mmi::String(Value + "/").c_str();

    if (strncmp(protocolString, baseProto, strlen(baseProto)) != 0)
     {
        return -1; 
    }

    MMI_CONST mmi::Char* remaining = protocolString + strlen(baseProto);
    mmi::Char* endPtr = (mmi::Char*)remaining;
    mmi::Int32 numValues = atoi(endPtr);

    // Move past the number
    while (*endPtr >= '0' && *endPtr <= '9') {
        endPtr++;
    }

    // Ensure there's a slash after the number
    if (*endPtr != '/') {
        return -3; // Missing separator
    }
    endPtr++; // Skip the slash

    // Parse each value
    for (mmi::Int32 i = 0; i < numValues; ++i) 
    {
        // Replace ',' with '.' for decimal parsing
        mmi::Char valueStr[16]; // Buffer for single value
        mmi::Int32 valueIndex = 0;

        while (*endPtr != '/' && *endPtr != '\0') 
        {
            // Replace ',' with '.'
            if (*endPtr == ',') 
            {
                valueStr[valueIndex++] = '.';
            } else 
            {
                valueStr[valueIndex++] = *endPtr;
            }
            endPtr++;
        }

        valueStr[valueIndex] = '\0'; // Null-terminate the string

        // Convert to float using atof
        values[i] = atof(valueStr);

        // Ensure there's a slash after each value except the last
        if (i < numValues - 1 && *endPtr != '/') 
        {
            return -4; // Missing separator for a value
        }
        endPtr++; // Skip the slash
    }

    return numValues; // Return the number of parsed values
}
#endif

MMI_STATIC mmi::String CreateValueMessage() 
{
  return Value + "/0";
}

template<typename T>
MMI_STATIC mmi::String CreateValueMessage(const T& first) 
{
  return Value + "/1/" + mmi::String(first);
}

template<typename T, typename... Args>
MMI_STATIC mmi::String CreateValueMessage(const T& first, const Args&... rest) 
{
  return Value + "/" + mmi::String(sizeof...(rest) + 1) + "/" + mmi::String(first) + CreateValueMessageHelper(rest...);
}

template<typename T>
MMI_STATIC mmi::String CreateValueMessageHelper(const T& value)
{
  return "/" + mmi::String(value);
}

template<typename T, typename... Args>
MMI_STATIC mmi::String CreateValueMessageHelper(const T& first, const Args&... rest)
{
  return "/" + mmi::String(first) + CreateValueMessageHelper(rest...);
}

MMI_NAMESPACE_END(SerialProtocol)
MMI_NAMESPACE_END(mmi)