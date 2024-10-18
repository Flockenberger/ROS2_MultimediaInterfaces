#include "SerialInterface.h"

const std::string mmi::SerialProtocol::Initialize = "mmi_p://initialize";
const std::string mmi::SerialProtocol::Ok = "Ok";
const std::string mmi::SerialProtocol::Fail = "Fail";

const mmi::Char mmi::SerialProtocol::MessageEnd = '\n';

const std::string  mmi::SerialProtocol::EnableLedBuiltin = "mmi_p://event/builtin_enable";
const std::string  mmi::SerialProtocol::DisableLedBuiltin = "mmi_p://event/builtin_disable";