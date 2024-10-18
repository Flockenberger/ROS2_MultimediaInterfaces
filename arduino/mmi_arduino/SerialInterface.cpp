#include "SerialInterface.h"

const String mmi::SerialProtocol::Initialize = "mmi_p://initialize";
const String mmi::SerialProtocol::Ok = "Ok";
const String mmi::SerialProtocol::Fail = "Fail";
const mmi::Char mmi::SerialProtocol::MessageEnd = '\n';

const String  mmi::SerialProtocol::EnableLedBuiltin = "mmi_p://event/builtin_enable";
const String  mmi::SerialProtocol::DisableLedBuiltin = "mmi_p://event/builtin_disable";