#include "MMIProtocol.h"
#include "HardwareSerial.h"
#include "wiring_digital.c"

mmi::MMIProtocol::MMIProtocol()
  : m_Initialized(false) {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void mmi::MMIProtocol::Begin(const mmi::SerialBaud& baud) {
  Serial.begin((mmi::UInt64)baud);
}

void mmi::MMIProtocol::End() {
  digitalWrite(LED_BUILTIN, LOW);
  m_Initialized = false;
  m_Message = "";
  Serial.end();
}

void mmi::MMIProtocol::Write(const String& protocol) {
  Serial.print(protocol);
}

void mmi::MMIProtocol::EstablishConnection()
{
  if(BeginMessage(m_Message))
  {
    if (IsProtocol(m_Message, mmi::SerialProtocol::Initialize)) {
          digitalWrite(LED_BUILTIN, HIGH);
          Write(mmi::SerialProtocol::Ok);
          m_Initialized = true;
      }
    EndMessage(m_Message);
  }
}

void mmi::MMIProtocol::EndMessage(String &message)
{
  message = "";
}

mmi::Bool mmi::MMIProtocol::BeginMessage(String& message) {

  if (Serial.available() > 0) {

    mmi::Char messageChar = (mmi::Char) Serial.read();
    mmi::Bool messageCompleted = false;
    
    if (messageChar == mmi::SerialProtocol::MessageEnd) {
      messageCompleted = true;
      message.trim();
    }
    else
    {
      message += messageChar;
    }
    
    return messageCompleted;
  }
}

mmi::Bool mmi::MMIProtocol::IsProtocol(const String& message, const String& protocol) {
  return message.indexOf(protocol) >= 0;
}

mmi::Bool mmi::MMIProtocol::ConnectionEstablished() {
  return m_Initialized;
}