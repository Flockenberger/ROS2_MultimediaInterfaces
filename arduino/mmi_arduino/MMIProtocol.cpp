#include "MMIProtocol.h"
#include "HardwareSerial.h"
#include "wiring_digital.c"

mmi::MMIProtocol::MMIProtocol()
  : m_Initialized(false) {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

mmi::Bool mmi::MMIProtocol::RegisterMessageHandler(MMI_CONST mmi::String& messageType, mmi::ProtocolCallback clb)
{
  for (mmi::UInt8 i = 0; i < MMI_PROTOCOL_NUM_CALLBACKS; ++i) 
  {
    if (m_Callbacks[i].Message.length() == 0 || m_Callbacks[i].Message == messageType) 
    {
      m_Callbacks[i].Message = messageType;
      m_Callbacks[i].Callback = clb;
      return true;
    }
  }
  // No space left to register callback
  return false;
}

mmi::Bool mmi::MMIProtocol::RegisterMessagePublisher(mmi::ProtocolPublisher publisher)
{
  for (mmi::UInt8 i = 0; i < MMI_PROTOCOL_NUM_PUBLISHERS; ++i) 
  {
    if (m_Publishers[i].Publish == publisher) 
    {
      return false;
    }
  }

  for (mmi::UInt8 i = 0; i < MMI_PROTOCOL_NUM_PUBLISHERS; ++i) 
  {
    if (m_Publishers[i].Publish == nullptr) 
    {
      m_Publishers[i].Publish = publisher;
      return true;
    }
  }
  // No space left to register callback
  return false;
}

mmi::Bool mmi::MMIProtocol::UnregisterMessagePublisher(mmi::ProtocolPublisher publisher)
{
  for (mmi::UInt8 i = 0; i < MMI_PROTOCOL_NUM_PUBLISHERS; ++i) 
  {
    if (m_Publishers[i].Publish == publisher) 
    {
      m_Publishers[i].Publish = nullptr;
      return true;
    }
  }
  return false;
}

void mmi::MMIProtocol::SetPublishInterval(mmi::UInt64 interval)
{
  if(interval > 0)
  {
    m_PublishInterval = interval;
  }
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
  Serial.print(protocol + mmi::SerialProtocol::MessageEnd);
}

void mmi::MMIProtocol::EstablishConnection() {
  if (BeginMessage(m_Message)) {
    if (IsProtocol(m_Message, mmi::SerialProtocol::Initialize)) {
      digitalWrite(LED_BUILTIN, HIGH);
      Write(mmi::SerialProtocol::Ok);
      m_Initialized = true;
    }
    EndMessage(m_Message);
  }
}

void mmi::MMIProtocol::EndMessage(String& message) {
  message = "";
}

mmi::Bool mmi::MMIProtocol::BeginMessage(String& message) {

  if (Serial.available() > 0) {

    mmi::Char messageChar = (mmi::Char)Serial.read();
    mmi::Bool messageCompleted = false;

    if (messageChar == mmi::SerialProtocol::MessageEnd) {
      messageCompleted = true;
      message.trim();
    } else {
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

mmi::Bool mmi::MMIProtocol::Run() {

  //check if we established a valid connection through protocol
  if (ConnectionEstablished()) 
  {
    mmi::Bool clbValue = false;
    //We wait until we have a message
    if (BeginMessage(m_ProtocolMessage))
    {
      
      for (mmi::UInt8 i = 0; i < MMI_PROTOCOL_NUM_CALLBACKS; ++i) 
      {
          if (IsProtocol(m_ProtocolMessage, m_Callbacks[i].Message)) 
          {
              // If callback exists, call it
              if (m_Callbacks[i].Callback != nullptr) 
              {
                clbValue =  m_Callbacks[i].Callback(m_ProtocolMessage, *this);
                break;
              }
          }
      }

      //once we used this message end it
      //syntax is similar to how ImGui works (though wayyy less strict :) )
      EndMessage(m_ProtocolMessage);
      
    }

    mmi::UInt64 currentMillis = millis();
    if (currentMillis - m_LastPublishTime >= m_PublishInterval) 
    {
      for (mmi::UInt8 i = 0; i < MMI_PROTOCOL_NUM_PUBLISHERS; ++i) 
      {
        if (m_Publishers[i].Publish != nullptr) 
        {
          m_Publishers[i].Publish(*this);
        }
      }
      m_LastPublishTime = currentMillis; // Update the last publish time
    }
    
    return clbValue;
  }
  //If we don't have an established connection -> wait until we do
  else 
  {
    EstablishConnection();
  }

  return false;
}
