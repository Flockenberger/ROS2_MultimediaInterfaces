#include "MMIProtocol.h"

//The MMI Protocol instance
//Note: Change the MMI_PROTOCOL_NUM_CALLBACKS and MMI_PROTOCOL_NUM_PUBLISHERS define to change the maximum number
//of available callbacks/publishers if needed!
mmi::MMIProtocol Protocol;

//Example Handling method for our Protocol
mmi::Bool HandleLedBuiltinEnable(MMI_CONST mmi::String& message, mmi::MMIProtocol& protocol)
{
  digitalWrite(LED_BUILTIN, HIGH);
  protocol.Write(mmi::SerialProtocol::Ok);
  return true;
}


///=============================================
/// Joystick
///=============================================

//Publishing method
mmi::Bool PublishJoystick(mmi::MMIProtocol& protocol)
{
  mmi::Int32 x = analogRead(A0);
  mmi::Int32 y = analogRead(A1);
  mmi::Int32 z = analogRead(A2);
  protocol.Write(mmi::SerialProtocol::CreateValueMessage(x, y, z));
  //Disable publisher on Joystick SW 
  if(z <= 100)
  {
    protocol.UnregisterMessagePublisher(PublishJoystick);
  }

  return true;
}

//Handle EnableJoystick
mmi::Bool HandleEnableJoystick(MMI_CONST mmi::String& message, mmi::MMIProtocol& protocol)
{
  return protocol.RegisterMessagePublisher(PublishJoystick);

}

//Handle DisableJoystick
mmi::Bool HandleDisableJoystick(MMI_CONST mmi::String& message, mmi::MMIProtocol& protocol)
{
  return protocol.UnregisterMessagePublisher(PublishJoystick);
}



///=============================================
/// HC-SR04
///=============================================

//With help from:
//https://projecthub.arduino.cc/Isaac100/getting-started-with-the-hc-sr04-ultrasonic-sensor-7cabe1
MMI_CONST mmi::UInt32 trigPin = 9;
MMI_CONST mmi::UInt32 echoPin = 10;
mmi::Float32 duration;
mmi::Float32 distance;

//Publish HC-SR04
mmi::Bool PublishHCSR04(mmi::MMIProtocol& protocol)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;
  protocol.Write(mmi::SerialProtocol::CreateValueMessage(distance, duration));
  return true;
}

//Handle EnableSR04
mmi::Bool HandleEnableSR04(MMI_CONST mmi::String& message, mmi::MMIProtocol& protocol)
{
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  return protocol.RegisterMessagePublisher(PublishHCSR04);
}

//Handle isableSR04
mmi::Bool HandleDisableSR04(MMI_CONST mmi::String& message, mmi::MMIProtocol& protocol)
{
 return protocol.UnregisterMessagePublisher(PublishHCSR04);
}

void setup() {
  
  //Two examples of how to use the message handlers, either inlined lambda or through a method
  Protocol.RegisterMessageHandler(mmi::SerialProtocol::EnableLedBuiltin, HandleLedBuiltinEnable);
  Protocol.RegisterMessageHandler(mmi::SerialProtocol::DisableLedBuiltin, [](MMI_CONST String& message, mmi::MMIProtocol& protocol) 
  {
    digitalWrite(LED_BUILTIN, LOW);
    protocol.Write(mmi::SerialProtocol::Ok);
    return true;
  });

  //Actual project registry
  Protocol.RegisterMessageHandler(mmi::SerialProtocol::EnableJoystick, HandleEnableJoystick);
  Protocol.RegisterMessageHandler(mmi::SerialProtocol::DisableJoystick, HandleDisableJoystick);
  Protocol.RegisterMessageHandler(mmi::SerialProtocol::EnableSR04, HandleEnableSR04);
  Protocol.RegisterMessageHandler(mmi::SerialProtocol::DisableSR04, HandleDisableSR04);


  //Begin our protocol
  //This will open up a serial connection at the given baud rate
  Protocol.Begin(mmi::SerialBaud::BAUD_115200);

 
}

void loop() {

  if(Protocol.Run())
  {
    //Received message was handled successfully
  }
  else
  {
    //Either no connection established, or message could not be handled
  }
}
