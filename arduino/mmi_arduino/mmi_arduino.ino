#include "MMIProtocol.h"

//The MMI Protocol instance
mmi::MMIProtocol Protocol;

//The message with which we communicate
String message = "";

void setup() {

  //Begin our protocol
  //This will open up a serial connection at the given baud rate
  Protocol.Begin(mmi::SerialBaud::BAUD_115200);
}

void loop() {

  //check if we established a valid connection through protocol
  if (Protocol.ConnectionEstablished()) {

    //We wait until we have a message
    if (Protocol.BeginMessage(message)) {

      //now we can do anything with our message
      if (Protocol.IsProtocol(message, mmi::SerialProtocol::EnableLedBuiltin)) {
        digitalWrite(LED_BUILTIN, HIGH);
        Protocol.Write(mmi::SerialProtocol::Ok);

      } else if (Protocol.IsProtocol(message, mmi::SerialProtocol::DisableLedBuiltin)) {
        digitalWrite(LED_BUILTIN, LOW);
        Protocol.Write(mmi::SerialProtocol::Ok);
      }

      //once we used this message end it
      //syntax is similar to how ImGui
      Protocol.EndMessage(message);
    }

  }
  //If we don't have an established connection -> wait until we do
  else {
    Protocol.EstablishConnection();
  }
}
