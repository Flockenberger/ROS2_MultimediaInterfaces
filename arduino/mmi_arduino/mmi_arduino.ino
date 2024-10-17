
bool initialized = false;
String input = "";

void setup() 
{
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    digitalWrite(LED_BUILTIN, LOW);
}

void loop() 
{

  if(Serial.available() > 0)
  {
   if(initialized)
   {
     int value = Serial.read();
     if(value == '1')
     {
       digitalWrite(LED_BUILTIN, HIGH);
     }
     else
     {
       digitalWrite(LED_BUILTIN, LOW);
     }
     Serial.print("Received: " + value);
   }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
    char messageChar = (char)Serial.read();
    bool complete = false;
    input += messageChar;

      if(messageChar == '\n')
      {
        complete = true;
      }

      if(complete)
      {
        if (input.indexOf("mmi") >= 0) 
        {
          Serial.print("Ok");
          digitalWrite(LED_BUILTIN, HIGH);
          initialized = true;
        }
        input = "";
      }
    }
  }
}
