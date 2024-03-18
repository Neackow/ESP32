#include <Wire.h>

void setup()
{ 
  Serial.begin(115200);
  Wire.begin(0x40);
  Wire.onRequest(requestEvent); // register event
  Wire.onReceive(receiveEvent); // register event
}

void loop()
{
  delay(10);
  Wire.onRequest(requestEvent);
}

void requestEvent()
{
  byte b1 = 0x01;
  byte b2 = 0x69;
  byte b3 = 0x42;
  Wire.write("1");
  Wire.write("2");
  Wire.write("6");
}

void receiveEvent(int howMany)
{
  while (Wire.available() > 0)
  {
    Serial.print(Wire.read());
  }
  Serial.println();
}