#include <Wire.h>

float dataArray[4];


void setup()
{ 
  Serial.begin(9600);

  Wire.begin(0x40); // Defines the card's slave address
  Wire.onRequest(requestEvent); // register event
  Wire.onReceive(receiveEvent); // register event
}

void loop()
{
  delay(10);
  Wire.onRequest(requestEvent);
}

//the function to be called, takes no parameters and returns nothing.
void requestEvent()
{
  byte b1 = 0x01;
  byte b2 = 0x69;
  byte b3 = 0x42;
  Wire.write("1");
  Wire.write("2");
  Wire.write("6"); // This write "1 ln 2 ln 6" on the ESP's console.
}

// the function to be called when the peripheral device receives data; this should take a single int parameter (the number of bytes read from the controller device) and return nothing.
void receiveEvent(int howMany) 
{
  for(int i=0; i < howMany; i++){
    dataArray[i] = Wire.read();
    Serial.println(dataArray[i]);
  }
  Serial.println();
}
