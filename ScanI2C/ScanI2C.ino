//wire library for I2c
#include <Wire.h> 

byte dataArray[4] = {0x10, 0x11, 0x12, 0x13};

//Setup function runs once
void setup()
{  
  //Start wire1 communication
  Wire.begin();
  Wire.setClock(100000);
  //Start serial at 9600 baud
  Serial.begin(9600);
  //Wait for serial connection
  while (!Serial); 
}

void loop()
{

  byte b1 = 0x01;
  byte b2 = 0x02;
  Wire.beginTransmission(64);
  //Wire.write(b1);
  //Wire.write(b2); // This will write "12" on the Pico's console.
  for (int i=0; i<4; i++)  {
    Wire.write(dataArray[i]);  //data bytes are queued in local buffer
  }
  Wire.endTransmission();

  Wire.requestFrom(0x40, 3);    // Request 6 bytes (1 BYTE = 1 CHAR) from slave device at address 0x40.
  // Slave may send less than requested
  while(Wire.available()) {
      char c = Wire.read();    // Receive a byte as character
      Serial.print(c);         // Prints the character
      Serial.println();
  }

  //wait to repeat the scan
  delay(5000); 
}

void Scani2c(int wireSelection){

  //variable for error code
  byte errorCode;
  //variable for device address
  byte deviceAddress; 
  //variable to hold total number of devices
  int totalDevices=0;
  //Print to serial monitor 
  Serial.printf("Now Scanning I2c port %d\n", wireSelection);
  //According to the I2c specification the first 8 and last 8 addresses are reserved
  //This loop loops through the addresses to find devices
  for (deviceAddress = 8; deviceAddress < 120; deviceAddress++ )
  {
    if(wireSelection==1){
    //Start wire 1 transmission on the current address
    Wire.beginTransmission(deviceAddress);
    //listen for a message from a device on that address
    errorCode = Wire.endTransmission();}else
    {
     //Start wire transmission on the current address
    Wire.beginTransmission(deviceAddress);
    //listen for a message from a device on that address
    errorCode = Wire.endTransmission(); 
    }
    
    
    //error code 0 will return if a device responds
    if (errorCode == 0)
    {
      //output the address to the serial monitor
      Serial.print("I2C device found at address 0x");
      //Add leading zero for addresses before 16
      if (deviceAddress < 16)
        Serial.print("0");
      Serial.print(deviceAddress, HEX);
      //Increment the device count
      totalDevices++;
    }
    //An error of 4 useually means a bad connection but can be some other failure.
    else if (errorCode == 4)
    {
      //print an error message
      Serial.print("Error at address 0x");
      if (deviceAddress < 16)
        Serial.print("0");
      Serial.println(deviceAddress, HEX);
    }
  }
  //if no dvices found let the user knoe.
  if (totalDevices == 0)
    Serial.println("No I2C devices found");
  else//otherwise tell the user how many devices were detected
    Serial.printf("\n%d", totalDevices);
    if(totalDevices>0){
    if(totalDevices<2){
    Serial.println(" I2c device found");}else
    Serial.println(" I2c devices found");
    }
    Serial.println("done\n");  
}
