#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

/* ----------------- { E8266 } ----------------- */

#ifndef APSSID
#define APSSID "motlogger"
#define APPSK  "uF0Ro(k5!"
#endif

/* Set these to your desired credentials. */
const char *ssid = APSSID;
const char *password = APPSK;
bool stackfull = false;
bool endtrans = false;

WiFiUDP Udp;
unsigned int localPort = 90;       // local port to listen on
unsigned int remotePort = 91;      // remote port to send to
IPAddress myIP;                    //local IP
IPAddress remoteIP(192, 168, 4, 2); //remote IP to send to

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE + 1]; //buffer to hold incoming packet,
char  ReplyBuffer[] = "acknowledged\r\n";       // a string to send back

/* ----------------- { MMA } ----------------- */
#define MMA8451_ADDRESS           0x1D     //(0x3A >> 1)    // this is  0x1D and means SA0 = 1

// register keys
#define MMA8451_REG_OUT_X_MSB     0x01
#define MMA8451_REG_WHOAMI        0x0D

#define MMA8451_REG_CTRL_REG1     0x2A
#define MMA8451_REG_CTRL_REG2     0x2B

/* The WeMos D1 Mini I2C bus uses pins:
 * D1 = SCL
 * D2 = SDA
 */
const int sclPin = D1;
const int sdaPin = D2;

// sensor values
float mz = 0;



/* ----------------- { General } ----------------- */
long OperationTime;

int Data [256][2];
byte i = 0;
byte GotAll = 0;

void setup() {  
  delay(1000);
  Serial.begin(115200);
  Serial.println();

  Wire.begin(sdaPin, sclPin);

  // Check if MMA sensor was found
  if (readRegister8(MMA8451_REG_WHOAMI) != 0x1A)
  {
    /* No MMA8451 detected don't do anything */
    Serial.println("MMA not found");
    while (true);
  }

  //Reset MMA
  writeRegister8(MMA8451_REG_CTRL_REG2, 0x40);
  while (readRegister8(MMA8451_REG_CTRL_REG2) & 0x40);
  //  set data rate 12.5 Hz(0x28), low noise mode (0x04), activate operation(0x01)
  writeRegister8(MMA8451_REG_CTRL_REG1, 0x28 | 0x04 | 0x01);

  OperationTime = micros();
}

void loop() {

  MMAread ();// derives raw values from MMA

  if (GotAll == 0) {
    //fill aray with [time, mz]
    Data[i][0] = OperationTime-micros();
    OperationTime = micros(); 
    Data[i++][1] = round(mz);

    if (i == 0) {
      GotAll = 1;
      i = 255;
      }
    
  } else if (GotAll = 1){
    if (i == 0) {
        GotAll = 2;
      }
      
    Serial.println((String)Data[i][0] + ": " + (String)Data[i--][1]);

      
    }



  
  //Serial.println((String)) + (String)(mz) + ";");
  
  
}

//-------------------- Subroutines ----------------

//Reads values from MMA
void MMAread(){
  int16_t helperz;

  
  // read x y z at once
  Wire.beginTransmission(MMA8451_ADDRESS);
  Wire.write((uint8_t)MMA8451_REG_OUT_X_MSB);
  Wire.endTransmission(false); // MMA8451 + friends uses repeated start!!

  Wire.requestFrom(MMA8451_ADDRESS, 6);

  // Shift values to create properly formed integer (MSB first)
  // Note MSB before LSB and 14 bit value
  Wire.read();  Wire.read(); 
  Wire.read(); Wire.read();
  helperz = Wire.read() << 8; helperz |= Wire.read(); helperz >>= 2;

  mz = (float)helperz;
}

//Reads 8-bits from MMA
uint8_t readRegister8(uint8_t reg) {
  Wire.beginTransmission(MMA8451_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false); // MMA8451 + friends uses repeated start!!

  Wire.requestFrom(MMA8451_ADDRESS, 1);
  if (! Wire.available()) return -1;
  return ((uint8_t)Wire.read());
}

//Writes 8-bits to MMA
void writeRegister8(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MMA8451_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}
