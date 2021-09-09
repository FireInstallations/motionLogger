#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

/* ----------------- { MMA } ----------------- */
#define MMA8451_ADDRESS           0x1D     //(0x3A >> 1)    // this is  0x1D and means SA0 = 1

// register keys
#define MMA8451_REG_STATUS        0x00 // returns z status
#define MMA8451_REG_OUT_Z_MSB     0x05 // returns z component
#define MMA8451_REG_WHOAMI        0x0D // returns MMA

#define MMA8451_REG_CTRL_REG1     0x2A // setup
#define MMA8451_REG_CTRL_REG2     0x2B // reset

/* The WeMos D1 Mini I2C bus uses pins:
   D1 = SCL
   D2 = SDA
*/
const int sclPin = D1;
const int sdaPin = D2;

/* ----------------- { E8266 } ----------------- */

#ifndef APSSID
#define APSSID "motlogger"
#define APPSK  "uF0Ro(k5!"
#endif

/* Set these to your desired credentials. */
const char *ssid = APSSID;
const char *password = APPSK;

WiFiUDP Udp;
const unsigned int localPort = 90;       // local port to listen on
const unsigned int remotePort = 91;      // remote port to send to
IPAddress myIP;                    //local IP
IPAddress remoteIP(192, 168, 4, 2); //remote IP to send to

/* ----------------- { General } ----------------- */
// system status
#define WAITING_FOR_INPUT 0 // waiting for control input via udp
#define READING_VALUES    1 // reading values fom MMA
#define WRITING_VALUES    2 // writing data array on udp

byte SystemStatus = WAITING_FOR_INPUT;

//measured values
int Data [256];
long Time [256];
byte i = 0;

//time between values
unsigned long OperationTime;

float Frequency = 400;

//buffer to hold incoming packet
char packetBuffer[UDP_TX_PACKET_MAX_SIZE + 1];

//-------------------- setup --------------------
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output

  delay(1000);
  Serial.begin(115200);
  Serial.println();

  /* ----------------- { MMA } ----------------- */
  Wire.begin(sdaPin, sclPin);

  // Check if MMA sensor was found
  if (readRegister8(MMA8451_REG_WHOAMI) != 0x1A)
  {
    /* No MMA8451 detected don't do anything */
    Serial.println("MMA not found");
    while (true);
  } else //turn LED on to sigalize the MMA is working

    //Reset MMA
    writeRegister8(MMA8451_REG_CTRL_REG2, 0x40);
  while (readRegister8(MMA8451_REG_CTRL_REG2) & 0x40);

  //  set data rate 400 Hz(0x28), low noise mode (0x04), activate operation(0x01)
  writeRegister8(MMA8451_REG_CTRL_REG1, 0x0D | 0x04 | 0x01);

  /* ----------------- { E8266 } ----------------- */
  Serial.println("Configuring access point...");

  /* You can add the password parameter if you don't want the AP to be open. */
  WiFi.softAP(ssid);
  Serial.println("SSID: " + (String)ssid);
  //WiFi.softAP(ssid, password);
  //Serial.println("SSID: " + (String)ssid + " pw: " + (String)password);

  myIP = WiFi.softAPIP();
  Serial.print("Me ->");
  Serial.print(myIP);

  Udp.begin(localPort);
  Serial.printf(":%d\n", localPort);

  Serial.print("remote ->");
  Serial.print(remoteIP);
  Serial.printf(":%d\n", remotePort);

}

//-------------------- loop --------------------
void loop() {
  switch (SystemStatus) {
    case WAITING_FOR_INPUT:
      digitalWrite(LED_BUILTIN, LOW);

      if (readUDP()) {
        SystemStatus = READING_VALUES;

        //turn LED off to indicate we are reading MMA values now
        digitalWrite(LED_BUILTIN, HIGH);

        //SystemStatus = READING_VALUES;
        OperationTime = micros();
      }
      break;

    case READING_VALUES:
      //wait for data
      while (!TestIfMMAReady() );
      Data[i] = round(MMAreadZ ()); // derives raw value from MMA

      //while the frequency didn't reatched the next data point read more data (if enouth time was given
      while (micros() - OperationTime < max(((1000000 / Frequency) - 2500), (float)0.0) ) {
        while (!TestIfMMAReady() );

        Data[i] = round(MMAreadZ() * 0.7 + Data[i] * 0.3);
      }
      //wait the resttime for next value
      while (micros() - OperationTime < (1000000 / Frequency));

      //add time to datapoint
      if (i)
        Time[i] = round(Time[i - 1] + (micros() - OperationTime) / 100); //time since last value
      else Time[i] = 0;
      i++;
      OperationTime = micros(); // new timer

      // if i is rolling over we got all 256 values
      if (i == 0) {
        SystemStatus = WRITING_VALUES;
      }
      break;

    case WRITING_VALUES:
      digitalWrite(LED_BUILTIN, HIGH);

      //send a value per time
      sendUDP((String)Time[i] + ";" + (String)Data[i] + ";");
      i++;
      // if i is rolling over we got all 256 values
      if (i == 0) {
        SystemStatus = WAITING_FOR_INPUT;
      } else
        delay(20);

      digitalWrite(LED_BUILTIN, LOW);
      break;
  }

}

//-------------------- Subroutines ----------------

/* ----------------- { MMA } ----------------- */

// Reads status bit number 2 and returns true if it was 1.
bool TestIfMMAReady () {
  return readRegister8((uint8_t)MMA8451_REG_STATUS) & (uint8_t)4;
}

//Reads values from MMA
float MMAreadZ() {
  int16_t helperz;

  // read x y z at once
  Wire.beginTransmission(MMA8451_ADDRESS);
  Wire.write((uint8_t)MMA8451_REG_OUT_Z_MSB);
  Wire.endTransmission(false); // MMA8451 + friends uses repeated start!!

  Wire.requestFrom(MMA8451_ADDRESS, 2);

  // Shift values to create properly formed integer (MSB first)
  // Note MSB before LSB and 14 bit value
  helperz = Wire.read() << 8; helperz |= Wire.read(); helperz >>= 2;

  return (float)helperz;
}

//Writes 8-bits to MMA
void writeRegister8(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MMA8451_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
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

/* ----------------- { E8266 } ----------------- */
bool readUDP() {
  // did we got something?
  int packetSize = Udp.parsePacket();

  if (packetSize) {
    // read the packet into packetBufffer
    int n = Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    packetBuffer[n] = 0;

    //is it a valid frequency?
    if (isInteger((String)packetBuffer)) {
      Frequency = ((String)packetBuffer).toInt();

      Serial.println("Got new frequency: " + (String)Frequency + " from " + Udp.remoteIP().toString() + ":" + (String)Udp.remotePort());
      sendUDP("got it; Set f to " + (String)Frequency + ";");

      return true;
    } else {
      Serial.println("Got new text: " + (String)packetBuffer + " from " + Udp.remoteIP().toString() + ":" + (String)Udp.remotePort());
      Serial.println("Didn't understand and therefor ignored it.");
      sendUDP("Didn't understand; Please repeat;");

      return false;
    }


  } else
    return false;
}


//Sends messages via UDP
void sendUDP(String string) {
  Serial.print("sendUDP: ");
  Serial.println(string);

  // convert string to char array
  char msg[255];
  string.toCharArray(msg, 255);

  Udp.beginPacket(remoteIP, remotePort);
  Udp.write(msg);
  Udp.endPacket();
}

/* ----------------- { General } ----------------- */
inline bool isInteger(const String s) {
  if ( ((!isdigit(s[0])) && (s[0] != '-') && (s[0] != '+'))) return false;

  char * p;
  strtol(s.c_str(), &p, 10);

  return (*p == 0);
}
