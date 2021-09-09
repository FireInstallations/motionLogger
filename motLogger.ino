#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

/* ----------------- { E8266 } ----------------- */

#ifndef APSSID
#define APSSID "motlogger"
#define APPSK  "uF0Ro(k5!"
#endif

/* ----------------- { MMA } ----------------- */
#define MMA8451_ADDRESS           0x1D     //(0x3A >> 1)    // this is  0x1D and means SA0 = 1

// register keys
#define MMA8451_REG_OUT_X_MSB     0x01
#define MMA8451_REG_WHOAMI        0x0D

#define MMA8451_REG_CTRL_REG1     0x2A
#define MMA8451_REG_CTRL_REG2     0x2B

/* ----------------- { E8266 } ----------------- */

/* Set these to your desired credentials. */
const char *ssid = APSSID;
const char *password = APPSK;
int data[256][2];
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

long SendTime;

/* ----------------- { MMA } ----------------- */

/* The WeMos D1 Mini I2C bus uses pins:
   D1 = SCL
   D2 = SDA
*/
const int sclPin = D1;
const int sdaPin = D2;

// sensor values
float mx = 0, my = 0, mz = 0;

void setup() {
  SendTime = millis();

  delay(1000);
  Serial.begin(115200);
  Serial.println();
  Serial.println("Configuring access point...");

  /* You can remove the password parameter if you want the AP to be open. */
  WiFi.softAP(ssid);
  Serial.println("SSID: " + (String)ssid + " pw: " + (String)password);

  myIP = WiFi.softAPIP();
  Serial.print("Me ->");
  Serial.print(myIP);

  Udp.begin(localPort);
  Serial.printf(":%d\n", localPort);

  Serial.print("remote ->");
  Serial.print(remoteIP);
  Serial.printf(":%d\n", remotePort);

  Wire.begin(sdaPin, sclPin);
  // Fill stack
  for (int i = 0; i <= 255; i++) {
    data[i][0] = i;
    delay(10);
  }
  /*
      // Check if MMA sensor was found
      if (readRegister8(MMA8451_REG_WHOAMI) != 0x1A)
      {
        /* No MMA8451 detected don't do anything */
  /* Serial.println("MMA not found");
    while (true);*/

  /*
    //Reset MMA
    writeRegister8(MMA8451_REG_CTRL_REG2, 0x40);
    while (readRegister8(MMA8451_REG_CTRL_REG2) & 0x40);
    //  set data rate 12.5 Hz(0x28), low noise mode (0x04), activate operation(0x01)
    writeRegister8(MMA8451_REG_CTRL_REG1, 0x28 | 0x04 | 0x01);
  */

}

void loop() {
  if (!stackfull) {

    // if there's data available, read a packet
    int packetSize = Udp.parsePacket();
    if (packetSize) {
      Serial.printf("Received packet of size %d from %s:%d\n    (to %s:%d, free heap = %d B)\n",
                    packetSize,
                    Udp.remoteIP().toString().c_str(), Udp.remotePort(),
                    Udp.destinationIP().toString().c_str(), Udp.localPort(),
                    ESP.getFreeHeap());

      // read the packet into packetBufffer
      int n = Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
      packetBuffer[n] = 0;
      Serial.println("Contents:");
      Serial.println(packetBuffer);

      // send a reply, to the IP address and port that sent us the packet we received
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write(ReplyBuffer);
      Udp.endPacket();

    }

    for (int i = 1; i <= 255; i++)
      data[i][1] = i * 10;
    stackfull = true;

  }

  //MMAread ();// derives raw values from MMA
  //Serial.println((String)(mx) + ";" + (String)(my) + ";" + (String)(mz) + ";");
  else if (!endtrans) {
    delay(5);
    endtrans = true;
    Serial.println(endtrans);


    for (int i = 0; i <= 255; i++) {
      int date = data[i][1];
      sendUDP((String)(millis() - SendTime) + ";" + (String)(millis() - SendTime) + ";" + (String)(date) + ";");
      SendTime = millis();
      delay(200);
      

    }

  }
}

//-------------------- Subroutines ----------------

//Sends messages via UDP
void sendUDP(String string) {
  Serial.print("sendUDP : ");
  Serial.println(string);

  // convert string to char array
  char msg[255];
  string.toCharArray(msg, 255);

  Udp.beginPacket(remoteIP, remotePort);
  Udp.write(msg);
  Udp.endPacket();
}

//Reads values from MMA
void MMAread() {
  int16_t helperx, helpery, helperz;

  // read x y z at once
  Wire.beginTransmission(MMA8451_ADDRESS);
  Wire.write((uint8_t)MMA8451_REG_OUT_X_MSB);
  Wire.endTransmission(false); // MMA8451 + friends uses repeated start!!

  Wire.requestFrom(MMA8451_ADDRESS, 6);

  // Shift values to create properly formed integer (MSB first)
  // Note MSB before LSB and 14 bit value
  helperx = Wire.read() << 8; helperx |= Wire.read(); helperx >>= 2;
  helpery = Wire.read() << 8; helpery |= Wire.read(); helpery >>= 2;
  helperz = Wire.read() << 8; helperz |= Wire.read(); helperz >>= 2;

  //filtering values with gliding avarage
  mx = (float)helperx * 0.3 + mx * 0.7;
  my = (float)helperx * 0.3 + my * 0.7;
  mz = (float)helperz * 0.3 + mz * 0.7;
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
