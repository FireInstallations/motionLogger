#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>

#ifndef APSSID
#define APSSID "motlogger"
#define APPSK  "ESPPW"
#endif

/* Set these to your desired credentials. */
const char *ssid = APSSID;
const char *password = APPSK;

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE + 1]; //buffer to hold incoming packet,
char  ReplyBuffer[] = "acknowledged\r\n";       // a string to send back



unsigned int localPort = 90;      // local port to listen on
//unsigned int localPort = 91;      // local port to send

IPAddress myIP;
WiFiUDP Udp;

void setup() {
  delay(1000);
  Serial.begin(115200);
  Serial.println();
  Serial.print("Configuring access point...");
  
  /* You can remove the password parameter if you want the AP to be open. */
  WiFi.softAP(ssid);
  Serial.println("SSID: " + (String)ssid + " pw: " + (String)password);

  myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  Serial.printf("UDP server on port %d\n", localPort);
  Udp.begin(localPort);
}

void loop() {
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Serial.printf("Received packet of size %d from %s:%d\n    (to %s:%d, free heap = %d B)\n",
                  packetSize,
                  Udp.remoteIP().toString().c_str(), Udp.remotePort(),
                  Udp.localIP().toString().c_str(), Udp.localPort(),
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
  


  
  //sendUDP((String)"hello world");
  //delay(2000);
  
}

void sendUDP(String string) {

  Serial.print("sendUDP : ");
  Serial.println(string);

  // convert string to char array
  char msg[255];
  string.toCharArray(msg,255);


  IPAddress gateway(192, 168, 4, 2);
  Udp.beginPacket(gateway, 91);
  Udp.write(msg);
  Udp.endPacket();
}
