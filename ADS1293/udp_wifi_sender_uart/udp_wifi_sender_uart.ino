#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
//#include <WiFiClient.h>

const char* ssid = "UDP_UART";
const char* password = "";

WiFiUDP Udp;
unsigned int localUdpPort = 4210;
char incomingPacket[50];

String msg;
char cmsg[50];

void setup()
{
  Serial.begin(230400);
  Serial.println();
  delay(5000);
/*  
  // Режим клиента
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
*/
 WiFi.mode(WIFI_AP);  
  WiFi.begin(ssid, password);

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  Udp.begin(localUdpPort);
  Serial.printf("Now listening at IP %s, UDP port %d\n", myIP.toString().c_str(), localUdpPort);


}

void loop()
{
  if(Serial.available()>0)
  {
    msg=Serial.readString();
  for(int i(0);i<msg.length();i++)cmsg[i]=msg[i];
  Serial.print(cmsg);
  };
  int packetSize = Udp.parsePacket();
  if (packetSize > 0)
  {
    Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
    int len = Udp.read(incomingPacket, 10);
    if (len > 0)
    {
      incomingPacket[len] = 0;
      Serial.printf("UDP packet contents: %s\n", incomingPacket);
      //Обработчик команд
    };

    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    //Ответ на команду
    Udp.write(cmsg);
    Udp.endPacket(); 
  }
}
