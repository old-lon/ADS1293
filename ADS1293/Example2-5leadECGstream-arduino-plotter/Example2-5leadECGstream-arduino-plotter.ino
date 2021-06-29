
#include "ads1293.h"
#include <SPI.h>

#define DRDY_PIN                02
#define CS_PIN                  10
#define buf 6
ads1293 ADS1293(DRDY_PIN, CS_PIN);
  
  char DataPacket[buf];
  volatile  int32_t ecgCh1;
  volatile  int32_t ecgCh2;

void drdyInterruptHndlr()
{
  
   ecgCh1 = ADS1293.getECGdata(1);
   ecgCh2 = ADS1293.getECGdata(2);  

  DataPacket[0] = ecgCh1;
  DataPacket[1] = ecgCh1 >> 8;
  DataPacket[2] = ecgCh1 >> 16;

  DataPacket[3] = ecgCh2;
  DataPacket[4] = ecgCh2 >> 8;
  DataPacket[5] = ecgCh2 >> 16;

}

void enableInterruptPin(){
  pinMode(ADS1293.drdyPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ADS1293.drdyPin), drdyInterruptHndlr, FALLING);
}

void setup(){
  Serial.begin(230400);
  SPI.begin();
  ADS1293.setAds1293Pins();
  //ADS1293.attachTestSignal(1,1); ADS1293.attachTestSignal(2,1);
  ADS1293.ads1293Begin_2CH();
  enableInterruptPin();
  delay(1000);
}


void loop() {

  for(int i=0; i<buf; i++)Serial.write(DataPacket[i]);
  Serial.println();
 
  /*
  if(b=='t')
  {
  Serial.print(ecgCh1);
  Serial.print(" ");

  Serial.print(ecgCh2);
  Serial.print(" ");
  Serial.println();
    }; */
}
