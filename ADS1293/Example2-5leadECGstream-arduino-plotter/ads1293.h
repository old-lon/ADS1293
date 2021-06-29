//  |pin label         | Pin Function         |Arduino Connection|
//  |----------------- |:--------------------:|-----------------:|
//  | MISO             | Slave Out            |  12              |
//  | MOSI             | Slave In             |  11              |
//  | SCLK             | Serial Clock         |  13              |
//  | CS               | Chip Select          |  10              |
//  | VCC              | Digital VDD          |  +5V             |
//  | GND              | Digital Gnd          |  Gnd             |
//  | DRDY             | Data ready           |  02              |

#include "Arduino.h"
#include <SPI.h>

#define WREG 0x7f
#define RREG 0x80

#define   CONFIG  0x00
#define   FLEX_CH1_CN   0x01
#define   FLEX_CH2_CN   0x02
#define   FLEX_CH3_CN   0x03
#define   FLEX_PACE_CN  0x04
#define   FLEX_VBAT_CN  0x05

#define   LOD_CN        0x06
#define   LOD_EN        0x07
#define   LOD_CURRENT   0x08
#define   LOD_AC_CN     0x09

#define   CMDET_EN      0x0a
#define   CMDET_CN      0x0b
#define   RLD_CN        0x0c


#define   REF_CN        0x11
#define   OSC_CN        0x12

#define   AFE_RES       0x13
#define   AFE_SHDN_CN   0x14
#define   AFE_FAULT_CN  0x15
#define   AFE_PACE_CN   0x17

#define   ERR_STATUS    0x19
#define   MASK_ERR      0x2a

#define   R2_RATE       0x21
#define   R3_RATE_CH1   0x22
#define   R3_RATE_CH2   0x23
#define   R3_RATE_CH3   0x24
#define   R1_RATE       0x25

#define   DIS_EFILTER   0x26

#define   DRDYB_SRC     0x27
#define   SYNCB_CN     0x28

#define   CH_CNFG       0x2f
#define   REVID         0x40

#define   POSITIVE_TST_SIG  0x01
#define   NEGATIVE_TST_SIG  0x02
#define   ZERO_TST_SIG  0x03
#define   ERROR         -1

class ads1293
{
  public:
    uint8_t drdyPin;
    uint8_t csPin;

    void ads1293Begin_2CH();
    void ads1293Begin5LeadECG();
    int32_t getECGdata(uint8_t channel);
    bool readSensorID();
    void setAds1293Pins();
    void disableCh1();
    uint8_t ads1293ReadRegister(uint8_t rdAddress);
    uint8_t readErrorStatus(uint8_t rdAddress);
    bool attachTestSignal(uint8_t channel, uint8_t pol);
    void setSamplingRate();
    void disableFilterAllChannels();
    bool disableFilter(uint8_t channel);
    uint8_t readErrorStatus();

    ads1293(uint8_t drdy, uint8_t chipSelect){
      csPin = chipSelect;
      drdyPin = drdy;
    }

  private:
    void ads1293WriteRegister(uint8_t wrAddress, uint8_t data);
    void configDCleadoffDetect();
    void configACleadoffDetect();
};



int32_t ads1293::getECGdata(uint8_t channel){

  uint8_t rawData[3];
  int32_t ecgData;

  if(channel < 1 || channel > 3){
    return -1;    //return error, -1
  }else {
    channel -= 1;
  }

  rawData[0] = ads1293ReadRegister(0x37 + (channel * 3));
  rawData[1] = ads1293ReadRegister(0x38 + (channel * 3));
  rawData[2] = ads1293ReadRegister(0x39 + (channel * 3));

  uint32_t tempData = (uint32_t)rawData[0]<<16;
  tempData = (uint32_t)rawData[1]<< 8;
  tempData |= rawData[2];
  tempData = tempData << 8;

  
  ecgData = (int32_t) (tempData); 
  return (ecgData );
}

void ads1293::setAds1293Pins(){
  pinMode(drdyPin, INPUT_PULLUP);
  pinMode(csPin,   OUTPUT);
}

void ads1293::ads1293Begin_2CH(){

  ads1293WriteRegister(FLEX_CH1_CN, 0x0C); //CH1+ Route to In1 
                                           //CH1- Route to In4
  delay(1);


  ads1293WriteRegister(FLEX_CH2_CN, 0x0D); //CH2+ Route to In2 
                                           //CH2- Route to In5
  delay(1);

 
  //ads1293WriteRegister(CMDET_EN, 0x11);  //CM_DET OFF
  delay(1);
  
  //ads1293WriteRegister(CMDET_CN, 0x07);  //
  delay(1);

  ads1293WriteRegister(RLD_CN, 0x03); //RLD V_cm route to In3 RLD bufer on to REF pin
  delay(1);
 
  ads1293WriteRegister(AFE_RES, 0x1B); //CH1 and CH2 high resolution
  delay(1);

  //ads1293WriteRegister(AFE_PACE_CN, 0x04);
  //delay(1);

  //ads1293WriteRegister(REF_CN, 0x03); delay(1);

  ads1293WriteRegister(OSC_CN, 0x04);
  delay(1);

  ads1293WriteRegister(AFE_SHDN_CN, 0x24); //0x24 RUN CH1 and CH2 
  delay(1);

// 2400 kHz per channel
  ads1293WriteRegister(R1_RATE, 0x03); // CH1 and CH2 R1 = 2
  delay(1);
  ads1293WriteRegister(R2_RATE, 0x04); // R2 = 6
  delay(1);
  ads1293WriteRegister(R3_RATE_CH1, 0x04); //R3 = 8
  delay(1);
  ads1293WriteRegister(R3_RATE_CH2, 0x04);
  delay(1);

  ads1293WriteRegister(DRDYB_SRC, 0x08);
  delay(1);

  ads1293WriteRegister(CH_CNFG, 0x30);
  delay(1);

  ads1293WriteRegister(CONFIG, 0x01); // Start conversion
  delay(1);
}

void ads1293::ads1293Begin5LeadECG(){

//channel 1 cn
  ads1293WriteRegister(0x01, 0x11);
  delay(1);

  //channel2
  ads1293WriteRegister(0x02, 0x19);
  delay(1);

///channel3 cn
  ads1293WriteRegister(0x03, 0x2e);
  delay(1);

  //Common-Mode Detection and Right-Leg Drive Common-Mode Feedback Control Register
  ads1293WriteRegister(0x0a, 0x07);
  delay(1);

  //RLD control
  ads1293WriteRegister(0x0c, 0x04);
  delay(1);

  ads1293WriteRegister(0x0d, 0x01);
  delay(1);

  ads1293WriteRegister(0x0e, 0x02);
  delay(1);

  ads1293WriteRegister(0x0f, 0x03); //0000 0011
  delay(1);

  ads1293WriteRegister(0x10, 0x01);
  delay(1);

  ads1293WriteRegister(0x12, 0x04);
  delay(1);


//debug
 // ads1293WriteRegister(0x13, 0x07);
  delay(1);

//debug
  ads1293WriteRegister(0x14, 0x00);
  delay(1);

  ads1293WriteRegister(0x21, 0x02);
  delay(1);

  ads1293WriteRegister(0x22, 0x02);
  delay(1);

  ads1293WriteRegister(0x23, 0x02);
  delay(1);

  ads1293WriteRegister(0x24, 0x02);
  delay(1);

  ads1293WriteRegister(0x27, 0x08);
  delay(1);

  ads1293WriteRegister(0x2f, 0x70);
  delay(1);

  ads1293WriteRegister(00, 0x01);
  delay(1);
}



void ads1293::ads1293WriteRegister(uint8_t wrAddress, uint8_t data){

  uint8_t dataToSend = (wrAddress & WREG);
  digitalWrite(csPin, LOW);
  SPI.transfer(dataToSend);
  SPI.transfer(data);
  digitalWrite(csPin, HIGH);
}

uint8_t ads1293::ads1293ReadRegister(uint8_t rdAddress){

  uint8_t rdData;
  uint8_t dataToSend = (rdAddress  | RREG);
  digitalWrite(csPin, LOW);
  SPI.transfer(dataToSend);
  rdData = SPI.transfer(0);
  digitalWrite(csPin, HIGH);

  return (rdData);
}

bool ads1293::readSensorID(){

  uint8_t ID=0xff;
  ID = ads1293ReadRegister(REVID);
  Serial.println(ID);
  if(ID != 0xff){
    return true;
  }else return false;
}


void ads1293::configDCleadoffDetect(){


}


void ads1293::configACleadoffDetect(){


}


void ads1293::setSamplingRate(){


}

void ads1293::disableCh1(){

  ads1293WriteRegister(FLEX_CH1_CN, 0x00);
  delay(1);
}


void ads1293::disableFilterAllChannels(){

  ads1293WriteRegister(DIS_EFILTER, 0x07);
  delay(1);
}


bool ads1293::disableFilter(uint8_t channel){
  
  if(channel > 3 || channel < 0){
    Serial.println("Wrong channel error!");
    return false;
  }

  uint8_t channelBitMask = 0x01;
  channelBitMask = channelBitMask << (channel-1);
  ads1293WriteRegister(DIS_EFILTER, channelBitMask);
  delay(1);
}


uint8_t ads1293::readErrorStatus(){

  return (ads1293ReadRegister(ERR_STATUS));
}


bool ads1293::attachTestSignal(uint8_t channel, uint8_t pol){

  if((channel > 3) || (channel <1)){
    Serial.println("Wrong channel error!");
    return ERROR;
  }

  pol = (pol<<6);
  ads1293WriteRegister((channel) , pol);
  delay(1);

  return true;
}








/* 

void ads1293::ads1293Begin_2CH(){

  ads1293WriteRegister(FLEX_CH1_CN, 0x11); //CH1+ Route to In1 
                                           //CH1- Route to In2
  delay(1);


  ads1293WriteRegister(FLEX_CH2_CN, 0x2E); //CH2+ Route to In5 
                                           //CH2- Route to In6
  delay(1);

 
  ads1293WriteRegister(CMDET_EN, 0x11);  //In 5 In1
  delay(1);
   ads1293WriteRegister(CMDET_CN, 0x07);  //
  delay(1);

  ads1293WriteRegister(RLD_CN, 0x03); //RLD route to In3 RLD bufer on
  delay(1);
  ads1293WriteRegister(0x0D, 0x04); //Wilson1 route to In4 bufer on
  delay(1);

  ads1293WriteRegister(AFE_RES, 0x1B); //CH1 and CH2 high resolution
  delay(1);
  
  ads1293WriteRegister(OSC_CN, 0x04);
  delay(1);

  ads1293WriteRegister(AFE_SHDN_CN, 0x24); //0x24 RUN CH1 and CH2 
  delay(1);

  ads1293WriteRegister(R1_RATE, 0x03); // CH1 and CH2 R1 = 2
  delay(1);
  ads1293WriteRegister(R2_RATE, 0x04); // R2 = 6
  delay(1);

  ads1293WriteRegister(R3_RATE_CH1, 0x04); //R3 = 8
  delay(1);

  ads1293WriteRegister(R3_RATE_CH2, 0x04);
  delay(1);

  ads1293WriteRegister(DRDYB_SRC, 0x08);
  delay(1);

  ads1293WriteRegister(CH_CNFG, 0x30);
  delay(1);

  ads1293WriteRegister(CONFIG, 0x01); // Start conversion
  delay(1);
}

*/
