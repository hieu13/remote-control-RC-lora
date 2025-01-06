
#include <LoRa.h>
#include "boards.h"
#include "crc8.h"
#include "EEPROM.h"
//define PIN_LED_GREEN  25
uint8_t  data_telemetry8[15];
uint16_t data_telemetry16[15];
uint32_t data_telemetry32[15];

int PIN_CH1,PIN_CH2,PIN_CH3,PIN_CH4,PIN_CH5,PIN_CH6,PIN_CH7,PIN_CH8,PIN_CH9;

uint8_t idx_fhss_schema = 0; 

#define MAX_LISTEN_TIME_ON_HOP_CHANNEL 112 //in ms. If no packet received within this time, we hop 

//--------------------------------------------------
unsigned long timeOfLastPacket,totalPacketsSent = 0;
uint8_t transmitterID = 0; //settable during bind
uint8_t receiverID = 0;    //randomly generated on bind
uint8_t idxRFPowerLevel = 0;
//#define MAX_PACKET_SIZE  80
#define MAX_PACKET_SIZE_RECEVER  19
//uint8_t packet[MAX_PACKET_SIZE];
uint8_t packet_recever[MAX_PACKET_SIZE_RECEVER];

enum{
  PAC_BIND                   = 0x0,
  PAC_ACK_BIND               = 0x1,
  PAC_READ_OUTPUT_CH_CONFIG  = 0x2,
  PAC_SET_OUTPUT_CH_CONFIG   = 0x3,
  PAC_ACK_OUTPUT_CH_CONFIG   = 0x4,
  PAC_RC_DATA                = 0x5,
  PAC_TELEMETRY              = 0x6,
};
uint8_t packetType = 0xFF;
bool isRequestingTelemetry = false;
uint16_t externalVolts = 0; //in millivolts
uint16_t telem_volts = 0x0FFF;     // in 10mV, sent by receiver with 12bits.  0x0FFF "No data"
uint16_t mode_select;
bool failsafeEverBeenReceived = false;
uint32_t rcPacketCount = 0;
uint32_t lastRCPacketMillis = 0;
uint8_t channel_1,channel_2,channel_3,channel_4,channel_5,channel_6,channel_7,channel_8;
int ch1to9Vals[10];
int ch1to9Failsafes[10];
int ch1to9Out[10];
uint32_t ch10to12Out[3];
uint8_t outputChConfig[10]; //0 digital, 1 Servo, 2 PWM
uint8_t maxOutputChConfig[10];
bool lora_stt;
//Declare an output pins array
int myOutputPins[9] = {PIN_CH1, PIN_CH2, PIN_CH3, PIN_CH4, PIN_CH5, PIN_CH6, PIN_CH7, PIN_CH8, PIN_CH9};
int i_cout,count100;
//-------------- EEprom stuff --------------------

#define EE_INITFLAG         0xBB 
#define EE_ADR_INIT_FLAG    0
#define EE_ADR_TX_ID        1
#define EE_ADR_RX_ID        2
#define EE_ADR_FHSS_SCHEMA  3
#define EE_ADR_RX_CH_CONFIG 20
#define EEPROM_SIZE 2

//--------------- Function Declarations ----------

void bind();
void hop();
void sendTelemetry();
void writeOutputs();
void getExternalVoltage();
uint8_t buildPacket(uint8_t srcID, uint8_t destID, uint8_t dataIdentifier, uint8_t *dataBuff, uint8_t dataLen);
bool checkPacket(uint8_t srcID, uint8_t destID, uint8_t dataIdentifier, uint8_t *packetBuff, uint8_t packetSize);
uint8_t getMaxOutputChConfig(int pin);

void setup()
{   
   initBoard();
   Serial.println("start...");
  if (!EEPROM.begin(EEPROM_SIZE))
  {
    Serial.println("failed to initialise EEPROM"); while (1);
  }
  // Read from EEPROM
  transmitterID = byte(EEPROM.read(0));
  receiverID    = byte(EEPROM.read(1));
  //Serial.println(transmitterID);
  //Serial.println(receiverID);
  //######################################################
    delay(1500);
    Serial.print(transmitterID);
    Serial.print("-");
    Serial.println(receiverID);
    Serial.println("LoRa Receiver");
    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DI0_PIN);
    if (!LoRa.begin(LoRa_frequency)) {
        Serial.println("Starting LoRa failed!");
        while (1);
    }
    //LoRa.setSyncWord(0xFA);
    Serial.println("Begin lora");
    //bind();
}

void loop()
{
  
  int packetSize = LoRa.parsePacket();
  bool hasValidPacket = false;
  uint8_t dataBuff[32];
  memset(dataBuff, 0, sizeof(dataBuff));
  if (packetSize > 0) //received a packet
  {
    timeOfLastPacket = millis();
    uint8_t msgBuff[30];
    memset(msgBuff, 0, sizeof(msgBuff));
    uint8_t cntr = 0;
    lora_stt=false;
    while (LoRa.available() > 0) 
    {
      if(cntr < (sizeof(msgBuff)/sizeof(msgBuff[0])))
      {
        msgBuff[cntr] = LoRa.read();
        cntr++;
      }
      else // discard any extra data
        LoRa.read(); 
    }
    
    //check packet 
    if(checkPacket_recever(transmitterID, receiverID, PAC_RC_DATA, msgBuff, packetSize))
    { mode_select = msgBuff[2] >>4;
      if((msgBuff[2] & 0x0F) == 12)
      {
        hasValidPacket = true;
        packetType = PAC_RC_DATA;
        memcpy(dataBuff, msgBuff + 3, 12);
      }
    }
    else {lora_stt=false;}
  if(hasValidPacket)
  {       
          lora_stt=true;
          hasValidPacket = false;
          ++rcPacketCount;
          
          lastRCPacketMillis = millis();
          //digitalWrite(PIN_LED_ORANGE, HIGH);
    
          //Decode
          int ch1to9Tmp[10];
          ch1to9Tmp[0] = ((uint16_t)dataBuff[0] << 2 & 0x3fc) |  ((uint16_t)dataBuff[1] >> 6 & 0x03); //ch1
          ch1to9Tmp[1] = ((uint16_t)dataBuff[1] << 4 & 0x3f0) |  ((uint16_t)dataBuff[2] >> 4 & 0x0f); //ch2
          ch1to9Tmp[2] = ((uint16_t)dataBuff[2] << 6 & 0x3c0) |  ((uint16_t)dataBuff[3] >> 2 & 0x3f); //ch3
          ch1to9Tmp[3] = ((uint16_t)dataBuff[3] << 8 & 0x300) |  ((uint16_t)dataBuff[4]      & 0xff); //ch4
          ch1to9Tmp[4] = ((uint16_t)dataBuff[5] << 2 & 0x3fc) |  ((uint16_t)dataBuff[6] >> 6 & 0x03); //ch5
          ch1to9Tmp[5] = ((uint16_t)dataBuff[6] << 4 & 0x3f0) |  ((uint16_t)dataBuff[7] >> 4 & 0x0f); //ch6
          ch1to9Tmp[6] = ((uint16_t)dataBuff[7] << 6 & 0x3c0) |  ((uint16_t)dataBuff[8] >> 2 & 0x3f); //ch7
          ch1to9Tmp[7] = ((uint16_t)dataBuff[8] << 8 & 0x300) |  ((uint16_t)dataBuff[9]     & 0xff);  //ch8
          ch1to9Tmp[8] = ((uint8_t)dataBuff[10]& 0xFF); //ch9
          ch1to9Tmp[9] = ((uint8_t)dataBuff[11] & 0xFF); //ch10
          //Check if failsafe data. If so, dont modify outputs
//          if((dataBuff[11] >> 4) & 0x01) //failsafe values
//          {
//            failsafeEverBeenReceived = true;
//            for(int i = 0; i < 9; i++)
//              ch1to9Failsafes[i] = ch1to9Tmp[i] - 500 ; //Center at 0 so range is -500 to 500
//          }
//          else //normal channel values
//          {
            ch1to9Vals[0] = map(ch1to9Tmp[0], 100,900 , 1000, 2000); 
            ch1to9Vals[1] = map(ch1to9Tmp[1], 100,900 , 1000, 2000);
            ch1to9Vals[2] = map(ch1to9Tmp[2], 100,900 , 1000, 2000);
            ch1to9Vals[3] = map(ch1to9Tmp[3],  0,1000 , 1000, 2000);//thr
            ch1to9Vals[4] = map(ch1to9Tmp[4], 100,900 , 1000, 2000);
            ch1to9Vals[5] = map(ch1to9Tmp[5], 100,900 , 1000, 2000);
            ch1to9Vals[6] = map(ch1to9Tmp[6], 100,900 , 1000, 2000);
            ch1to9Vals[7] = map(ch1to9Tmp[7], 100,900 , 1000, 2000);
            ch1to9Vals[8] = ch1to9Tmp[8];
            ch1to9Vals[9] = ch1to9Tmp[9];
            for(int i = 0; i < 10; i++){ch1to9Out[i]= ch1to9Vals[i]; }
//            Serial.print(ch1to9Tmp[7]);
//            Serial.print("\t");
//            Serial.println(ch1to9Vals[7]);
//#############TO SEND DATA TO QUAD #######################
          //SendSerialData();
//#######################################################
//          for(uint8_t i = 0; i < 9; i++)              
//          {
//          Serial.print(ch1to9Vals[i]);
//          Serial.print(",");
//          }
//          Serial.println("|");

 hop();    
}
 else {lora_stt=false;}   
}
    
  //sendTelemetry();
  SerialCommunication_send();
  bar_signal();
  //SerialCommunication_recv();
  //oled_lcd();
  //i_cout++;
  //count100 = totalPacketsSent;
  

 // 
}

void bar_signal(){

  channel_1 = map(ch1to9Vals[0], 1000, 2000, 1, 60);
  channel_2 = map(ch1to9Vals[1], 1000, 2000, 1, 60);
  channel_3 = map(ch1to9Vals[2], 1000, 2000, 1, 60);
  channel_4 = map(ch1to9Vals[3], 1000, 2000, 1, 60);
  channel_5 = map(ch1to9Vals[4], 1000, 2000, 1, 60);
  channel_6 = map(ch1to9Vals[5], 1000, 2000, 1, 60);
  channel_7 = map(ch1to9Vals[6], 1000, 2000, 1, 60);
  //channel_8 = map(ch1to9Vals[7], 1000, 2000, 1, 60);
  
  u8g2->clearBuffer();          // clear the internal memory
  //u8g2->setCursor(0, 0);
  u8g2->drawBox(10,0, channel_1,6);
  u8g2->drawBox(10,8, channel_2,6);
  u8g2->drawBox(10,16,channel_3,6);
  u8g2->drawBox(10,24,channel_4,6);
  u8g2->drawBox(10,32,channel_5,6);
  u8g2->drawBox(10,40,channel_6,6);
  u8g2->drawBox(10,48,channel_7,6);
 // u8g2->drawBox(10,56,channel_8,6);


u8g2->setFont(u8g2_font_6x12_mr);
u8g2->setFontDirection(1);
u8g2->setCursor(80, 0);
u8g2->print("rssi:"+String(LoRa.packetRssi()));
u8g2->setCursor(95, 0);
u8g2->print("Snr:"+String(LoRa.packetSnr()));
u8g2->setCursor(110, 0);
if (lora_stt == true){
  u8g2->print("Connected");}
else {u8g2->print("Not connected");}
  u8g2->sendBuffer();
}

void oled_lcd(){
  u8g2->clearBuffer();          // clear the internal memory
  u8g2->setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  u8g2->drawStr(0,10,"Xin Chao");  // write something to the internal memory
  u8g2->setCursor(0, 20);
  //u8g2->print(String(i_cout));
  u8g2->print("Quadcopter");
  u8g2->setCursor(0, 30);
  u8g2->print("Running");
  u8g2->setCursor(0, 40);
  u8g2->print("Be safe");
  u8g2->sendBuffer();
}
void SerialCommunication_send(){
   enum {
    RF_ENABLED  = 0x08,
    TEMP_1      = 0x10,
    POW_TEMP   = 0x20,
  };
 
  
  ///----------- ENCODE ---------
  uint8_t status0 = 0x03; //key 
  uint8_t status1 = 0x08; 
  uint8_t status2 = 0x01;
  uint8_t tmpBuff_send[22];//36>>22
  memset(tmpBuff_send, 0, sizeof(tmpBuff_send));
  
  tmpBuff_send[0] = status0;
  tmpBuff_send[1] = status1;
  tmpBuff_send[2] = status2; 
  
    for(uint8_t i = 0; i < 8; i++)
    {
      uint16_t val = ch1to9Out[i] & 0xFFFF; 
      tmpBuff_send[3 + i*2] = (val >> 8) & 0xFF;
      tmpBuff_send[4 + i*2] = val & 0xFF;
    }
//     for(uint8_t i = 0; i < 3; i++)
//    {
//      uint32_t val1 = ch10to12Out[i] & 0xFFFFFFFF; 
//      tmpBuff_send[21 + i*4] = (val1 >> 24) & 0xFF;
//      tmpBuff_send[22 + i*4] = (val1 >> 16) & 0xFF;
//      tmpBuff_send[23 + i*4] = (val1 >> 8) & 0xFF;
//      tmpBuff_send[24 + i*4] = val1 & 0xFF;
//    }
    tmpBuff_send[19] = ch1to9Out[8];//ch9
    if (lora_stt==true){tmpBuff_send[20] = ch1to9Out[9]+128;}//ch10 } 
    else {tmpBuff_send[20] = ch1to9Out[9];}//ch10 } 
    
  //add a crc
  tmpBuff_send[21] = crc8Maxim(tmpBuff_send, 21);//35>>21
  
  //Send to slave mcu
  Serial1.write(tmpBuff_send, sizeof(tmpBuff_send));
}
//void SerialCommunication_recv(){
//   ///--------- CHECK IF READY ------------------
//  
//  const uint8_t msgLength = 76;
//  if (Serial1.available() < msgLength)
//  {
//    return;
//  }
//  
//  /// -------- READ INTO TEMP BUFFER ------------
//  uint8_t tmpBuff_recv[msgLength]; 
//  uint8_t cntr = 0;
//  while (Serial1.available() > 0)
//  {
//    if (cntr < msgLength) 
//    {
//      tmpBuff_recv[cntr] = Serial1.read();
//      cntr++;
//    }
//    else //Discard any extra data
//      Serial1.read();
//  }
//  //------ CHECK CRC ---------------------------
//  if(tmpBuff_recv[msgLength - 1] != crc8Maxim(tmpBuff_recv, msgLength - 1))
//  {
//    return;
//  }  
//  //------ EXTRACT -----------------------------
//  
//  for(uint8_t i = 0; i < 11; i++)
//    {
//      data_telemetry8[i] = tmpBuff_recv[i];
//      Serial.println(data_telemetry8[i]);
//    }
//
//  for(uint8_t i = 0; i < 10; i++){
//     data_telemetry16[i] = (uint16_t)tmpBuff_recv[11 + i * 2] << 8 | (uint16_t)tmpBuff_recv[12 + i * 2];
//     Serial.println(data_telemetry16[i]);
//     } 
//  
//  for(uint8_t i = 0; i < 11; i++)
//    {
//       data_telemetry32[i] = (uint32_t)tmpBuff_recv[31 + i * 4] << 24 | (uint32_t)tmpBuff_recv[32 + i * 4]<<16| (uint32_t)tmpBuff_recv[33 + i * 4]<<8| (uint32_t)tmpBuff_recv[34 + i * 4];
//     
//     Serial.println(UintToFloat(data_telemetry32[i]));
//    } 
// 
//  
//  
//  }
void hop(){
  
    LoRa.sleep();
    LoRa.setFrequency(LoRa_frequency);
    LoRa.idle();
  
}
//void sendTelemetry(){
//  //Calculate packets per second
//  static bool transmitInitiated = false;
//  static bool hopPending = false;
//  static uint32_t prevRCPacketCount = 0; 
//  static uint32_t ttPrevMillis = 0;
//  static uint8_t rcPacketsPerSecond = 0; 
//  uint32_t ttElapsed = millis() - ttPrevMillis;
//  if (ttElapsed >= 1000)
//  {
//    ttPrevMillis = millis();
//    rcPacketsPerSecond = ((rcPacketCount - prevRCPacketCount) * 1000) / ttElapsed;
//    prevRCPacketCount = rcPacketCount;
//  }
//  else {return;}
//  if(millis() - lastRCPacketMillis > 1000)
//    rcPacketsPerSecond = 0;
//
//  ////////////////////////////////////////////////////////////
//  uint8_t dataToSend[75];
//  
//  memset(dataToSend, 0, sizeof(dataToSend));
//data_telemetry8[0]=1;
//data_telemetry8[1]=2;
//data_telemetry8[2]=3;
//data_telemetry8[3]=4;
//data_telemetry8[4]=5;
//data_telemetry8[5]=6;
//data_telemetry8[6]=7;
//data_telemetry8[7]=8;
//data_telemetry8[8]=9;
//data_telemetry8[9]=10;
//
//
////8byte
//  for(uint8_t i = 0; i < 11; i++)
//    {
//      dataToSend[i] = data_telemetry8[i];
//    }
//
////16byte
//  for(uint8_t i = 0; i < 10; i++)
//    {
//      uint16_t val = (data_telemetry16[i]) & 0xFFFF; 
//      dataToSend[11 + i*2] = (val >> 8) & 0xFF;
//      dataToSend[12 + i*2] = val & 0xFF;
//    }
////32byte
//  for(uint8_t i = 0; i < 11; i++)
//    {
//      uint32_t val1 = data_telemetry32[i] & 0xFFFFFFFF; 
//      dataToSend[31 + i*4] = (val1 >> 24) & 0xFF;
//      dataToSend[32 + i*4] = (val1 >> 16) & 0xFF;
//      dataToSend[33 + i*4] = (val1 >> 8) & 0xFF;
//      dataToSend[34 + i*4] = val1 & 0xFF;
//    }
//    
// // dataToSend[75] = crc8Maxim(dataToSend, 75);
//
//
//
//  ////////////////////////////////////////////////////////////
//  uint8_t _packetLen = buildPacket(receiverID, transmitterID, PAC_TELEMETRY, dataToSend, sizeof(dataToSend));
//  
//  //Serial.print("im in");
//  if(LoRa.beginPacket())
//  {
//    LoRa.write(packet, _packetLen);
//    LoRa.endPacket(); //block until done transmitting
//    delay(1);
//    //Serial.println("OK");
//    transmitInitiated = true;
//    totalPacketsSent++;
//  }
//    if(!LoRa.isTransmitting())
//  {
//    //hasPendingRCData = false;
//    transmitInitiated = false;
//    if(hopPending)
//    {
//      hop();
//      hopPending = false;
//    }
//  }
//}
//void bind(){
//  LoRa.sleep();
//  LoRa.setFrequency(LoRa_frequency);
//  LoRa.idle();
//  Serial.println("bind_mode");
//  const uint16_t BIND_LISTEN_TIMEOUT = 5000;
//  uint8_t msgBuff[30];
//  memset(msgBuff, 0, sizeof(msgBuff));
//  //---- listen for bind -----
//  bool receivedBind = false;
//  uint32_t stopTime = millis() + BIND_LISTEN_TIMEOUT;
//  while(millis() < stopTime)
//  { 
//    int packetSize = LoRa.parsePacket();
//    if (packetSize > 0) //received a packet
//    {
//      //read into buffer
//      uint8_t cntr = 0;
//      while (LoRa.available() > 0) 
//      { 
//        if(cntr < (sizeof(msgBuff)/sizeof(msgBuff[0])))
//        {
//          msgBuff[cntr] = LoRa.read();
//          cntr++;
//        }
//        else // discard any extra data
//          LoRa.read(); 
//       
//      }
//      // Check packet
//      if( checkPacket(msgBuff[0], 0x00, PAC_BIND, msgBuff, packetSize) && msgBuff[0] > 0x00)
//      {  
//        if((msgBuff[2] & 0x0F) == 3) //check length 
//        {
//          receivedBind = true;
//          break; //exit while loop
//        }}}
//    delay(2);
//    
//  }
//  
//  if(!receivedBind)
//  {
//    hop(); //set to operating frequencies
//    Serial.println("bind_fail");
//  }
//  
//  if(receivedBind)
//  {  transmitterID = byte(msgBuff[0]);
//    //---- send reply 
//    uint8_t dataToSend[1]; 
//    //generate random receiverID
//    randomSeed(millis()); //Seed PRNG
//    receiverID = byte(random(0x01, 0xFF));
//    dataToSend[0] = receiverID;
//    delay(2);
//    uint8_t _packetLen = buildPacket(0x00, transmitterID, PAC_ACK_BIND, dataToSend, sizeof(dataToSend));
//    if(LoRa.beginPacket())
//    {
//      LoRa.write(packet, _packetLen);
//      LoRa.endPacket(); //blocking
//    }
//    hop();
//    
//    //indicate we received bind
//    for(int i = 0; i < 5; i++) //flash led 3 times
//    {
//      digitalWrite(BOARD_LED, HIGH);
//      delay(200);
//      digitalWrite(BOARD_LED, LOW);
//      delay(200);
//    }
//    EEPROM.write(0, transmitterID);
//    Serial.print(transmitterID);
//    Serial.print("-");
//    EEPROM.write(1, receiverID);
//    Serial.println(receiverID);
//    Serial.println("bind_success");
//    EEPROM.commit();
//  }
//   packetType = PAC_RC_DATA;
//   Serial.println("bind mode off >>> RC mode");
//}
//uint8_t buildPacket(uint8_t srcID, uint8_t destID, uint8_t dataIdentifier, uint8_t *dataBuff, uint8_t dataLen){
//  // Builds packet and returns its length
//  
//  packet[0] = srcID;
//  packet[1] = destID;
//  dataLen &= 0xFF; //limit 0x0F
//  packet[2] = (dataIdentifier << 4) | dataLen;
//  for(uint8_t i = 0; i < dataLen; i++)
//  {
//    packet[3 + i] = *dataBuff;
//    ++dataBuff;
//  }
//  packet[4 + dataLen] = crc8Maxim(packet, 4 + dataLen);
//  return 5 + dataLen; 
//}
//bool    checkPacket(uint8_t srcID, uint8_t destID, uint8_t dataIdentifier, uint8_t *packetBuff, uint8_t packetSize){
//  if(packetSize < 4 || packetSize > 80) //packet may be from other Lora radios
//    return false;
//  
//  if(packetBuff[0] != srcID || packetBuff[1] != destID || (packetBuff[2] >> 4) != dataIdentifier)
//    return false;
//  
//  //check packet crc
//  uint8_t _crcQQ = packetBuff[packetSize - 1];
//  uint8_t _computedCRC = crc8Maxim(packetBuff, packetSize - 1);
//  if(_crcQQ != _computedCRC)
//    return false;
//  
//  return true;
//}
uint8_t buildPacket_recever(uint8_t srcID, uint8_t destID, uint8_t dataIdentifier, uint8_t *dataBuff, uint8_t dataLen){
  // Builds packet and returns its length
  
  packet_recever[0] = srcID;
  packet_recever[1] = destID;
  dataLen &= 0x0F;
  packet_recever[2] = (dataIdentifier << 4) | dataLen;
  for(uint8_t i = 0; i < dataLen; i++)
  {
    packet_recever[3 + i] = *dataBuff;
    ++dataBuff;
  }
  packet_recever[3 + dataLen] = crc8Maxim(packet_recever, 3 + dataLen);
  return 4 + dataLen; 
}
bool    checkPacket_recever(uint8_t srcID, uint8_t destID, uint8_t dataIdentifier, uint8_t *packetBuff, uint8_t packetSize){
  if(packetSize < 4 || packetSize > 19) //packet may be from other Lora radios
    return false;
  
  if(packetBuff[0] != srcID || packetBuff[1] != destID || (packetBuff[2] >> 4) != dataIdentifier)
    return false;
  
  //check packet crc
  uint8_t _crcQQ = packetBuff[packetSize - 1];
  uint8_t _computedCRC = crc8Maxim(packetBuff, packetSize - 1);
  if(_crcQQ != _computedCRC)
    return false;
  
  return true;
}
uint32_t FloatToUint(float n){
   return (uint32_t)(*(uint32_t*)&n);
}
float   UintToFloat(uint32_t n){
   return (float)(*(float*)&n);
}
