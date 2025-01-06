/*
  LoRa Duplex communication

  Sends a message every half second, and polls continually
  for new incoming messages. Implements a one-byte addressing scheme,
  with 0xFF as the broadcast address.

  Uses readString() from Stream class to read payload. The Stream class'
  timeout may affect other functuons, like the radio's callback. For an

  created 28 April 2017
  by Tom Igoe
*/
#include <SPI.h>              // include libraries
#include <LoRa.h>
#include "crc8.h"
#include <EEPROM.h>

HardwareSerial Serial1 (PB11, PC4);

#define joystick_1    PA0
#define joystick_2    PA1
#define joystick_3    PA4
#define joystick_4    PB0
#define joystick_5    PC5
#define joystick_6    PB1
#define joystick_7    PC1
#define joystick_8    PC0

#define Switch_1_A    PA11
#define Switch_1_B    PA12
#define Switch_2_A    PC8
#define Switch_2_B    PC9
#define Switch_3_A    PB8
#define Switch_3_B    PC6
#define Switch_4_A    PB4
#define Switch_4_B    PB14
#define Switch_5_A    PA10
#define Switch_5_B    PB3
#define Switch_6_A    PB5
#define Switch_6_B    PB13

unsigned long totalPacketsSent = 0;
int rollIn, pitchIn, throttleIn, yawIn, qua_lai,accellerate,up_downIn,left_rightIn;
int rollIn_count,pitchIn_count, throttleIn_count, yawIn_count, qua_lai_count,accellerate_count,up_downIn_count,left_rightIn_count;
int dead_band_max = 600;
int dead_band_min = 400;
int joystickValue_1,joystickValue_2,joystickValue_3,joystickValue_4,joystickValue_5,joystickValue_6,joystickValue_7,joystickValue_8;
const int csPin = PB3;          // LoRa radio chip select
const int resetPin = PB10;       // LoRa radio reset
const int irqPin = PA8;         // change for your board; must be a hardware interrupt pin
uint16_t      data_joystickch1to9[9];
String outgoing;              // outgoing message

byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xBB;     // address of this device
byte destination = 0xFF;      // destination to send to
long lastSendTime = 0;        // last send time
int interval = 2000;          // interval between sends
byte switch1_A_stt,switch2_A_stt,switch3_A_stt,switch4_A_stt,switch5_A_stt,switch6_A_stt;
byte switch1_B_stt,switch2_B_stt,switch3_B_stt,switch4_B_stt,switch5_B_stt,switch6_B_stt;
uint8_t transmitterID = 0; //set on bind
uint8_t receiverID = 0;
uint8_t stt_1,stt_2,stt_3;
uint8_t stt_tx1,stt_tx2,stt_tx3;
uint8_t stt_lora1,stt_lora2,stt_lora3;
//#define MAX_PACKET_SIZE               80
#define MAX_PACKET_SEND_SIZE          19
//uint8_t       packet[MAX_PACKET_SIZE];
uint8_t       packet_send[MAX_PACKET_SEND_SIZE];

enum{
  PAC_BIND                   = 0x0,
  PAC_ACK_BIND               = 0x1,
  PAC_READ_OUTPUT_CH_CONFIG  = 0x2,
  PAC_SET_OUTPUT_CH_CONFIG   = 0x3,
  PAC_ACK_OUTPUT_CH_CONFIG   = 0x4,
  PAC_RC_DATA                = 0x5,
  PAC_TELEMETRY              = 0x6,
};
uint8_t buildPacket(uint8_t srcID, uint8_t destID, uint8_t dataIdentifier, uint8_t *dataBuff, uint8_t dataLen);
bool checkPacket(uint8_t srcID, uint8_t destID, uint8_t dataIdentifier, uint8_t *packetBuff, uint8_t packetSize);
uint8_t sum_switch_1,sum_switch_2;



void setup() {
  Serial.begin(250000);                   // initialize serial
  while (!Serial);
    pinMode(joystick_1, INPUT_PULLUP); 
    pinMode(joystick_2, INPUT_PULLUP); 
    pinMode(joystick_3, INPUT_PULLUP); 
    pinMode(joystick_4, INPUT_PULLUP); 
    pinMode(joystick_5, INPUT_PULLUP); 
    pinMode(joystick_6, INPUT_PULLUP); 
    pinMode(joystick_7, INPUT_PULLUP); 
    pinMode(joystick_8, INPUT_PULLUP); 

    pinMode(Switch_1_A, INPUT_PULLUP);
    pinMode(Switch_1_B, INPUT_PULLUP); 
    pinMode(Switch_2_A, INPUT_PULLUP); 
    pinMode(Switch_2_B, INPUT_PULLUP); 
    pinMode(Switch_3_A, INPUT_PULLUP); 
    pinMode(Switch_3_B, INPUT_PULLUP); 
    pinMode(Switch_4_A, INPUT_PULLUP); 
    pinMode(Switch_4_B, INPUT_PULLUP); 
    pinMode(Switch_5_A, INPUT_PULLUP); 
    pinMode(Switch_5_B, INPUT_PULLUP); 
    pinMode(Switch_6_A, INPUT_PULLUP); 
    pinMode(Switch_6_B, INPUT_PULLUP); 
    
    analogReadResolution(10);
    Serial.println("LoRa Duplex");
    transmitterID = EEPROM.read(0);
    receiverID = EEPROM.read(1);
    // override the default CS, reset, and IRQ pins (optional)
    LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin
    if (!LoRa.begin(868E6)) {             // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

    //LoRa.setSyncWord(0xFA);           // ranges from 0-0xFF, default 0x34, see API docs
    transmitterID = EEPROM.read(0);
    receiverID = EEPROM.read(1);
  Serial.print(transmitterID);
  Serial.print("-");
  Serial.println(receiverID);
  Serial.println("LoRa init succeeded.");
}

void loop() {
  joystick_readanalog();
  computeChannelOutputs();
  transmitRCdata();
//  if (millis() - lastSendTime > 1) {
//    transmitRCdata();
//    lastSendTime = millis();            // timestamp the message
//  }
// onReceive(LoRa.parsePacket());
}

void transmitRCdata() {
  uint8_t dataToSend[12];
    memset(dataToSend, 0, sizeof(dataToSend));
    
    dataToSend[0]  = (data_joystickch1to9[0] >> 2) & 0xFF;
    dataToSend[1]  = (data_joystickch1to9[0] << 6 | data_joystickch1to9[1] >> 4) & 0xFF;
    dataToSend[2]  = (data_joystickch1to9[1] << 4 | data_joystickch1to9[2] >> 6) & 0xFF;
    dataToSend[3]  = (data_joystickch1to9[2] << 2 | data_joystickch1to9[3] >> 8) & 0xFF;
    dataToSend[4]  = data_joystickch1to9[3] & 0xFF;
    
    dataToSend[5]  = (data_joystickch1to9[4] >> 2) & 0xFF;
    dataToSend[6]  = (data_joystickch1to9[4] << 6 | data_joystickch1to9[5] >> 4) & 0xFF;
    dataToSend[7]  = (data_joystickch1to9[5] << 4 | data_joystickch1to9[6] >> 6) & 0xFF;
    dataToSend[8]  = (data_joystickch1to9[6] << 2 | data_joystickch1to9[7] >> 8) & 0xFF;
    dataToSend[9] = data_joystickch1to9[7] & 0xFF;
    
    dataToSend[10] = sum_switch_1 & 0xFF;
    dataToSend[11] = sum_switch_2 & 0xFF;
    //Serial.println(data_joystickch1to9[1]);
    
    
          
uint8_t _packetLen = buildPacket_send(transmitterID, receiverID, PAC_RC_DATA, dataToSend, sizeof(dataToSend));
    
    if(LoRa.beginPacket())
    {
      LoRa.write(packet_send, _packetLen);
      LoRa.endPacket(); //async
      delay(1);
      totalPacketsSent++;
      }
      hop();
}

void hop(){
  
    LoRa.sleep();
    LoRa.setFrequency(868E6);
    LoRa.idle();
}
//void onReceive(int packetSize) {
//  if (packetSize > 0) 
//  {
//
//  uint8_t msgBuff[80];
//    memset(msgBuff, 0, sizeof(msgBuff));
//    uint8_t cntr = 0;
//    
//  while (LoRa.available()) {
//
//    if(cntr < 80)
//      {
//        msgBuff[cntr] = LoRa.read();
//        cntr++;
//        
//      }
//      else // discard any extra data
//        LoRa.read(); 
//  }
////  Serial.println(checkPacket(receiverID,transmitterID, PAC_TELEMETRY, msgBuff, packetSize));
//if(checkPacket(receiverID, transmitterID, PAC_TELEMETRY, msgBuff, packetSize))
//    { 
//      for(uint8_t i = 0; i < 9; i++){
//        Serial.print(msgBuff[i]);
//      }
//      Serial.println("-");
//    }
//    }
//}
//uint8_t buildPacket(uint8_t srcID, uint8_t destID, uint8_t dataIdentifier, uint8_t *dataBuff, uint8_t dataLen){
//  // Builds packet and returns its length
//  
//  packet[0] = srcID;
//  packet[1] = destID;
//  dataLen &= 0xFF;
//  packet[2] = (dataIdentifier << 4) | dataLen;
//  for(uint8_t i = 0; i < dataLen; i++)
//  {
//    packet[3 + i] = *dataBuff;
//    ++dataBuff;
//  }
//  packet[4 + dataLen] = crc8Maxim(packet, 4 + dataLen);
//  return 5 + dataLen; 
//}
//bool checkPacket(uint8_t srcID, uint8_t destID, uint8_t dataIdentifier, uint8_t *packetBuff, uint8_t packetSize){
//
//  
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
uint8_t buildPacket_send(uint8_t srcID, uint8_t destID, uint8_t dataIdentifier, uint8_t *dataBuff, uint8_t dataLen){
  // Builds packet and returns its length
  
  packet_send[0] = srcID;
  packet_send[1] = destID;
  dataLen &= 0x0F;
  packet_send[2] = (dataIdentifier << 4) | dataLen;
  for(uint8_t i = 0; i < dataLen; i++)
  {
    packet_send[3 + i] = *dataBuff;
    ++dataBuff;
  }
  packet_send[3 + dataLen] = crc8Maxim(packet_send, 3 + dataLen);
  return 4 + dataLen; 
}
bool checkPacket_send(uint8_t srcID, uint8_t destID, uint8_t dataIdentifier, uint8_t *packetBuff, uint8_t packetSize){
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
void joystick_readanalog(){
    joystickValue_1 = analogRead(joystick_1);
    joystickValue_2 = analogRead(joystick_2);
    joystickValue_3 = analogRead(joystick_3);
    joystickValue_4 = analogRead(joystick_4);
    joystickValue_5 = analogRead(joystick_5);
    joystickValue_6 = analogRead(joystick_6);
    joystickValue_7 = analogRead(joystick_7);
    joystickValue_8 = analogRead(joystick_8);

    switch1_A_stt = !digitalRead(Switch_1_A);
    switch1_B_stt = !digitalRead(Switch_1_B);
    switch2_A_stt = !digitalRead(Switch_2_A);
    switch2_B_stt = !digitalRead(Switch_2_B);
    switch3_A_stt = !digitalRead(Switch_3_A);
    switch3_B_stt = !digitalRead(Switch_3_B);
    switch4_A_stt = !digitalRead(Switch_4_A);
    switch4_B_stt = !digitalRead(Switch_4_B);
    switch5_A_stt = !digitalRead(Switch_5_A);
    switch5_B_stt = !digitalRead(Switch_5_B);
    switch6_A_stt = !digitalRead(Switch_6_A);
    switch6_B_stt = !digitalRead(Switch_6_B);



    
    //add deadzone to roll, pitch, yaw sticks CENTERs. 
    rollIn = map(joystickValue_8, 0, 1024, 0, 1000);
    //Serial.print(rollIn);
    //Serial.print("\t");
    if (rollIn < dead_band_max && rollIn > dead_band_min){rollIn=500;}
    if (rollIn > dead_band_max ){
      rollIn_count=rollIn-dead_band_max;
      rollIn=500+rollIn_count;}
    if (rollIn < dead_band_min ){
      rollIn_count=dead_band_min-rollIn;
      rollIn=500-rollIn_count;}
    rollIn = constrain(rollIn, 100, 900);
   
  //=============================================
    pitchIn = map(joystickValue_6, 0, 1024, 1000, 0);
    //Serial.print(pitchIn);
    if (pitchIn < dead_band_max && pitchIn > dead_band_min){pitchIn=500;}
    
    if (pitchIn > dead_band_max ){
      pitchIn_count=pitchIn-dead_band_max;
      pitchIn=500+pitchIn_count;
      }
    if (pitchIn < dead_band_min ){
      pitchIn_count=dead_band_min-pitchIn;
      pitchIn=500-pitchIn_count;
      }
    pitchIn = constrain(pitchIn, 100, 900);
    
  //==============================================
    yawIn = map(joystickValue_4, 0, 1024, 0, 1000);
    if (yawIn < dead_band_max && yawIn > dead_band_min){yawIn=500;}
    if (yawIn > dead_band_max ){
      yawIn_count=yawIn-dead_band_max;
      yawIn=500+yawIn_count;
      }
    if (yawIn < dead_band_min ){
      yawIn_count=dead_band_min-yawIn;
      yawIn=500-yawIn_count;
      }
    yawIn = constrain(yawIn, 0, 1000);
  //==============================================
    throttleIn = map(joystickValue_2, 0, 1024, 0, 1000);
    if (throttleIn < 50 ){throttleIn=0;}
    throttleIn = constrain(throttleIn, 0, 1000);

  
  //UP DOWN
  up_downIn = map(joystickValue_1, 0, 1024, 0, 1000); 
  if (up_downIn < dead_band_max && up_downIn > dead_band_min){up_downIn=500;}
  if (up_downIn > dead_band_max ){
      up_downIn_count=up_downIn-dead_band_max;
      up_downIn=500+up_downIn_count;
      }
    if (up_downIn < dead_band_min ){
      up_downIn_count=dead_band_min-up_downIn;
      up_downIn=500-up_downIn_count;
      }
  up_downIn = constrain(up_downIn, 100, 900);
  //
  //LEFT RIGHT
  left_rightIn = map(joystickValue_3, 0, 1024, 0, 1000); 
  if (left_rightIn < dead_band_max && left_rightIn > dead_band_min){left_rightIn=500;}
  if (left_rightIn > dead_band_max ){
      left_rightIn_count=left_rightIn-dead_band_max;
      left_rightIn=500+left_rightIn_count;
      }
    if (left_rightIn < dead_band_min ){
      left_rightIn_count=dead_band_min-left_rightIn;
      left_rightIn=500-left_rightIn_count;
      }
  left_rightIn = constrain(left_rightIn,0, 1000);
  //
  //qua lai 
  qua_lai = map(joystickValue_7, 0, 1024, 0, 1000); 
  if (qua_lai < dead_band_max && qua_lai > dead_band_min){qua_lai=500;}
  if (qua_lai > dead_band_max ){
      qua_lai_count=qua_lai-dead_band_max;
      qua_lai=500+qua_lai_count;
      }
    if (qua_lai < dead_band_min ){
      qua_lai_count=dead_band_min-qua_lai;
      qua_lai=500-qua_lai_count;
      }
  qua_lai = constrain(qua_lai,0, 1000);
  //len xuong
  //accellerate = map(joystickValue_5, 0, 1024, 0, 1000); 
  //if (accellerate < 50){accellerate=0;}
 // accellerate = constrain(accellerate,0, 1000);

//    if (switch1_stt==0){accellerate = 1000;}
//    else {accellerate = 0;};

  
sum_switch_1 = switch1_A_stt + (switch1_B_stt<<1)+(switch2_A_stt<<2)+(switch2_B_stt<<3)+
             (switch3_A_stt<<4)+(switch3_B_stt<<5);
             
sum_switch_2 =(switch4_A_stt)+(switch4_B_stt<<1)+(switch5_A_stt<<2)+(switch5_B_stt<<3)+
            (switch6_A_stt<<4)+(switch6_B_stt<<5);
    
    Serial.print(sum_switch_1);
    Serial.print("\t");
    Serial.println(sum_switch_2);
    }
void computeChannelOutputs(){
  data_joystickch1to9[0] = rollIn ;
  data_joystickch1to9[1] = pitchIn ;
  data_joystickch1to9[2] = yawIn; 
  data_joystickch1to9[3] = throttleIn; 
  data_joystickch1to9[4] = up_downIn;
  data_joystickch1to9[5] = left_rightIn;
  data_joystickch1to9[6] = qua_lai;
  data_joystickch1to9[7] = accellerate;
  //data_joystickch1to9[8] = sum_switch;
}
