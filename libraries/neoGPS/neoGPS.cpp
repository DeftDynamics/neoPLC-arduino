/*
  neoGPS.cpp - UBlox Core Messaging System
  Proprietary Deft Dynamics 2017

 This is a library for communicating with uBlox GPS modules
 Based on "u-blox 8 / u-blox M8 Receiver Description" (UBX-13003221 - R13)
 It is not likely to work with Devices older that protocol 20
*/

#include "Arduino.h"
#include "neoGPS.h"
#include <Wire.h>

neoGPS::neoGPS()
{
  // constructor
}

void neoGPS::begin(uint32_t Rate)
{
    uint32_t address = 0x42;
    Wire.begin();
    configDDCPort(address, 1,0,0, 1,0,0); // set I2C Address and protocols: transmits on UBX only
    _address = address;
  
     waitForAckNack();
     
    // silence all default NMEA messages (p 171)
    char IDs[] = { DTM, GBS, GGA, GLL, GNS, GRS, GSA, GST, GSV, RMC, VLW, VTG, ZDA};
    int numIDs = sizeof(IDs);
    for (int i=0; i<numIDs; i++){
      configMessageRate(0xF0,IDs[i],0x00);
      waitForAckNack();
    }
    configDataRate(Rate); // set message rate (pp 204)
        waitForAckNack();
        
    enableMessage(NAV,PVT,0x01); // turn on/off the PVT message
    enableMessage(NAV,SAT,0x00); // turn on/off the SAT message
}

// ------------------------------------ MESSAGES AND CHECKSUMS -------------------------------------

void neoGPS::configDDCPort(uint32_t address, bool UBX_in, bool NMEA_in, bool RTCM3_in, bool UBX_out, bool NMEA_out, bool RTCM3_out){ // (see pp 194)
// CHECK THIS ON P MODEL
  //UBX_in = true; // don't let anyone actually turn off UART inputs or the device is locked when saved (CFG-CFG)
  bool RTCM_in = false;
  
    //  prt: 0=i2c, 1=UART1, 2=???, 3=USB, 4=SPI (Each has different payload length and flags!)
    //  txR: 0 = disable TxReady
    // mode: char length, parity,#stop bits
    // baud: baudrate as U4
    //   in/out:                 Ubx  |             NMEA |                        RTCM    |                   RTCM3; // choose which protocols to enable
   uint8_t  in0 = (0b0000001* UBX_in) | (0b0000010* NMEA_in) | (0b0000100*RTCM_in) | (0b0100000* RTCM3_in); // default is 0x07
   uint8_t out0 = (0b0000001*UBX_out) | (0b0000010*NMEA_out) |                       (0b0100000*RTCM3_out); // default is 0x03
 
  uint8_t mode = (address<<1);
  
              //   { hdr, hdr, cls,  ID, ln1, ln2 |  prt, rs1,       txR,                mode,                res2,    inputs,   outputs,     flags, rs3, rs4, | ckA, ckB}
  char message[] = {  MU,BLOX, CFG, PRT,0x14,0x00,  0x00,0x00, 0x00,0x00, mode,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,  in0,0x00, out0,0x00, 0x00,0x00,0x00,0x00,  0xC0,0x7E};
  ubxChecksumAndSend(message,sizeof(message));
  if (verbose)
  {  
    Serial.printf("set DDC Address to 0x%2X, in: 0x%2X (%d %d %d), out: 0x%2X (%d %d %d)\n",DDC_ADDRESS, in0,UBX_in,NMEA_in,RTCM3_in, out0,UBX_out,NMEA_out,RTCM3_out);
  }
}

void neoGPS::configMessageRate(uint8_t CL, uint8_t ID, uint8_t IsOn){
               //  { hdr, hdr, cls,  ID, ln1, ln2 | cls,  ID, i2c, ua1, ua2, usb, spi, rsv | ckA, ckB}
  char message[] = {  MU,BLOX, CFG, MSG,0x08,0x00,   CL,  ID,IsOn,IsOn,0x00,IsOn,0x00,0x00, 0x00,0x00};
  ubxChecksumAndSend(message,sizeof(message));
  if (verbose)
  {
      Serial.printf("set %2X-%2X rate to %2X...",CL,ID,IsOn);
  }
  waitForAckNack();
}

void neoGPS::enableMessage(uint8_t CL, uint8_t ID, uint8_t IsOn){
  configMessageRate( CL, ID, IsOn);
}

void neoGPS::configDataRate(uint16_t Rate){
  char r0 = Rate  & 0xFF;
  char r1 = (Rate  >> 8) & 0xFF;
              //   { hdr, hdr, cls,  ID, ln1, ln2 | measRate,   navRate,   timeRef,| ckA, ckB}
  char message[] = {  MU,BLOX, CFG,RATE,0x06,0x00,    r0, r1, 0x01,0x00, 0x00,0x00, 0x00,0x00};
  ubxChecksumAndSend(message,sizeof(message));
  waitForAckNack();
}

void neoGPS::get(char Class, char ID){
  char message[] = {  MU,BLOX,Class,ID,0x00,0x00,0x00,0x00};
  ubxChecksumAndSend(message,sizeof(message));
}

void neoGPS::ubxReplaceChecksum(char *message, int mLength){  
  char CK_A = 0;
  char CK_B = 0;
    for (int i=2; i<(mLength-2); i++) 
    {
        CK_A = CK_A + message[i]; 
        CK_B = CK_B + CK_A;
    }
  message[mLength-2] = CK_A;
  message[mLength-1] = CK_B;
}

void neoGPS::ubxChecksumAndSend(char *message, int mLength){
  ubxReplaceChecksum(message,mLength);
    Wire.beginTransmission(DDC_ADDRESS);  // open DDC line
      for (int i = 0; i<mLength; i++){
        Wire.write(message[i]);
      }
    Wire.endTransmission();               // stop transmitting and release the port
}

bool neoGPS::confirmChecksum(char *message, int mLength){
  char CK_A = 0;
  char CK_B = 0;
    for (int i=2; i<(mLength-2); i++) 
    {
        CK_A = CK_A + message[i]; 
        CK_B = CK_B + CK_A;
    }
  if ((message[mLength-2] == CK_A) && (message[mLength-1] == CK_B)) {
    return true;
  } else {
    return false;
  }
}

uint8_t neoGPS::waitForAckNack(uint32_t timeout){
  uint8_t replyReceived = 0;
  uint32_t startTime = millis();
  while ((replyReceived < 2) && (millis()-startTime)<timeout){
       replyReceived = poll();
  }
  return replyReceived;
}


// ------------------------------------ PARTIAL MESSAGE ENCODING -------------------------------------

uint8_t neoGPS::poll(){

  uint8_t replyReceived = 0;
  
    Wire.requestFrom(_address, 255);   // request max buffer (may not get it all, but should get something)
    while (Wire.available()) {
      char c = Wire.read();
      replyReceived = encode(c);
    }

 return replyReceived;
}

uint8_t neoGPS::encode(char c){
  
  uint8_t out = 0;

  #define BUFFSIZE 512
  static char buff[BUFFSIZE];
  static int k = 0; // index in buffer
  static char prev_c = 0;
  
  static bool lookingForNewMessage = true;
  static bool messageTypeIsKnown = false;
  static bool completeMessageReceived = false;
         bool doResetStaticVariables = false;
         
  static uint32_t messageRecvTime = 0;
  uint32_t messageTimeout = 500;

  static uint8_t  Class = 0;
  static uint8_t  ID = 0;
  static uint16_t Length = 0;


  // determine if we have a header:
  if ((lookingForNewMessage == true) && ((prev_c == 0xB5) && (c == 0x62))){
    lookingForNewMessage = false;
    k = 1;
    buff[0] = 0xB5;
  }
  prev_c = c;

  // if we are in a message, start stacking the buffer:
  if (lookingForNewMessage == false){
    buff[k] = c;
    k++;
  }

  // if we are at the start of the message, identify it
  if ((messageTypeIsKnown == false) && (k==6)) {
    Class = buff[2];
    ID = buff[3];
    Length = buff[4]+(buff[5]<<8);
    messageTypeIsKnown = true;
    messageRecvTime = millis();
  }

  // if we know the message ID and length, and have received entire payload
  bool timerExceeded = ((millis()-messageRecvTime)>messageTimeout);
  if ((messageTypeIsKnown == true) && ((k == (Length+8))||(timerExceeded))){
    if (verbose)
    {
      Serial.printf("(CL = %d, ID = %d, LN = %d)... ",Class,ID,Length);
    }
    if (timerExceeded) {
      if (verbose) {Serial.printf("only received %d/%d characters\n",k-8,Length);}
      doResetStaticVariables = true;
    } else {
      if (verbose){Serial.printf("received all %d chars... ",k-8);}
      completeMessageReceived = true;
    }
  }

  if (completeMessageReceived == true){
    if (confirmChecksum(buff,Length+8)){
      if (verbose){Serial.print("checksum pass... ");}
      out = parse(buff);
    } else {
      if (verbose){Serial.print("checksum failed\n");}
    }
    doResetStaticVariables = true;
  }

  // if we have completed parsing or exceeded a timer or overflow the buffer, reset and restart
  if ((doResetStaticVariables == true)||(k >= BUFFSIZE)){
    // reset all static variables to start new search
    lookingForNewMessage = true;
    messageTypeIsKnown = false;
    completeMessageReceived = false;
    Class = 0;
    ID = 0;
    Length = 0;
    k = 0;
  }  

  return out;
}

// ------------------------------------ COMPLETE MESSAGE PARSING -------------------------------------

// identify message and parse
uint8_t neoGPS::parse(char *buffer){ 
  if (buffer[2] == NAV) { // NAV Class
        if (buffer[3] == PVT){  // NAV-PVT
             memcpy(nav_pvt.raw,buffer+6,92);
             parse_NAV_PVT(nav_pvt);
             if (verbose) {Serial.print("NAV-PVT parsed\n");}
             return 1;
        } else if (buffer[3] == SAT){  // NAV-SAT
             memcpy(nav_sat.raw,buffer+6,488);
             parse_NAV_SAT(nav_sat);
             if (verbose) {Serial.print("NAV-SAT parsed\n");}
             return 1;
        } else {
             if (verbose){Serial.print("unknown NAV message\n");}
             return 0;
        }
  } else if (buffer[2] == ACK) { // ACK/NAK Class
        if (buffer[3] == _ACK){  // ACK-ACK
             if (verbose){Serial.printf("ACK-ACK parsed\n");}
             ackNak = true;
             return 3;
        } else if (buffer[3] == _NAK){
             if (verbose){Serial.printf("ACK-NAK parsed\n");}
             ackNak = false;
             return 2;
        } else {
            if (verbose) {Serial.printf("unknown ACK/NAK message\n");}
            return 0;
        }
  } else {
        if (verbose){Serial.print("unknown message class\n");}
        return 0;
  }
}

// ------------------------------------ NAV-PVT -------------------------------------
void neoGPS::parse_NAV_PVT(NAV_PVT x){
  
   pvt.iTOW = x.pcs.iTOW;  // ms
   pvt.year = x.pcs.year ; // y
  pvt.month = x.pcs.month; // month
    pvt.day = x.pcs.day;   // d
   pvt.hour = (x.pcs.hour+TIMEZONE_OFFSET+24) % 24; //h
    pvt.min =  x.pcs.min;  // min
    pvt.sec = x.pcs.sec; // s
   pvt.tOff = x.pcs.nano*1e-9; // s
   pvt.tAcc = (double)x.pcs.tAcc*1e-9;  // s
   
     pvt.validDate = (x.pcs.valid & 0b0000001)>>0; // 'valid' bit 0
     pvt.validTime = (x.pcs.valid & 0b0000010)>>1; // 'valid' bit 1
 pvt.fullyResolved = (x.pcs.valid & 0b0000100)>>2; // 'valid' bit 2
      pvt.validMag = (x.pcs.valid & 0b0001000)>>3; // 'valid' bit 3
    
     pvt.gnssFixOK = (x.pcs.flags & 0b0000001)>>0; // 'flags' bit 0
      pvt.diffSoln = (x.pcs.flags & 0b0000010)>>1; // 'flags' bit 1
      pvt.psmState = (x.pcs.flags & 0b0011100)>>2; // 'flags' bits 2-4
  pvt.headVehValid = (x.pcs.flags & 0b0100000)>>5; // 'flags' bit 5
      pvt.carrSoln = (x.pcs.flags & 0b11000000)>>6; // 'flags' bits 6-7
      
 pvt.confirmedAvai = (x.pcs.flags2 & 0b0100000)>>5; // 'flags2' bit 5
 pvt.confirmedDate = (x.pcs.flags2 & 0b1000000)>>6; // 'flags2' bit 6
 pvt.confirmedTime = (x.pcs.flags2 & 0b10000000)>>7; // 'flags2' bit 7
       pvt.fixType = x.pcs.fixType; // mask: 0-5
   
   pvt.numSV = x.pcs.numSV;        // -
     pvt.lon = x.pcs.lon*1e-7;     // deg
     pvt.lat = x.pcs.lat*1e-7;     // deg
  pvt.height = x.pcs.height*1e-3;  // m
    pvt.hMSL = x.pcs.hMSL*1e-3;    // m
    pvt.hAcc = x.pcs.hAcc*1e-3;    // m
    pvt.vAcc = x.pcs.vAcc*1e-3;    // m
    
    pvt.velN = x.pcs.velN*1e-3;    // m/s
    pvt.velE = x.pcs.velE*1e-3;    // m/s
    pvt.velD = x.pcs.velD*1e-3;    // m/s
  pvt.gSpeed = x.pcs.gSpeed*1e-3;  // m/s
 pvt.headMot = x.pcs.headMot*1e-5; // deg
    pvt.sAcc = x.pcs.sAcc*1e-3;    // m/s
 pvt.headAcc = (float)x.pcs.headAcc*1e-5; // deg
 
    pvt.pDOP = x.pcs.pDOP*0.01;    // m/m
 pvt.headVeh = x.pcs.headVeh*1e-5; // deg
  pvt.magDec = x.pcs.magDec*1e-2;  // deg
  pvt.magAcc = x.pcs.magAcc*1e-2;  // deg
  
  pvt.isUpdated = true;
}


// ------------------------------------ NAV-SAT -------------------------------------

void neoGPS::parse_NAV_SAT(NAV_SAT x){
 
      sat.iTOW = x.pcs.iTOW;    // ms
   sat.version = x.pcs.version; // -
    sat.numSVs = x.pcs.numSVs;  // -

    for (int i = 0; i<min(40,sat.numSVs); i++){
       sat.svData[i].gnssID = x.pcs.svData[i].gnssID;
         sat.svData[i].svID = x.pcs.svData[i].svID;       // -
          sat.svData[i].cno = x.pcs.svData[i].cno;         // dBHz
         sat.svData[i].elev = x.pcs.svData[i].elev;       // deg
         sat.svData[i].azim = x.pcs.svData[i].azim;       // deg
        sat.svData[i].prRes = x.pcs.svData[i].prRes*0.1; // m
        
       sat.svData[i].quality = (x.pcs.svData[i].flags & 0x000007) >>  0;
        sat.svData[i].svUsed = (x.pcs.svData[i].flags & 0x000008) >>  3;
        sat.svData[i].health = (x.pcs.svData[i].flags & 0x000030) >>  4;
      sat.svData[i].diffCorr = (x.pcs.svData[i].flags & 0x000040) >>  6;
      sat.svData[i].smoothed = (x.pcs.svData[i].flags & 0x000080) >>  7;
   sat.svData[i].orbitSource = (x.pcs.svData[i].flags & 0x000700) >>  8;
      sat.svData[i].ephAvail = (x.pcs.svData[i].flags & 0x000800) >> 11;
      sat.svData[i].almAvail = (x.pcs.svData[i].flags & 0x001000) >> 12;
      sat.svData[i].anoAvail = (x.pcs.svData[i].flags & 0x002000) >> 13;
      sat.svData[i].aopAvail = (x.pcs.svData[i].flags & 0x004000) >> 14;
      
  sat.svData[i].sbasCorrUsed = (x.pcs.svData[i].flags & 0x010000) >> 16;
  sat.svData[i].rtcmCorrUsed = (x.pcs.svData[i].flags & 0x020000) >> 17;
  
    sat.svData[i].prCorrUsed = (x.pcs.svData[i].flags & 0x100000) >> 20;
    sat.svData[i].crCorrUsed = (x.pcs.svData[i].flags & 0x200000) >> 21; 
    sat.svData[i].doCorrUsed = (x.pcs.svData[i].flags & 0x400000) >> 22; 
    }
    
    sat.isUpdated = true;
}


// ------------------------------------ MISCELLANEOUS -------------------------------------


void neoGPS::printCharArray(char message[], uint32_t mLength){
  Serial.print("{");
  for (uint16_t i=0; i<mLength; i++){
    if (message[i]<0x10){
      if (verbose){Serial.printf(" 0%1X",message[i]);}
    } else {
      if (verbose){Serial.printf(" %2X",message[i]);}
    }
  }
  Serial.print("}");
}


