/***************************************************************************
neoPLC-GPS Library

A library for UBlox GPS 

This is a library for communicating with uBlox GPS modules
Based on "u-blox 8 / u-blox M8 Receiver Description" (UBX-13003221 - R13)
It is not likely to work with Devices older that protocol 20
 
Copyright (c) 2017, Deft Dynamics
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:
   This product includes software developed by Deft Dynamics.
4. Neither the name of Deft Dynamics nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY DEFT DYNAMICS ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL DEFT DYNAMICS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************/

#ifndef __NEOGPS_H__
#define __NEOGPS_H__

#include "Arduino.h"
#include <Wire.h>

// Synchronizing Header
#define   MU 0xB5
#define BLOX 0x62

// Message Class
#define   ACK 0x05
#define   AID 0x0B
#define   CFG 0x06
#define   ESF 0x10
#define   HNR 0x28
#define   INF 0x04
#define   LOG 0x21
#define   MGA 0x13
#define   MON 0x13
#define   NAV 0x01
#define   RXM 0x02
#define   SEC 0x27
#define   TIM 0x0D
#define   UPD 0x09
#define  NMEA 0xF0
#define  PUBX 0xF1
#define RTCM3 0xF3

// Message ID
#define   _ACK 0x01 // ACK
#define   _NAK 0x00

#define   _HNR 0x5C // CFG
#define    MSG 0x01
#define   _ODO 0x1E
#define   NAV5 0x24
#define  NAVX5 0x23
#define    PRT 0x00
#define    PWR 0x57
#define   RATE 0x08
#define TMODE3 0x71

#define    INS 0x15 // ESF
#define   MEAS 0x02
#define    RAW 0x03
#define   STAT 0x10

#define   _PVT 0x00 // HNR

#define    ATT 0x05 // NAV
#define    PVT 0x07
#define    ODO 0x09
#define RESETODO 0x10
#define   SVIN 0x3B
#define STATUS 0x03
#define RELPOSNED 0x3C
#define    SAT 0x35

#define    DTM 0x0A // NMEA
#define    GBS 0x09
#define    GGA 0x00
#define    GLL 0x01
#define    GNS 0x0D
#define    GRS 0x06
#define    GSA 0x02
#define    GST 0x07
#define    GSV 0x03
#define    RMC 0x04
#define    VLW 0x0F
#define    VTG 0x05
#define    ZDA 0x08

#define R1005 0x05
#define R1077 0x4D
#define R1087 0x57
#define R1230 0xE6

class neoGPS
{
  
// -------------------------------------- Variable Types (pp 132) ---------------------------------------

  typedef  uint8_t U1;    // unsigned  8 bit integer
  typedef   int8_t I1;    //   signed  8 bit integer
  typedef uint16_t U2;    // unsigned 16 bit integer
  typedef  int16_t I2;    //   signed 16 bit integer
  typedef uint32_t U4;    // unsigned 32 bit integer
  typedef  int32_t I4;    //   signed 32 bit integer
  typedef  uint8_t X1;    //  1-byte bitfield
  typedef uint16_t X2;    //  2-byte bitfield
  typedef uint32_t X4;    //  4-byte bitfield
  typedef  uint8_t RU1_3; // eeebbbbb 8-bit float
  typedef    float R4;    // IEEE Float
  typedef   double R8;    // IEEE Double
  typedef     char CH;    // one byte character

// ------------------------------------ Public Variables & Methods -------------------------------------
 public:
    neoGPS(uint8_t addr = 0x42);
    void begin(uint32_t Rate);
 uint8_t poll();
    void configOdometer(uint8_t mode, bool withReset=false); 
    void enableMessage(uint8_t CL, uint8_t ID, uint8_t IsOn);
  
    bool verbose = false;
    int TIMEZONE_OFFSET = -5; // CST = -6, CDT = -5
    bool ackNak = false;
    
// --------------------  NAV-PVT --------------------

  union NAV_PVT {
    char raw[92];
    struct {
      U4 iTOW;      // ms
      U2 year;      // y
      U1 month;     // month
      U1 day;       // d
      U1 hour;      // h
      U1 min;       // min
      U1 sec;       // s
      X1 valid;     // bitfield 
      U4 tAcc;      // ns
      I4 nano;      // ns
      U1 fixType;   // -
      X1 flags;     // bitfield
      X1 flags2;    // bitfield
      U1 numSV;     // -
      I4 lon;       // deg / 1e-7
      I4 lat;       // deg / 1e-7
      I4 height;    // mm
      I4 hMSL;      // mm
      U4 hAcc;      // mm
      U4 vAcc;      // mm
      I4 velN;      // mm/s
      I4 velE;      // mm/s
      I4 velD;      // mm/s
      I4 gSpeed;    // mm/s
      I4 headMot;   // deg / 1e-5
      U4 sAcc;      // mm/s
      U4 headAcc;   // deg / 1e-5
      U2 pDOP;      // m/m / 0.01
      U1 reserved1; // -
      U1 reserved2; // -
      U1 reserved3; // -
      U1 reserved4; // -
      U1 reserved5; // -
      U1 reserved6; // -
      I4 headVeh;   // deg / 1e-5
      I2 magDec;    // deg / 1e-2
      U2 magAcc;    // deg / 1e-2
    } pcs;
  } nav_pvt;
  
    struct {
     U4 iTOW;            // ms
     U2 year;            // y
     U1 month;           // month
     U1 day;             // d
     U1 hour;            // h
     U1 min;             // min
     U1 sec;             // s
     R4 tOff;            // s
     R4 tAcc;            // ns
     
   bool validDate=0;     // 'valid' bit 0
   bool validTime=0;     // 'valid' bit 1
   bool fullyResolved=0; // 'valid' bit 2
   bool validMag=0;      // 'valid' bit 3
     U1 fixType;         // mask: 0-5
   bool gnssFixOK=0;     // 'flags' bit 0
   bool diffSoln=0;      // 'flags' bit 1
     U1 psmState;        // 'flags' bits 2-4
   bool headVehValid=0;  // 'flags' bit 5
     U1 carrSoln  ;      // 'flags' bits 6-7
   bool confirmedAvai=0; // 'flags2' bit 5
   bool confirmedDate=0; // 'flags2' bit 6
   bool confirmedTime=0; // 'flags2' bit 7
   
     U1 numSV;         // -
     R8 lon;           // deg
     R8 lat;           // deg
     R8 height;        // m
     R8 hMSL;          // m
     R4 hAcc;          // m
     R4 vAcc;          // m
     R4 velN;          // m/s
     R4 velE;          // m/s
     R4 velD;          // m/s
     R4 gSpeed;        // m/s
     R4 headMot;       // deg
     R4 sAcc;          // m/s
     R4 headAcc;       // deg
     R4 pDOP;          // m/m
     R4 headVeh;       // deg
     R4 magDec;        // deg
     R4 magAcc;        // deg

   bool isUpdated = false;
   } pvt;

// ------------------  NAV-SAT -------------------

   typedef struct {
        U1 gnssID;  // -
        U1 svID;    // -
        U1 cno;     // dBHz
        I1 elev;    // deg
        I2 azim;    // deg
        I2 prRes;   // 0.1 m
        X4 flags;   // -
   } sv;
         
   typedef struct {
        U1 gnssID;  // -
        U1 svID;    // -
        U1 cno;     // dBHz
        I1 elev;    // deg
        I2 azim;    // deg
        R4 prRes;   // m
        U1 quality; // 0 - no sig, 1 - searching, 2 - acquired, 3 - detected not used, 4 - code locked, 5/6/7 - carrier locked 
        bool svUsed;        // is used for navigation
        U1 health;          // 0 - unknown, 1 - healthy, 2- unhealthy
        bool diffCorr;      // diff. correction data is available
        bool smoothed;      // carrier smoothed prange used
        U1 orbitSource;     // 0 - no data, 1 eph used, 2 alm used, 3 ANO used, 4 ANA used, 5/6/7 other data used
        bool ephAvail;
        bool almAvail;
        bool anoAvail;
        bool aopAvail;
        bool sbasCorrUsed;
        bool rtcmCorrUsed;
        bool prCorrUsed;
        bool crCorrUsed;
        bool doCorrUsed; 
   } svDetail;
   
   union NAV_SAT {
      char raw[488];
      struct {
        U4 iTOW;       // ms
        U1 version;    // -
        U1 numSVs;     // -
        U1 reserved1;  // - 
        U1 reserved2;  // -
        sv svData[40]; // -
      } pcs;
   } nav_sat;

   struct {
        U4 iTOW;       // ms
        U1 version;    // -
        U1 numSVs;     // -
  svDetail svData[40]; // -
   bool isUpdated = false;
     
   } sat;

	union {
	  char raw[];
	  struct {
		 U1 header;  // alignment header
		 U1 ID;      // message ID
		 U1 flags;   // 3 bits fix type (0=NF, 1=DR only, 2=2D, 3=3D, 4=GNSS+DR, 5=Time only |  5 bits num SVs
		 U1 hAcc;    // horizontal accuracy (0.1m)
		 U4 iTOW;    // ms
		 I4 lon;     // longitude (1e-7 deg)
		 I4 lat;     // latitude (1e-7 deg)
		 I4 height;  // height above ellipsoid (mm)
	  } pcs;
	} DXa;

	union {
	  char raw[];
	  struct {
		 U1 header;  // alignment header
		 U1 ID;      // message ID
		 U2 year;    // year 
		 U1 month;   // month
		 U1 day;     // day
		 U1 hour;    // hour
		 U1 min;     // min
		 U2 sec;     // sec (ms)
		 I2 velN;    // North velocity (0.01 m/s)
		 I2 velE;    // North velocity (0.01 m/s)
		 I2 velD;    // North velocity (0.01 m/s)
		 I2 heading; // heading (0.001 deg)
		 U1 sAcc;    // speed accuracy (0.1 m)
		 U1 vAcc;    // vertical accuracy (0.1 m)
	  } pcs;
	} DXb;
   
   void updateDX();
   
// --------------------  NAV-ODO --------------------

   union NAV_ODO {
      char raw[20];
      struct {
		U1 version;       // -
		U1 reserved1;     // -
		U1 reserved2;     // -
		U1 reserved3;     // -
        U4 iTOW;          // ms
		U4 distance;      // m
		U4 totalDistance; // m
		U4 distanceStd;   // m
      } pcs;
    } nav_odo;
    
   struct {
        U1 version;       // -
		U4 iTOW;          // -
		U4 distance;      // m
		U4 totalDistance; // m
		U4 std;           // m

   bool isUpdated = false;
   } odo;
   
   
// ------------------------------------ Private Variables & Methods -------------------------------------
 private:
 
uint16_t _address = 0x42;
 uint8_t encode(char c);
 uint8_t parse(char *buffer);
    void configMessageRate(uint8_t CL, uint8_t ID, uint8_t IsOn);
    void configDDCPort(uint32_t Baud, bool UBX_in, bool NMEA_in, bool RTCM3_in, bool UBX_out, bool NMEA_out, bool RTCM3_out);
    void configDataRate(uint16_t Rate);
    void get(char Class, char ID);

 uint8_t waitForAckNack(uint32_t timeout=100);
    void ubxReplaceChecksum(char *message, int mLength);
    void ubxChecksumAndSend(char *message, int mLength);
    bool confirmChecksum(char *message, int mLength);
    
    void printCharArray(char message[], uint32_t mLength);
    
    void parse_NAV_PVT(NAV_PVT x);
    void parse_NAV_SAT(NAV_SAT x);
    void parse_NAV_ODO(NAV_ODO x);

};

#endif
