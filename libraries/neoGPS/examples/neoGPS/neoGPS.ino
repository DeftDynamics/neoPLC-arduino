// neoPLC-GPS Demo

/* 
 This is a library for communicating with uBlox GPS modules viaI2C.
 
 Based on "u-blox 8 / u-blox M8 Receiver Description" (UBX-13003221 - R13)
 It is not likely to work with devices older that protocol 20
 
 This demo uses Wire library. On all boards besides neoPLC, the default Wire buffer size is 16 bytes, and must be increased to 255
 for most messages, and as high as 512 for NAV-SAT messages. Polling faster will not help decrease the buffer
 Default address is 0x42, default rate is 1000 ms / solution.
 
 (8-7-17) The library is ~7.9 kBytes, but including Serial1 and/or Wire will take more space. 
*/


#include "neoGPS.h"

neoGPS gps = neoGPS();

bool led_state = true;
  
// ----------------------------------------- SETUP ----------------------------------------- 

void setup() {
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,led_state);
  
  delay(1000);
  Serial.print("neoGPS test\n");

  gps.verbose = false;
  gps.begin(1000); //  DDC: default DDC channel is 0x42, default data rate is 1000ms (= 1 Hz)
  
}

// ----------------------------------------- LOOP -------------------------------------------

void loop() {
  
// // all messages need this for parsing
   gps.poll();

// normal PVT solution
   if (gps.pvt.isUpdated == true){
      gps.pvt.isUpdated = false;
      printPVT();
   }

   
   if (gps.sat.isUpdated == true){
      gps.sat.isUpdated = false;
      printSAT();
   }
   
   
  led_state = !led_state;              
  digitalWrite(LED_BUILTIN,led_state);
  regulateLoop(0.1); // loop regulation (10 Hz)  
}


// ----------------------------------------- FUNCTIONS -------------------------------------------

void regulateLoop(float dt)
{
  // regulate loop until the end of the specified period (in seconds)
  static uint32_t prev_time = 0;
  uint32_t dT = int(dt*1000000);
  while((micros()-prev_time)<dT) {}
  prev_time = micros();
}

void printPVT(){
  /*
     Serial.printf("PVT flags\n");
     Serial.printf(" > Date = %d | Time = %d | Rslv = %d\n",gps.pvt.validDate,gps.pvt.validTime,gps.pvt.fullyResolved);
     Serial.printf(" >  PSM = %d |  Mag = %d | Hdng = %d\n",gps.pvt.psmState,gps.pvt.validMag,gps.pvt.headVehValid);
     Serial.printf(" > fxOK = %d |  Fix = %d | Diff = %d\n",gps.pvt.gnssFixOK,gps.pvt.fixType,gps.pvt.diffSoln);
     Serial.printf(" > Carrier Solution = %d\n",gps.pvt.carrSoln);
     
     Serial.printf("PVT data\n");
     Serial.printf(" >    date = %d/%d/%d\n",gps.pvt.year,gps.pvt.month,gps.pvt.day);
     Serial.printf(" >    time = %d:%d:%2.2f\n",gps.pvt.hour,gps.pvt.min,gps.pvt.sec+gps.pvt.tOff);
     Serial.printf(" >    tAcc = %d ns\n",int(gps.pvt.tAcc*1e9));
     
     Serial.printf(" >     lat = %2.8f deg\n",gps.pvt.lat);
     Serial.printf(" >     lon = %2.8f deg\n",gps.pvt.lon);
     Serial.printf(" >    hAcc = %1.3f m\n",gps.pvt.hAcc);
     Serial.printf(" >  height = %2.3f m\n",gps.pvt.height);
     Serial.printf(" >    hMSL = %2.3f m\n",gps.pvt.hMSL);
     Serial.printf(" >    vAcc = %1.3f m\n",gps.pvt.vAcc);
     
     Serial.printf(" >    velN = %1.3f m/s\n",gps.pvt.velN);
     Serial.printf(" >    velE = %1.3f m/s\n",gps.pvt.velE);
     Serial.printf(" >    velD = %1.3f m/s\n",gps.pvt.velD);
     Serial.printf(" >  gSpeed = %1.3f m/s\n",gps.pvt.gSpeed);
     Serial.printf(" >    sAcc = %1.3f m/s\n",gps.pvt.sAcc);
     
     Serial.printf(" > headMot = %1.3f deg\n",gps.pvt.headMot);
     Serial.printf(" > headVeh = %1.3f deg\n",gps.pvt.headVeh);
     Serial.printf(" > headAcc = %1.3f deg\n",gps.pvt.headAcc);
     
     Serial.printf(" >    pDOP = %1.3f\n",gps.pvt.pDOP);
     
     Serial.printf(" >  magDec = %1.3f deg\n",gps.pvt.magDec);
     Serial.printf(" >  magAcc = %1.3f deg\n\n",gps.pvt.magAcc);
     */
     
     Serial.print("Fix = ");
     Serial.println(gps.pvt.fixType);
     
     Serial.print(" >     lat = ");
     Serial.print(gps.pvt.lat,7);
     Serial.print(" deg\n");
     
     Serial.print(" >     lon = ");
     Serial.print(gps.pvt.lon,7);
     Serial.print(" deg\n\n");
     
}


void printSAT(){
/*
     Serial.printf("SAT data\n");
     Serial.printf(" >    numSVs = %d",gps.sat.numSVs);

     Serial.printf("\n >     GPS: ");
     for (int sv = 0; sv < gps.sat.numSVs; sv++){
        if (gps.sat.svData[sv].gnssID == 0) {
         Serial.printf(" %d,",gps.sat.svData[sv].svID);
        }
     }
     Serial.printf("\n >    SBAS: ");
     for (int sv = 0; sv < gps.sat.numSVs; sv++){
        if (gps.sat.svData[sv].gnssID == 1) {
         Serial.printf(" %d,",gps.sat.svData[sv].svID);
        }
     }
     Serial.printf("\n > GLONASS: ");
     for (int sv = 0; sv < gps.sat.numSVs; sv++){
        if (gps.sat.svData[sv].gnssID == 6) {
         Serial.printf(" %d,",gps.sat.svData[sv].svID);
        }
     }
     Serial.println();
     
     Serial.printf("Individual SV data demo\n");
     uint8_t sv = 0;
     if (gps.sat.numSVs >= sv){
       Serial.printf(" >  gnssID = %2d |  svID = %d\n",gps.sat.svData[sv].gnssID,gps.sat.svData[sv].svID);
       Serial.printf(" >     cno = %2d | prRes = %1.1f\n",gps.sat.svData[sv].cno,gps.sat.svData[sv].prRes);
       Serial.printf(" >    elev = %2d |  azim = %d\n",gps.sat.svData[sv].elev,gps.sat.svData[sv].azim);
       Serial.printf(" > quality = %2d |  used = %d\n\n",gps.sat.svData[sv].quality,gps.sat.svData[sv].svUsed);
     }
     */
}

