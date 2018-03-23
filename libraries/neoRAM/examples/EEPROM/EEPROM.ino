// neoPLC EEPROM Test
#include "neoRAM.h"

neoRAM ram = neoRAM();

// create the data structure type that will be logged at each sample time:
// this union of a char array and structure allows us to more easily manipulate the data. 
// modify the contents of the structure for the data you plan to store. 
// be aware that the size of the structure may exceed the sum of the structure value sizes due to alignment
typedef union dataStruct {
       char raw[]; 
  struct {
   uint32_t w;
      float x;
   uint16_t y;
    uint8_t z;
       bool q;
   uint32_t time;
  } pcs;
};
dataStruct writeData; // an instance of the data structure that we will modify and save
dataStruct readData;  // an instance of the data structure to save values to 
uint32_t dataStructSize = sizeof(writeData);


// ------------------------ SETUP ---------------------------


void setup() {
  
  Serial.begin(115200);
  delay(2000);

  ram.begin(dataStructSize); // initialization only needs the size of the data structure that will be logged

  // put some generic numbers in the structure we will be writing just so it's interesting
  writeData.pcs.w = 0;
  writeData.pcs.x = 123.45;
  writeData.pcs.y = 1;
  writeData.pcs.z = 254;
  writeData.pcs.q = true;
  writeData.pcs.time = millis();
  
  ram.restartLog(); // the log currently restarts automatically with the first write 
                    // this function resets writes to the initial index and writes over existing data
  
}

// ------------------------ LOOP ---------------------------

void loop() {

  int samplesToWrite = 10;

  // append a sample to the log
  for (int i=0; i<samplesToWrite; i++){
    writeData.pcs.w = writeData.pcs.w+1;
    writeData.pcs.time = millis();
    bool success = ram.addLog(writeData.raw);
    if (!success){
      Serial.println("\n*** ERROR: Tried to write beyond memory limit ***");
      while(1==1){}
    }
  }

  
  // read total sample count
  Serial.printf("%d total samples have been logged\n",ram.logSize());


  // confirm what we wrote
  for (int i=0; i<samplesToWrite; i++){
    uint8_t offset = samplesToWrite-i;
    bool success = ram.readLog(ram.logSize()-offset,readData.raw);
    if (!success){
      Serial.println("\n*** ERROR: Tried to read beyond memory limit ***");
    }
  }


  Serial.printf("read sample %d @ 0x%2X: w = %d, raw = ",ram.logSize(),ram.logSize()*dataStructSize,readData.pcs.w);
  for (int i=0; i<dataStructSize; i++){
    Serial.printf(" %2X",readData.raw[i]);
  }
  Serial.println("\n");


  delay(500);
  
}


