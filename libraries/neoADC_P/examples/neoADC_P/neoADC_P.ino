// neoPLC-ADC Demo

#include "neoADC_P.h"
#include <Wire.h>

neoADC_P adc = neoADC_P();
bool led_state = true;


    float FSR = 0.0;
     int RATE = 0.0;


// -------- Setup --------

void setup()
{
  
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,led_state);
  delay(2000);
  
  Serial.println("neoADC-P test");

  // input: 
  // > max expected range (+/- for differential)
  // > sample rate (8,16,32,...,860)
  // > mode (CONTINUOUS (faster if using a single channel), SINGLESHOT (low power)) 
  // WARNING - Set expected range carefully. Exceeding abs(RANGE+0.3) will damage the ADC
  adc.begin(3.3, 860, CONTINUOUS);

  adc.readConfiguration();
}

// -------- Loop --------

void loop()
{

  // single ended
  for(int i=0; i<4; i++){
    float voltage = adc.read(i);
    Serial.printf("                 read channel %d single-ended: %2.6f\n",i,voltage);
  }

  // differential
  float voltage01 = adc.read(0,1);
  Serial.printf("read the difference between channels 0 and 1: %2.4f mV\n",voltage01*1000);
  float voltage23 = adc.read(2,3);
  Serial.printf("read the difference between channels 2 and 3: %2.4f mV\n\n",voltage23*1000);
  
  led_state = !led_state;              
  digitalWrite(LED_BUILTIN,led_state);
  regulateLoop(1.0); // loop regulation (10 Hz)  
  
}

// -------- Functions --------

void regulateLoop(float dt)
{
  // regulate loop until the end of the specified period (in seconds)
  static uint32_t prev_time = 0;
  uint32_t dT = int(dt*1000000);
  while((micros()-prev_time)<dT) {}
  prev_time = micros();
}







