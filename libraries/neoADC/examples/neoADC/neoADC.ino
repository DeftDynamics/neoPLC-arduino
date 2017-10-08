// neoPLC-ADC Demo

#include "neoADC.h"
#include <Wire.h>

neoADC adc = neoADC();

bool led_state = true;

// -------- Setup --------

void setup()
{
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,led_state);
  delay(2000);
  
  Serial.println("neoADC test");
  adc.begin();
}

// -------- Loop --------

void loop()
{
  
  float voltage = 0;
  
  for(int x = 0; x < 8; x++){
    voltage = adc.read(x);
    Serial.print("Channel "); Serial.print(x); 
    Serial.print(" = ");
    Serial.print(voltage,4); Serial.println(" V");
  }
  Serial.println();
  
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



