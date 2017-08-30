#include "Arduino.h"
#include <stdint.h>
#include <stdbool.h>
#include <neoBLE.h>

int led = PIN_LED1;
int counter = 0;
bool led_state = LOW;

uint16_t beef_service;
uint16_t beef_value;

void setup() {       
  pinMode(led, OUTPUT);  
  BLE.begin();
  beef_service = BLE.add_service(0xBEEF);
  beef_value = BLE.add_characteristic(beef_service, 0xDEAD);
  BLE.set_value(beef_value, "test");
  
  BLE.start_advertizing();
}

void loop() 
{
  Serial.println("ping");
  counter += 1;
  BLE.set_value(beef_value, counter);
  while(Serial.available() > 0){
    Serial.println(Serial.read());
  }
  
  if(led_state == LOW){
    digitalWrite(led, HIGH);
    led_state = HIGH;
  }
  else
  {
    digitalWrite(led, LOW);
    led_state = LOW;
  }
  
  delay(100);
}



