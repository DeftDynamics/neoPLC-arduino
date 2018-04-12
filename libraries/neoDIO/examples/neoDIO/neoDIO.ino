// neoPLC-DIO Demo

#include <neoDIO.h>
#include <neoBLE.h>

neoDIO dio = neoDIO();

bool led_state = LOW;

int mode = 0; // mode = 0 is input test  - test by connecting buttons or switches to ground
              // mode = 1 is output test - test by connecting LEDs with pull-up resistor (>330 Ohm!) to V+  
  
// -------- Setup --------

void setup() {  
  pinMode(LED_BUILTIN, OUTPUT);
  delay(2000);
  Serial.println("neoDIO test");

  BLE.begin();
  
  dio.begin();

  if (mode == 0){
    for(int x = 0; x < 8; x++){
      dio.pinMode(x, INPUT); // set mode to input
      dio.pullUp(x, HIGH);   // turn on internal 100K pullup
    }
  } else {
    for(int x = 0; x < 8; x++){
      dio.pinMode(x, OUTPUT); // set mode to output
    }
  }

  regulateLoop(1.0);
}

// -------- Loop --------

void loop() {

  if (mode == 0){
    // read all channels
    for(int x = 0; x < 8; x++){
      bool state = dio.digitalRead(x);
      Serial.print("Channel "); Serial.print(x); 
      Serial.print(" = ");
      Serial.println(state);
    }
    Serial.println();
  } else {
    // write all channels
    Serial.print("\nAll channels = "); Serial.println(led_state);
    for(int x = 0; x < 8; x++){
      dio.digitalWrite(x,!led_state); // low = LED on since the LEDS are wired to power
    }
  }

  BLE.post(dio.DX.raw); // standard DX message for streaming this sensor over BLE to neoPLC apps
  
  digitalWrite(LED_BUILTIN,led_state);
  led_state = !led_state;              
  regulateLoop(1.0); // loop regulation (1 Hz)  
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