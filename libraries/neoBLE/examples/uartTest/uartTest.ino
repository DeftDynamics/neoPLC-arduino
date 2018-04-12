// neoBLE Demo

#include <neoBLE.h>

bool led_state = HIGH;
int led = LED_BUILTIN;
unsigned int tic = 0;

void setup() {
  
  pinMode(led, OUTPUT);               // initialize the digital pin as an output.
  digitalWrite(led, led_state);   // turn the LED on
 
  delay(2000);
  Serial.println("*** neoBLE UART Test ***\n");
  
  
  BLE.begin();                  // begin the BLE softdevice
  BLE.setDeviceName("neoPLC"); // change the blutooth BLE device name
  
}

void loop() {

  if (!BLE.connected()){
    waitForConnection();
  }

  if (BLE.available()){
    while (BLE.available()){
      char c = BLE.read();
      Serial.print(c);
    }
  }

 if (Serial.available()){
   //Serial.print("TX: ");
   while (Serial.available()){
     char c = Serial.read();
     BLE.print(c);
     //Serial.print(c);
   }
   BLE.flush();
 }

  regulateLoop(0.1);
  
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

void waitForConnection(){
  while (!BLE.connected())
  {
    if ((millis()-tic) > 500)
    {
      led_state = !led_state;
      digitalWrite(led,led_state);
      tic = millis();
    }
  }
  led_state = HIGH;
  digitalWrite(led,led_state);
}


