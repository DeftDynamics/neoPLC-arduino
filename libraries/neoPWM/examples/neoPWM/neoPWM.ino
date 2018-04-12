// neoPLC-PWM Demo

#include <neoPWM.h>
#include <neoBLE.h>

neoPWM pwm = neoPWM();

bool led_state = true;

// -------- Setup --------

void setup()
{
  BLE.begin();
  
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,led_state);
  
  delay(2000);
  Serial.println("neoPWM Test");

  pwm.begin(1600); // maximum frequency is 1600 Hz

}

// -------- Loop --------

void loop() {
  
  float duty = 0.5+0.5*sin(millis()/1000.0);
  
  Serial.print("set all pins to ");
  Serial.print(duty*100,0);
  Serial.println("%");
  
  for (int i = 0; i<8; i++){
    pwm.analogWrite(i,duty*4095);
  }

  BLE.post(pwm.DX.raw); // standard DX message for streaming this sensor over BLE to neoPLC apps
  
  led_state = !led_state;              
  digitalWrite(LED_BUILTIN,led_state);
  regulateLoop(0.5); // loop regulation (10 Hz)  
  
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

void shutdownISR(){
  // this function is called when the program shuts down (like for re-programming)
  // turn off all output to be safe
  pwm.zeroAll();
}

