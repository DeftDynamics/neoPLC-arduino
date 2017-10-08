// neoPLC-ADC Demo

#include "neoTimer.h"

void timer_ISR(){
  Serial.println("interrupt");
}

// -------- Setup --------

void setup() {
  delay(2000);
  
  Timer.setPrescaler(4);
  Serial.print("max timer length is ");
  Serial.print(Timer.maxTime());
  Serial.println(" seconds");
  Serial.print("timer resolution is ");
  Serial.print(Timer.resolution());
  Serial.println(" microseconds");

  int y = doubler();
  
  //addTimer(timer_number [0-5],time (in micros), ISR function)
  Timer.addTimer(0,1000000,timer_ISR);
  Timer.addTimerMillis(1,2500,timer_ISR);
  Timer.start();
}

// -------- Loop --------

void loop() {
  Serial.println("looping...");
  delay(100);
}


