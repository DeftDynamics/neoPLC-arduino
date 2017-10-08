// neoBLE Demo

#include <neoBLE.h>

int all_parsed = true;
bool led_state = HIGH;
int led = LED_BUILTIN;
unsigned int tic = 0;

uint8_t my_byte = 0;

void setup() {
  pinMode(led, OUTPUT);               // initialize the digital pin as an output.
  digitalWrite(led, led_state);   // turn the LED on
 
  delay(2000);
  Serial.println("*** neoBLE Demo ***\n");
  Serial.println("The neoPLC LED will blink until you connect to a phone.");
  Serial.println("We recommend our demo app 'neoPLC' from the iOS app store or on our GitHub.\n");
  
  
  BLE.begin();                  // begin the BLE softdevice
  BLE.setDeviceName("neoPLC"); // change the blutooth BLE device name
  
  waitForConnection();  // blink the LED until the phone is connected
  
}

void loop() {

  bleManager();                 // check incoming messages and parse
  digitalWrite(led, led_state); // turn the LED on/off as led_state is read from the phone

  if (!BLE.connected()){
    waitForConnection();
  }
  
  delay(100);

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
}


