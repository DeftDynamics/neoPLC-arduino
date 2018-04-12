// neoPLC-ENV Demo

#include <neoENV.h>
#include <neoBLE.h>

neoENV env = neoENV();


bool led_state = true;

// -------- Setup --------

void setup() {
  
  BLE.begin();

  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,led_state);

  delay(2000);
  Serial.println("neoPLC-ENV test...");
  
  bool success = env.begin();
  if (success) {
    Serial.println("Successfully connected\n");
  } else {
    Serial.println("Could not connect");
    while (1==1){};
  }

  regulateLoop(1.0);
}

// -------- Loop --------

void loop() { 
  float temp = env.readTemperature();
    Serial.print("Temperature = "); Serial.print(temp); Serial.println(" *C");
  float humi = env.readHumidity();
    Serial.print("   Humidity = "); Serial.print(humi); Serial.println(" %");
  float pres = env.readPressure();
    Serial.print("   Pressure = "); Serial.print(pres); Serial.println(" Pa");
  float estAlt = env.readAltitude();
    Serial.print("  ~Altitude = "); Serial.print(estAlt); Serial.println(" m\n");

  BLE.post(env.DX.raw); // standard DX message for streaming this sensor over BLE to neoPLC apps

  led_state = !led_state;              
  digitalWrite(LED_BUILTIN,led_state);
  regulateLoop(1.0);
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
