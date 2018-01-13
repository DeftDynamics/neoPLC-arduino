// neoPLC Scanner - scans all connected i2c devices and reports the name and address of known neoPLC boards
// adapted from http://playground.arduino.cc/Main/I2cScanner

#include <Wire.h>
int led_pin = 13;

void setup() {
  Wire.begin();
  pinMode(led_pin,OUTPUT);
  Serial.begin(9600);
  while (!Serial);
  Serial.println("\nneoPLC i2c Scanner\n");
}


void loop() {
  byte error, address;
  int nDevices;

  digitalWrite(led_pin,HIGH);
  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    byte x = 0x00;
    Wire.write(x);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address,HEX);
      Serial.print("  (");
      printKnownChips(address);
      Serial.println(")");

      nDevices++;
    } else if (error==4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("done\n");
  }
  digitalWrite(led_pin,LOW);
  
  delay(1000);           // wait 5 seconds for next scan
}


void printKnownChips(byte address)
{
  switch (address) {
    case 0x42: Serial.print("neoPLC-GPS Global Positioning System"); break;
    case 0x13: Serial.print("neoPLC-IMU Inertial Measurement Unit: Magnetometer"); break;
    case 0x19: Serial.print("neoPLC-IMU Inertial Measurement Unit: Accelerometer"); break;
    case 0x27: Serial.print("neoPLC-DIO Digital Input/Output"); break;
    case 0x3C: Serial.print("neoPLC-OLED 64x48 Display"); break;
    case 0x43: Serial.print("neoPLC-PWM Pulse Width Output (Single Board)"); break;
    case 0x48: Serial.print("neoPLC-ADC Analog Input"); break;
    case 0x69: Serial.print("neoPLC-IMU Inertial Measurement Unit: Gyroscope"); break;
    case 0x70: Serial.print("neoPLC-PWM Pulse Width Output (All Call)"); break;
    case 0x77: Serial.print("neoPLC-ENV Environmental Sensor (Pressure/Temperature/Humidity)"); break;
    default: Serial.print(F("not recognized: neoPLC with adjusted address?"));
  }
}