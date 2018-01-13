// neoPLC-IMU Demo

#include <neoIMU.h>       // Bosch BMX055 setup and parse

neoIMU imu = neoIMU();

float ax, ay, az, gx, gy, gz, mx, my, mz;       // variables to hold latest sensor data values 

uint32_t loop_time = 0;
uint32_t math_time = 0;
uint32_t dt = 0.1; // loop time in seconds ( = 1/Hz)

bool led_state = HIGH;

void setup() 
{

  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,led_state);
  delay(2000);
  
    /* potential values for setup:
  Accelerometer Range:
    2, 4, 8, 16 (Gs) | default: 2
  Accelerometer Bandwidth:
    8, 16, 32, 64, 125, 250, 500, 1000 (Hz) | default: 125
  Gyroscope Range:
    125, 250, 500, 1000, 2000 (deg/s) | default: 250
  Gyroscope Bandwidth:
    12, 23, 32, 47, 64, 116, 230, 523 (Hz) | default: 116
  Magnetometer Output Rate:
    2, 6, 8, 10, 15, 20, 25, 30 (Hz) | default: 30
  */

  Serial.print("Initialize IMU, set scale and speed... ");
  bool success = imu.begin(4,64,250,23,30); // Acc. Range & BW, Gyro Range & BW, Mag. output rate

  if (success)
  {Serial.println("IMU is online");}
  else
  { Serial.print("Could not connect to neoPLC-IMU"); while(1){}}

  regulateLoop(0.0);
}

    
void loop() 
{
 
  //imu.pollAccel(); // update acceleration in Gs IF new data is available
  //imu.pollGyro(); // update rotation rate in deg/s 
  //imu.pollMag(); // update magnetic flux in mGauss IF new data is available
  
  imu.pollAll(); // update all sensor values

  Serial.print("Die temperature = "); Serial.print(imu.readTemp()); Serial.println(" deg C");
  Serial.print("ax = ");     Serial.print(imu.ax,4);  
  Serial.print("\tay = ");   Serial.print(imu.ay,4); 
  Serial.print("\taz = ");   Serial.print(imu.az,4); Serial.println(" g");
  Serial.print("gx = ");     Serial.print(imu.gx,4);  
  Serial.print("\tgy = ");   Serial.print(imu.gy,4); 
  Serial.print("\tgz = ");   Serial.print(imu.gz,4); Serial.println(" deg/s");
  Serial.print("mx = ");     Serial.print(imu.mx,4);  
  Serial.print("\tmy = ");   Serial.print(imu.my,4); 
  Serial.print("\tmz = ");   Serial.print(imu.mz,4); Serial.println(" mG\n");
    
  led_state = !led_state;              
  digitalWrite(LED_BUILTIN,led_state);
  
  regulateLoop(0.02); // loop regulation (50 Hz)  
}

void regulateLoop(float dt)
{
  // regulate loop until the end of the specified period (in seconds)
  static uint32_t prev_time = 0;
  uint32_t dT = int(dt*1000000);
  while((micros()-prev_time)<dT) {}
  prev_time = micros();
}
