// *** neoPLC Universal Sensor Demonstation ***

// This script finds and connects to all neoPLC board, this demo is designed to be used with the neoPLC app:
// It streams basic sensor data using the neoPLC DX messaging system, the app displays it in the interactive data-stream table. 

// Most functionality in this script is copied from the stand-alone demos that can be found in File-Examples
// Because of the complexity and multiple boards used at once, this script is a good test for the Stream table but is
// probably not a great place to start exploring the embedded side of neoPLC - check out the individual examples
// for your boards instead - they will run faster and the amount of code required will be more clear.

// ------------------- INCLUDE ALL NEEDED LIBRARIES ---------------------
#include <Wire.h>
#include <neoBLE.h>
#include <neoGPS.h>
#include <neoIMU.h>
#include <neoENV.h>
#include <neoBAT.h>
#include <neoOLED.h>
#include <neoADC.h>
#include <neoDIO.h>
#include <neoPWM.h>
#include <neoTCA.h>

neoGPS gps = neoGPS();
neoIMU imu = neoIMU();
neoENV env = neoENV();
neoBAT bat = neoBAT();
neoOLED oled = neoOLED();
neoADC adc = neoADC();
neoDIO dio = neoDIO();
neoPWM pwm = neoPWM();
neoTCA tca = neoTCA();

// ------------------------- INITIALIZATIONS ----------------------------

int all_parsed = true;
bool led_state = HIGH;
int oled_state = 0;
int led = LED_BUILTIN;
unsigned int tic = 0;

uint8_t my_byte = 0;

//                         GPS,  IMU,  ENV,  BAT, OLED,  ADC,  DIO,  PWM,  TCA,
uint8_t  addressList[] = {0x42, 0x13, 0x77, 0x36, 0x3C, 0x48, 0x27, 0x43, 0x28};
bool connectedBoards[] = {   0,    0,    0,    0,    0,    0,    0,    0,    0};
int boardCount = 9;

// --------------------------- SETUP BLE --------------------------------
void setup() {
  
  Wire.begin();
  BLE.begin();                  // begin the BLE softdevice
  BLE.setDeviceName("neoPLC"); // change the blutooth BLE device name
  
  pinMode(led, OUTPUT);               // initialize the digital pin as an output.
  digitalWrite(led, led_state);   // turn the LED on
 
  delay(2000);
  Serial.println("*** neoBLE Demo ***\n");
  Serial.println("The neoPLC LED will blink until you connect to a phone.");
  Serial.println("This example works with our demo app 'neoPLC' from the iOS App Store or our GitHub.\n");
  
  waitForConnection();  // blink the LED until the phone is connected

}



// --------------------------- MAIN LOOP ---------------------------------
void loop() {

// ~~~~~~~~ Wait For Connection ~~~~~~~~~~

  if (!BLE.connected()){
    waitForConnection();
  }

// ~~~~~~~~ Identify a newly connected neoPLC boards ~~~~~~~~~~
  
  // scan and begin any newfound boards

  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte x = 0x00;
    Wire.write(x);
    byte error = Wire.endTransmission();
    if (error == 0) {
      // if we find a board at this address, connect to it unless we already did
      if ((address == addressList[0]) && (connectedBoards[0]==false)){
        gps.begin(1000);
        connectedBoards[0] = true;
      } else if ((address == addressList[1]) && (connectedBoards[1]==false)){
        imu.begin(4,16,250,12,20);
        connectedBoards[1] = true;
      } else if ((address == addressList[2]) && (connectedBoards[2]==false)){
        env.begin();
        connectedBoards[2] = true;
      } else if ((address == addressList[3]) && (connectedBoards[3]==false)){
        bat.begin();
        connectedBoards[3] = true;
      } else if ((address == addressList[4]) && (connectedBoards[4]==false)){
        oled.begin();
        oled.setFont(System5x7);
        oled.clear();
        oled.println("neoPLC");
        oled.println("app + hw");  
        oled.println("ecosystem"); 
        connectedBoards[4] = true;
      } else if ((address == addressList[5]) && (connectedBoards[5]==false)){
        adc.begin();
        connectedBoards[5] = true;
      } else if ((address == addressList[6]) && (connectedBoards[6]==false)){
        dio.begin();
        dio.pinMode(x, INPUT); // set mode to input
        dio.pullUp(x, HIGH);   // turn on internal 100K pullup
        connectedBoards[6] = true;
      } else if ((address == addressList[7]) && (connectedBoards[7]==false)){
        pwm.begin(1600);
        connectedBoards[7] = true;
      } else if ((address == addressList[8]) && (connectedBoards[8]==false)){
        tca.begin();
        connectedBoards[8] = true;
      } 
    } else {
      // if we don't find a board at this address, mark it as disconnected
      for (int i=0; i<boardCount; i++){
        if (address == addressList[i]){
          connectedBoards[i] = false;
        }
      }
    }
  }

// ~~~~~~~~ Stream data from connected neoPLC boards ~~~~~~~~~~

  if (connectedBoards[0] == true) {
    Serial.println("*** GPS ***");
    gps.poll();
    if (gps.pvt.isUpdated == true){
      gps.pvt.isUpdated = false;
      BLE.post(gps.DXa.raw); // standard DX message for streaming this sensor over BLE to neoPLC apps
      BLE.post(gps.DXb.raw); // because the GPS has diverse data, two messages are needed (DXa and DXb)
      Serial.printf("Fix = %d\n",gps.pvt.fixType);
      Serial.printf(" >     lon = %2.7f deg\n",gps.pvt.lon);
      Serial.printf(" >     lat = %2.7f deg\n",gps.pvt.lat);
      Serial.printf(" >  height = %2.3f m\n\n",gps.pvt.height);
    }
  }
  if (connectedBoards[1] == true) {
    Serial.println("*** IMU ***");
    imu.pollAll(); // update all sensor values
    BLE.post(imu.DX.raw); // standard DX message for streaming this sensor over BLE to neoPLC apps
    Serial.print("Die temperature = "); Serial.print(imu.readTemp()); Serial.println(" deg C");
    Serial.print("ax = ");     Serial.print(imu.ax,4);  
    Serial.print("\tay = ");   Serial.print(imu.ay,4); 
    Serial.print("\taz = ");   Serial.print(imu.az,4); Serial.println(" g");
    Serial.print("gx = ");     Serial.print(imu.gx,4);  
    Serial.print("\tgy = ");   Serial.print(imu.gy,4); 
    Serial.print("\tgz = ");   Serial.print(imu.gz,4); Serial.println(" deg/s");
    Serial.print("mx = ");     Serial.print(imu.mx,4);  
    Serial.print("\tmy = ");   Serial.print(imu.my,4); 
    Serial.print("\tmz = ");   Serial.print(imu.mz,4); Serial.println(" mG\n\n");
  }
  if (connectedBoards[2] == true) {
    Serial.println("*** ENV ***");
    float temp = env.readTemperature();
    float humi = env.readHumidity();
    float pres = env.readPressure();
    float estAlt = env.readAltitude();
    BLE.post(env.DX.raw); // standard DX message for streaming this sensor over BLE to neoPLC apps
      Serial.print("Temperature = "); Serial.print(temp); Serial.println(" *C");
      Serial.print("   Humidity = "); Serial.print(humi); Serial.println(" %");
      Serial.print("   Pressure = "); Serial.print(pres); Serial.println(" Pa");
      Serial.print("  ~Altitude = "); Serial.print(estAlt); Serial.println(" m\n\n");
  }
  if (connectedBoards[3] == true) {
    Serial.println("*** BAT ***");
    bat.poll();
    Serial.printf("vcell = %2.4f, SoC = %2.2f%%\n\n",bat.vcell,bat.soc);
    BLE.post(bat.DX.raw);
  }
  if (connectedBoards[4] == true) {
    Serial.println("*** OLED ***");
    Serial.println("printing the current time to the screen...\n\n");
    oled.setRow(4);
    
    if (oled_state==0){
      oled.clearToEOL();
      oled.println("Custom IoT!");
    } else if (oled_state==8) {
      oled.clearToEOL();
      oled.println("neoplc.org");
    }
    oled_state++;
    oled_state = oled_state % 16;
    
    //oled.clearToEOL();
    //oled.print("  ");
    //oled.println((float)millis()/1000.0);
    for (int i=0; i<24; i++){
        oled.updateDX(i);
        BLE.post(oled.DX.raw);
        delay(5);
    }
  }
  if (connectedBoards[5] == true) {
    Serial.println("*** ADC ***");
    for(int x = 0; x < 8; x++){
      float voltage = adc.read(x);
      Serial.printf("pin %d = %2.4f V\n",x,voltage);
    }
    Serial.print("\n");
    BLE.post(adc.DX.raw);
  }
  if (connectedBoards[6] == true) {
    // read all channels
    Serial.println("*** DIO ***");
    for(int x = 0; x < 8; x++){
      bool state = dio.digitalRead(x);
      Serial.printf("pin %d = %d\n",x,state);
    }
    Serial.print("\n");
    BLE.post(dio.DX.raw);
  }
  if (connectedBoards[7] == true) {
    float duty = 0;
    Serial.println("*** PWM ***");
    Serial.printf("set all pins to %1.0f%%\n\n",duty*100);
    for (int i = 0; i<8; i++){
      pwm.analogWrite(i,duty*4095);
    }
    BLE.post(pwm.DX.raw);
  }
  if (connectedBoards[8] == true) {
    float temperature = tca.read();
    Serial.println("*** TCA ***");
    Serial.printf("Temp = %2.3f, refTemp = %2.3f\n\n",tca.temp,tca.refTemp);
    BLE.post(tca.DX.raw);
  }
  
  delay(100);
}




// ------------------------ CUSTOM SCRIPT FUNCTIONS ---------------------

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


