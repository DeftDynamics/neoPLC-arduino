#include <neoBLE.h> 

int led = LED_BUILTIN;
bool led_state = LOW;

//Pointer to the structure holding our service
neoBLE_service* myService;
//Pointer to the structure holding our characterisitc
neoBLE_characteristic* myCharacteristic;

//A custom UUID for our our package. This is not required, it will default to a "neoPLC UUID" thats basically just a random string so as to not clash with other specific UUIDs
//const ble_uuid128_t MY_UUID ={ { 0xEF, 0xBE, 0xEF, 0xBE, 0xEF, 0xBE, 0xEF, 0xBE, 0xEF, 0xBE, 0xEF, 0xBE, 0x00, 0x00, 0xEF, 0xBE } };

//The actual UUIDs of our service and characteristic, which is combined with the above UUID for the actual BLE protocol
const uint16_t      MY_SERVICE_UUID = 0x0001;
const uint16_t      MY_CHAR_UUID = 0x0002;
int counter = 0;

void setup() {
  pinMode(led, OUTPUT);           // initialize the digital pin as an output.
  digitalWrite(led, led_state);   // turn the LED on
  
  BLE.begin();                  // begin the BLE softdevice REQUIRED
  BLE.setDeviceName("neoPLC");  // change the blutooth BLE device name
  
  //add a service that uses our custom package UUID
  //myService = BLE.add_service(MY_UUID, MY_SERVICE_UUID); //this function returns a pointer to a neoBLE_service object
  //add a service that uses the default neoPLC package UUID
  myService = BLE.addService(MY_SERVICE_UUID); //this function returns a pointer to a neoBLE_service object
  
  //add a characteristic to that service 
  myCharacteristic = myService->addCharacteristic(MY_CHAR_UUID);  //this function returns a pointer to a neoBLE_characteristic object
  
  //(optional) set the descriptor of our characteristic the UUID for this is based on the BLE spec: https://www.bluetooth.com/specifications/gatt/descriptors
  myCharacteristic->addDescriptor(0x2901,"Custom Chara");
  
  //set a value
  myCharacteristic->setValue("Hello World");
}

void loop() {
  if (!BLE.connected()){
    led_state = !led_state;
  }else{
    led_state = HIGH;
  }
  digitalWrite(led, led_state); // turn the LED on/off
  //update the value of the characteristic
  myCharacteristic->setValue(counter);
  counter++;
  delay(1000);
}
