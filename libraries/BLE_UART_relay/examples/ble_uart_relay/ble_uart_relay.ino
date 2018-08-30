#include <neoBLE.h>
#include <neoDFU.h>
#include <SoftwareSerial.h>

SoftwareSerial usbSerial(8,7);

void setup() {
  // put your setup code here, to run once:
  neoDFU_init();
  usbSerial.begin(115200);
  delay(100);

  BLE.begin();
  Serial.core_begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  
  int q = usbSerial.available();
  if(q >= 3){
    if(usbSerial.read() == 36 && usbSerial.read() == 38 && usbSerial.read() == 36){
      usbSerial.end();
      delay(100);
      sd_nvic_SetPendingIRQ(boot_irq_n);
    }
  }
}
