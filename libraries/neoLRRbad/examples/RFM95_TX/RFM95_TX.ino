// neoPLC LoRa (LongRange) Radio Test
// ~~~ TX TRANSMITTER ~~~
// (RX and TX test scripts are almost the same,
// but the device running TX starts the radio conversation)

#include <neoLRR.h>

neoLRR lrr = neoLRR();

int CS = 10;
int16_t packetnum = 0;  // packet counter, we increment per xmission
elapsedMillis tic = 0;

bool useAck = false; // use ACK system (BOTH radios must have the same setting)

// -------------------- SETUP -----------------------
void setup() {
  Serial.begin(115200);
  delay(2000);
  
  lrr.setFromAddress(0x00); // address of this radio
  lrr.setToAddress(0x01);   // address of the other radio
  lrr.reliableData(useAck); // use ACK system (BOTH radios must have the same setting)
  lrr.begin(CS, 17, 1);   // begin with 'chipSelect pin', 'transmit power', 'rxStandby'
}

// -------------------- LOOP -----------------------
void loop() {

  packetnum++;
  String packet = "Hello World #" + String(packetnum);
  Serial.print("Send: '");
  Serial.print(packet);
  Serial.print("' ... ");
  tic = 0;
  
  bool success = lrr.send(packet);
  if (useAck){
    if (success){
      Serial.println("Receipt acknowledged");  
    } else {
      Serial.println("Receipt not acknowledged. Try again");
    }
  } else {
    Serial.print("Wait for packet to be released... ");
    lrr.waitForDispatch();
    Serial.println("complete");
  }
  
  // Now wait for a reply
  Serial.print("Wait for reply... ");
  lrr.waitForMessage();
  if (lrr.available()) {
    uint32_t toc = (uint32_t)tic;
    Serial.printf("received: '");
    while (lrr.available()) {
      char c = lrr.read();
      Serial.print(c);
    }
    Serial.printf("'\n[trip: %d ms][RSSI: %d][SNR: %2.2f]\n",toc, lrr.lastRssi, lrr.lastSNR);
  }
  else
  {
    Serial.println("No reply");
  }

  Serial.println();
  delay(1000);
}



