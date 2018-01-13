// neoPLC LoRa (LongRange) Radio Test
// ~~~ RX RECEIVER ~~~
// (RX and TX test scripts are almost the same, 
// but the device running TX starts the radio conversation)

#include <neoLRR.h>

neoLRR lrr = neoLRR();

int CS = 10;
uint8_t penPalAddress = 0x00;   // address of the radio we want to speak to
                                // 0xff will broadcast to all radios in range

bool useAck = false; // use ACK system (BOTH radios must have the same setting)

// -------------------- SETUP -----------------------
void setup() {
  Serial.begin(115200);
  delay(2000);
  
  lrr.setFromAddress(0x01); // address of this radio
  lrr.setToAddress(0x00);   // address of the other radio
  lrr.reliableData(useAck); // use ACK system (BOTH radios must have the same setting)
  lrr.begin(CS, 17, 1);   // begin with 'chipSelect pin', 'transmit power', 'rxStandby'
}

// -------------------- LOOP -----------------------
void loop() {

  //lrr.waitForMessage();
  lrr.poll();

  
  if (lrr.available()) {
    
    Serial.print("\nReceived: '");
    while (lrr.available()) {
      char c = lrr.read();
      Serial.print(c);
    }
    Serial.printf("' [RSSI: %d][SNR: %2.2f]\n", lrr.lastRssi, lrr.lastSNR);

    // Send a reply
    Serial.print("Send a reply: '");
    char data[] = "And hello back to you";
    Serial.print(data);
    Serial.print("' ... ");
    String Data = data;
    bool success = lrr.send(Data);
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

  } 

  delay(100);
}



