/***************************************************************************
  nePLC-LRR Library

  A library for RFM95 by Hoperf Electronic

  Copyright (c) 2017, Deft Dynamics
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
  1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
  3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:
   This product includes software developed by Deft Dynamics.
  4. Neither the name of Deft Dynamics nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY DEFT DYNAMICS ''AS IS'' AND ANY
  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL DEFT DYNAMICS BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************/

#include "Arduino.h"
#include "neoLRR.h"
#include <SPI.h>

neoLRR::neoLRR(uint8_t addr) {
  // constructor
  address = addr;
}

void neoLRR::begin(uint8_t cs, int8_t txPower, bool rxstandby) {
  CS = cs;
  SPI.begin();
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);

  // set up the LoRa module

  // set to sleep mode, set to LoRa mode
  writeByte(REG_01_OP_MODE, MODE_SLEEP | LONG_RANGE_MODE);
  delay(10);
  // check that the settings stuck
  if (readByte(REG_01_OP_MODE) != (MODE_SLEEP | LONG_RANGE_MODE))
  {
    Serial.println("*** failed to set LoRa mode - something is wrong ***");
    while (1 == 1) {}
  }

  // would set interrupt pins here - skip for now

  // Set up FIFO
  // We configure so that we can use the entire 256 byte FIFO for either receive
  // or transmit, but not both at the same time
  writeByte(REG_0E_FIFO_TX_BASE_ADDR, 0);
  writeByte(REG_0F_FIFO_RX_BASE_ADDR, 0);


  // Packet format is preamble + explicit-header + payload + crc
  // Explicit Header Mode
  // payload is TO + FROM + ID + FLAGS + message data
  // RX mode is implmented with RXCONTINUOUS
  // max message data length is 255 - 4 = 251 octets

  setModeIdle();

  // Set up default configuration
  // No Sync Words in LORA mode.
  setModemRegisters(0); // 0=mid+mid, 1=short+fast, 2=long+slow, 3=long+slow

  setPreambleLength(8); // Default is 8

  // An innocuous ISM frequency, same as RF22's
  setFrequency(915.0);

  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm.
  // 'false' as second argument will use LoRa mode (default) not RFO
  setTxPower(txPower, false); 
  // only reason not to use higher power is current consumption during transmission:
  //  +20dBm - 120 mA
  //  +17dBm -  87 mA

  // the radio listener can operate in two modes:
  // 0) standby unless explicity changed to transmit/receive/sleep
  // 1) receiver unless explicity changed to transmit/standby/sleep
  // standby pulls 1.6 mA current, receive pulls ~12mA current
  // in general it's easier to use rxStandby = true so messages cannot be missed
  rxStandby = rxstandby;
  if (rxStandby){
    setModeRx();
  } else {
    setModeIdle();
  }

  // set up ring Buffer
  _rxHead = _rxTail = 0;
}


// -------------------- SPI FUNCTIONS -----------------------

void neoLRR::writeByte(byte Register, byte Data)
{
  byte Register2 = Register | 0b10000000;
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS, LOW);
  SPI.transfer(Register2);
  SPI.transfer(Data);
  digitalWrite(CS, HIGH);
  SPI.endTransaction();
}

void neoLRR::writeBytes(uint8_t Register, const uint8_t* src, uint16_t len)
{
  byte Register2 = Register | 0b10000000;
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS, LOW);
  SPI.transfer(Register2);
  while (len--) {
    SPI.transfer(*src++);
  }
  digitalWrite(CS, HIGH);
  SPI.endTransaction();
}

unsigned int neoLRR::readByte(byte Register)
{
  byte Register2 = Register | 0b00000000;
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS, LOW);
  SPI.transfer(Register2);
  byte result = SPI.transfer(0x00);
  digitalWrite(CS, HIGH);
  SPI.endTransaction();
  return (result);
}

void neoLRR::readBytes(byte Register, uint8_t* dest, uint8_t len)
{
  byte Register2 = Register | 0b00000000;
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS, LOW);
  SPI.transfer(Register2);
  while (len--) {
    *dest++ = SPI.transfer(0x00);
  }
  digitalWrite(CS, HIGH);
  SPI.endTransaction();
}


// -------------------- RFM95 SETUP FUNCTIONS -----------------------

void neoLRR::setModeIdle()
{
  writeByte(REG_01_OP_MODE, MODE_STDBY);
  mode = MODE_STDBY;
}

void neoLRR::sleep()
{
  writeByte(REG_01_OP_MODE, MODE_SLEEP);
  mode = MODE_SLEEP;
}

void neoLRR::setModeRx()
{
  writeByte(REG_01_OP_MODE, MODE_RXCONTINUOUS);
  writeByte(REG_40_DIO_MAPPING1, 0x00); // Interrupt on RxDone
  mode = MODE_RXCONTINUOUS;
}

void neoLRR::setModeTx()
{
  writeByte(REG_01_OP_MODE, MODE_TX);
  writeByte(REG_40_DIO_MAPPING1, 0x40); // Interrupt on TxDone
  mode = MODE_TX;
}

void neoLRR::setTxPower(int8_t power, bool useRFO)
{
  // Sigh, different behaviours depending on whther the module use PA_BOOST or the RFO pin
  // for the transmitter output
  if (useRFO)
  {
    if (power > 14)
      power = 14;
    if (power < -1)
      power = -1;
    writeByte(REG_09_PA_CONFIG, MAX_POWER | (power + 1));
  }
  else
  {
    if (power > 23)
      power = 23;
    if (power < 5)
      power = 5;

    // For RH_RF95_PA_DAC_ENABLE, manual says '+20dBm on PA_BOOST when OutputPower=0xf'
    // RH_RF95_PA_DAC_ENABLE actually adds about 3dBm to all power levels. We will us it
    // for 21, 22 and 23dBm
    if (power > 20)
    {
      writeByte(REG_4D_PA_DAC, PA_DAC_ENABLE);
      power -= 3;
    }
    else
    {
      writeByte(REG_4D_PA_DAC, PA_DAC_DISABLE);
    }

    // RFM95/96/97/98 does not have RFO pins connected to anything. Only PA_BOOST
    // pin is connected, so must use PA_BOOST
    // Pout = 2 + OutputPower.
    // The documentation is pretty confusing on this topic: PaSelect says the max power is 20dBm,
    // but OutputPower claims it would be 17dBm.
    // My measurements show 20dBm is correct
    writeByte(REG_09_PA_CONFIG, PA_SELECT | (power - 5));
  }
}

bool neoLRR::setFrequency(float center)
{
  // Frf = FRF / FSTEP
  uint32_t frf = (center * 1000000.0) / FSTEP;
  writeByte(REG_06_FRF_MSB, (frf >> 16) & 0xff);
  writeByte(REG_07_FRF_MID, (frf >> 8) & 0xff);
  writeByte(REG_08_FRF_LSB, frf & 0xff);
  return true;
}


void neoLRR::setModemRegisters(uint8_t mode)
{
  uint8_t reg_1D = 0x72; // Bandwidth (see datasheet)
  uint8_t reg_1E = 0x74; // Spreading factor in chips/symbol (see datasheet)
  uint8_t reg_26 = 0x00; // Static Node, fixed LNA gain
  
         if (mode==0){ // BW = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. medium range (=DEFAULT)
    reg_1D = 0x72;
    reg_1E = 0x74;
  } else if (mode==1){ // BW = 500 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Fast+short range
    reg_1D = 0x92;
    reg_1E = 0x74;
  } else if (mode==2){ // BW = 31.25 kHz, Cr = 4/8, Sf = 512chips/symbol, CRC on. Slow+long range
    reg_1D = 0x48;
    reg_1E = 0x94;
  } else if (mode==3){ // BW = 125 kHz, Cr = 4/8, Sf = 4096chips/symbol, CRC on. Slow+long range
    reg_1D = 0x78;
    reg_1E = 0xC4;
  }
  //  Bw500Cr45Sf128   = { 0x92,   0x74,    0x00} < Bw = 500 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Fast+short range
  //  Bw125Cr45Sf128   = { 0x72,   0x74,    0x00} < Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Default medium range
  //  Bw31_25Cr48Sf512 = { 0x48,   0x94,    0x00} < Bw = 31.25 kHz, Cr = 4/8, Sf = 512chips/symbol, CRC on. Slow+long range
  //  Bw125Cr48Sf4096  = { 0x78,   0xc4,    0x00} < Bw = 125 kHz, Cr = 4/8, Sf = 4096chips/symbol, CRC on. Slow+long range
  writeByte(REG_1D_MODEM_CONFIG1, reg_1D);
  writeByte(REG_1E_MODEM_CONFIG2, reg_1E);
  writeByte(REG_26_MODEM_CONFIG3, reg_26);
}

void neoLRR::setPreambleLength(uint16_t bytes)
{
  writeByte(REG_20_PREAMBLE_MSB, bytes >> 8);
  writeByte(REG_21_PREAMBLE_LSB, bytes & 0xff);
}


// -------------------- RFM95 COMMON FUNCTIONS -----------------------

bool neoLRR::_send(const uint8_t* data, uint8_t len)
{
  if (len > MAX_MESSAGE_LEN) {
    return false;
  }

  // Make sure we dont interrupt an outgoing message
  while (mode == MODE_TX) {
    poll();
    delay(10);
  }
  setModeIdle();

  // Position at the beginning of the FIFO
  writeByte(REG_0D_FIFO_ADDR_PTR, 0);
  // The headers
  writeByte(REG_00_FIFO, _txHeaderTo);
  writeByte(REG_00_FIFO, _txHeaderFrom);
  writeByte(REG_00_FIFO, _txHeaderId);
  writeByte(REG_00_FIFO, _txHeaderFlags);
  // The message data
  writeBytes(REG_00_FIFO, data, len);
  writeByte(REG_22_PAYLOAD_LENGTH, len + HEADER_LEN);

  setModeTx(); // Start the transmitter
  return true;
}


bool neoLRR::poll()
{
  // Read the interrupt register
  uint8_t irq_flags = readByte(REG_12_IRQ_FLAGS);

  if (mode == MODE_RXCONTINUOUS && irq_flags & (RX_TIMEOUT | PAYLOAD_CRC_ERROR))
  {
    // return a flag that there was an error
    Serial.print("[CRC Error]");
    writeByte(REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags
    return false;
  }
  else if (mode == MODE_RXCONTINUOUS && irq_flags & RX_DONE)
  {
    // Have received a packet
    uint8_t len = readByte(REG_13_RX_NB_BYTES);

    // Reset the fifo read ptr to the beginning of the packet
    writeByte(REG_0D_FIFO_ADDR_PTR, readByte(REG_10_FIFO_RX_CURRENT_ADDR));
    readBytes(REG_00_FIFO, _buf, len);
    _bufLen = len;
    
    writeByte(REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags

    // Remember the RSSI of this packet
    // this is according to the doc, but is it really correct?
    // weakest receiveable signals are reported RSSI at about -66
    lastRssi = readByte(REG_1A_PKT_RSSI_VALUE) - 137;
    // Remember the estimated SNR of this packet
    lastSNR = (int8_t)readByte(REG_19_PKT_SNR_VALUE)/4.0;

    // We have received a message.
    if (validateRxBuf())
    {
      // successfully received a message

      // copy data into Stream buffer
      for (int i = HEADER_LEN; i < len; i++) {
        _rxHead = (_rxHead + 1) % sizeof(_rxBuffer);
        _rxBuffer[_rxHead] = _buf[i];
      }
      clearRxBuf(); // clear the packet buffer

      // send Ack if needed
      if ((usingAck)&&(!waitingForAck)){
        _txHeaderFlags = 0x01;
        char x[] = "!";
        _send((uint8_t *)x,1);
        waitForDispatch();
        _txHeaderFlags = 0x00;
      }
    
      if (rxStandby){
        setModeRx();
      } else {
        setModeIdle();
      }
      return true;
    } else {
      // something went wrong
      return false;
    }
  }
  else if (mode == MODE_TX && irq_flags & TX_DONE)
  {
    if (rxStandby){
      setModeRx();
    } else {
      setModeIdle();
    }
    writeByte(REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags
    return true;
  }
  return false;
}

// Check whether the latest received message is complete and uncorrupted
bool neoLRR::validateRxBuf()
{
  if (_bufLen < 4) {
    return false; // Too short to be a real message
  }
  // Extract the 4 headers
  _rxHeaderTo    = _buf[0];
  _rxHeaderFrom  = _buf[1];
  _rxHeaderId    = _buf[2];
  _rxHeaderFlags = _buf[3];
  if (_rxHeaderTo == _thisAddress || _rxHeaderTo == BROADCAST_ADDRESS)
  {
    _rxBufValid = true;
    return true;
  }
  return false;
}

void neoLRR::clearRxBuf()
{
  _rxBufValid = false;
  _bufLen = 0;
}

bool neoLRR::waitForDispatch(uint32_t timeout)
{
  elapsedMillis timer = 0;
  bool sendComplete = poll();
  while ((timer < timeout) && (!sendComplete))
  {
    sendComplete = poll();
  }
  return sendComplete;
}

bool neoLRR::waitForMessage(uint32_t timeout)
{
  if (mode != MODE_RXCONTINUOUS) {
    setModeRx();
  } 
  bool  received = poll();
  elapsedMillis timer = 0;
  while ((timer < timeout) && (!received)) {
    received = poll();
  }
  return received;
}

void neoLRR::setFromAddress(uint8_t addr){
  _thisAddress = addr;
  _txHeaderFrom = addr;
}

void neoLRR::setToAddress(uint8_t addr){
  _txHeaderTo = addr;
}

void neoLRR::reliableData(bool useAck){
  usingAck = useAck;
}

// ------------------- BUFFER Control ---------------------------

int neoLRR::available(void) {
  int retval = (_rxHead - _rxTail + sizeof(_rxBuffer)) % sizeof(_rxBuffer);
  return retval;
}

int neoLRR::peek(void) {
  if (_rxTail == _rxHead) return -1;
  uint8_t byte = _rxBuffer[_rxTail];
  return byte;
}

int neoLRR::read(void) {
  if (_rxTail == _rxHead) return -1;
  _rxTail = (_rxTail + 1) % sizeof(_rxBuffer);
  uint8_t byte = _rxBuffer[_rxTail];
  return byte;
}

bool neoLRR::send(String packet) {
  uint8_t stringLen = packet.length();
  char charBuf[250];
  packet.toCharArray(charBuf,stringLen+1);
  _send((uint8_t *)charBuf, stringLen);

  if (usingAck){
    waitingForAck = true;
    waitForDispatch(1000);
    waitForMessage(1000);
    waitingForAck = false;
    if ((_rxHeaderFlags = 0x01)){
      _rxHeaderFlags = 0x00; // reset flags
      read();                      // clear the '!' from the buffer
      return 1;
    } else {
      return 0;
    }
  } else {
    return 1;
  }
}

