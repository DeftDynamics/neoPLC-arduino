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


#ifndef NEOLRR_H
#define NEOLRR_H

#include "Arduino.h"
#include <SPI.h>

// default address 0xff is radioHead default when addressing is not used
#define BROADCAST_ADDRESS 0xff
// The crystal oscillator frequency of the module
#define FXOSC 32000000.0
// The Frequency Synthesizer step = RH_RF95_FXOSC / 2^^19
#define FSTEP  (FXOSC / 524288)

// Max number of octets the LORA Rx/Tx FIFO can hold
#define RH_RF95_FIFO_SIZE 255
// This is the maximum number of bytes that can be carried by the LORA.
// We use some for headers, keeping fewer for RadioHead messages
#define MAX_PAYLOAD_LEN RH_RF95_FIFO_SIZE
// The length of the headers we add. The headers are inside the LORA's payload
#define HEADER_LEN 4
// This is the maximum message length that can be supported by this driver.
#define MAX_MESSAGE_LEN (MAX_PAYLOAD_LEN - HEADER_LEN)


// Register names (LoRa Mode, from table 85)
#define REG_00_FIFO                                0x00
#define REG_01_OP_MODE                             0x01
#define REG_02_RESERVED                            0x02
#define REG_03_RESERVED                            0x03
#define REG_04_RESERVED                            0x04
#define REG_05_RESERVED                            0x05
#define REG_06_FRF_MSB                             0x06
#define REG_07_FRF_MID                             0x07
#define REG_08_FRF_LSB                             0x08
#define REG_09_PA_CONFIG                           0x09
#define REG_0A_PA_RAMP                             0x0a
#define REG_0B_OCP                                 0x0b
#define REG_0C_LNA                                 0x0c
#define REG_0D_FIFO_ADDR_PTR                       0x0d
#define REG_0E_FIFO_TX_BASE_ADDR                   0x0e
#define REG_0F_FIFO_RX_BASE_ADDR                   0x0f
#define REG_10_FIFO_RX_CURRENT_ADDR                0x10
#define REG_11_IRQ_FLAGS_MASK                      0x11
#define REG_12_IRQ_FLAGS                           0x12
#define REG_13_RX_NB_BYTES                         0x13
#define REG_14_RX_HEADER_CNT_VALUE_MSB             0x14
#define REG_15_RX_HEADER_CNT_VALUE_LSB             0x15
#define REG_16_RX_PACKET_CNT_VALUE_MSB             0x16
#define REG_17_RX_PACKET_CNT_VALUE_LSB             0x17
#define REG_18_MODEM_STAT                          0x18
#define REG_19_PKT_SNR_VALUE                       0x19
#define REG_1A_PKT_RSSI_VALUE                      0x1a
#define REG_1B_RSSI_VALUE                          0x1b
#define REG_1C_HOP_CHANNEL                         0x1c
#define REG_1D_MODEM_CONFIG1                       0x1d
#define REG_1E_MODEM_CONFIG2                       0x1e
#define REG_1F_SYMB_TIMEOUT_LSB                    0x1f
#define REG_20_PREAMBLE_MSB                        0x20
#define REG_21_PREAMBLE_LSB                        0x21
#define REG_22_PAYLOAD_LENGTH                      0x22
#define REG_23_MAX_PAYLOAD_LENGTH                  0x23
#define REG_24_HOP_PERIOD                          0x24
#define REG_25_FIFO_RX_BYTE_ADDR                   0x25
#define REG_26_MODEM_CONFIG3                       0x26

#define REG_40_DIO_MAPPING1                        0x40
#define REG_41_DIO_MAPPING2                        0x41
#define REG_42_VERSION                             0x42

#define REG_4B_TCXO                                0x4b
#define REG_4D_PA_DAC                              0x4d
#define REG_5B_FORMER_TEMP                         0x5b
#define REG_61_AGC_REF                             0x61
#define REG_62_AGC_THRESH1                         0x62
#define REG_63_AGC_THRESH2                         0x63
#define REG_64_AGC_THRESH3                         0x64

// REG_01_OP_MODE                             0x01
#define LONG_RANGE_MODE                       0x80
#define ACCESS_SHARED_REG                     0x40
#define MODE                                  0x07
#define MODE_SLEEP                            0x00
#define MODE_STDBY                            0x01
#define MODE_FSTX                             0x02
#define MODE_TX                               0x03
#define MODE_FSRX                             0x04
#define MODE_RXCONTINUOUS                     0x05
#define MODE_RXSINGLE                         0x06
#define MODE_CAD                              0x07

// REG_09_PA_CONFIG                           0x09
#define PA_SELECT                             0x80
#define MAX_POWER                             0x70
#define OUTPUT_POWER                          0x0f

// REG_0A_PA_RAMP                             0x0a
#define LOW_PN_TX_PLL_OFF                     0x10
#define PA_RAMP                               0x0f
#define PA_RAMP_3_4MS                         0x00
#define PA_RAMP_2MS                           0x01
#define PA_RAMP_1MS                           0x02
#define PA_RAMP_500US                         0x03
#define PA_RAMP_250US                         0x0
#define PA_RAMP_125US                         0x05
#define PA_RAMP_100US                         0x06
#define PA_RAMP_62US                          0x07
#define PA_RAMP_50US                          0x08
#define PA_RAMP_40US                          0x09
#define PA_RAMP_31US                          0x0a
#define PA_RAMP_25US                          0x0b
#define PA_RAMP_20US                          0x0c
#define PA_RAMP_15US                          0x0d
#define PA_RAMP_12US                          0x0e
#define PA_RAMP_10US                          0x0f

// REG_0B_OCP                                 0x0b
#define OCP_ON                                0x20
#define OCP_TRIM                              0x1f

// REG_0C_LNA                                 0x0c
#define LNA_GAIN                              0xe0
#define LNA_BOOST                             0x03
#define LNA_BOOST_DEFAULT                     0x00
#define LNA_BOOST_150PC                       0x11

// REG_11_IRQ_FLAGS_MASK                      0x11
#define RX_TIMEOUT_MASK                       0x80
#define RX_DONE_MASK                          0x40
#define PAYLOAD_CRC_ERROR_MASK                0x20
#define VALID_HEADER_MASK                     0x10
#define TX_DONE_MASK                          0x08
#define CAD_DONE_MASK                         0x04
#define FHSS_CHANGE_CHANNEL_MASK              0x02
#define CAD_DETECTED_MASK                     0x01

// REG_12_IRQ_FLAGS                           0x12
#define RX_TIMEOUT                            0x80
#define RX_DONE                               0x40
#define PAYLOAD_CRC_ERROR                     0x20
#define VALID_HEADER                          0x10
#define TX_DONE                               0x08
#define CAD_DONE                              0x04
#define FHSS_CHANGE_CHANNEL                   0x02
#define CAD_DETECTED                          0x01

// REG_18_MODEM_STAT                          0x18
#define RX_CODING_RATE                        0xe0
#define MODEM_STATUS_CLEAR                    0x10
#define MODEM_STATUS_HEADER_INFO_VALID        0x08
#define MODEM_STATUS_RX_ONGOING               0x04
#define MODEM_STATUS_SIGNAL_SYNCHRONIZED      0x02
#define MODEM_STATUS_SIGNAL_DETECTED          0x01

// REG_1C_HOP_CHANNEL                         0x1c
#define PLL_TIMEOUT                           0x80
#define RX_PAYLOAD_CRC_IS_ON                  0x40
#define FHSS_PRESENT_CHANNEL                  0x3f

// REG_1D_MODEM_CONFIG1                       0x1d
#define BW                                    0xc0
#define BW_125KHZ                             0x00
#define BW_250KHZ                             0x40
#define BW_500KHZ                             0x80
#define BW_RESERVED                           0xc0
#define CODING_RATE                           0x38
#define CODING_RATE_4_5                       0x00
#define CODING_RATE_4_6                       0x08
#define CODING_RATE_4_7                       0x10
#define CODING_RATE_4_8                       0x18
#define IMPLICIT_HEADER_MODE_ON               0x04
#define RX_PAYLOAD_CRC_ON                     0x02
#define LOW_DATA_RATE_OPTIMIZE                0x01

// REG_1E_MODEM_CONFIG2                       0x1e
#define SPREADING_FACTOR                      0xf0
#define SPREADING_FACTOR_64CPS                0x60
#define SPREADING_FACTOR_128CPS               0x70
#define SPREADING_FACTOR_256CPS               0x80
#define SPREADING_FACTOR_512CPS               0x90
#define SPREADING_FACTOR_1024CPS              0xa0
#define SPREADING_FACTOR_2048CPS              0xb0
#define SPREADING_FACTOR_4096CPS              0xc0
#define TX_CONTINUOUS_MOE                     0x08
#define AGC_AUTO_ON                           0x04
#define SYM_TIMEOUT_MSB                       0x03

// REG_4D_PA_DAC                              0x4d
#define PA_DAC_DISABLE                        0x04
#define PA_DAC_ENABLE                         0x07

class neoLRR
{
  public:
    neoLRR(uint8_t addr = 0x00);
    void begin(uint8_t cs=10, int8_t txPower=17, bool rxStandby=1);
    void setModeIdle();
    void sleep();
    void setModeRx();
    void setModeTx();
    void setTxPower(int8_t power, bool useRFO);
    bool poll();
    bool waitForDispatch(uint32_t timeout=2000);
    bool waitForMessage(uint32_t timeout=2000);
    void setFromAddress(uint8_t addr=BROADCAST_ADDRESS);
    void setToAddress(uint8_t addr=BROADCAST_ADDRESS);
    void reliableData(bool useAck=true);
    
    // variables
    int8_t lastRssi = 0;
    float lastSNR = 0.0;
    bool rxStandby = false;

    // inherit some stuff from Stream to create a virtual buffer
     int available(void);
     int read(void);
     int peek(void);
    bool send(String packet);
    
  private:
  
    // methods
    bool setFrequency(float center);
    void setModemRegisters(uint8_t mode=0);
    void setPreambleLength(uint16_t bytes);
    bool validateRxBuf();
    void clearRxBuf();
    void writeByte(byte Register, byte Data);
    void writeBytes(uint8_t Register, const uint8_t* src, uint16_t len);
    unsigned int readByte(byte Register);
    void readBytes(byte Register, uint8_t* dest, uint8_t len);
    bool _send(const uint8_t* data, uint8_t len);
    

    // variables

    bool usingAck = false;
    bool waitingForAck = false;
    uint8_t _thisAddress = BROADCAST_ADDRESS;
    uint8_t _txHeaderTo = BROADCAST_ADDRESS;
    uint8_t _txHeaderFrom = BROADCAST_ADDRESS;
    uint8_t _txHeaderId = 0;
    uint8_t _txHeaderFlags = 0;

    uint8_t _rxHeaderTo    = 0;
    uint8_t _rxHeaderFrom  = 0;
    uint8_t _rxHeaderId    = 0;
    uint8_t _rxHeaderFlags = 0;
    
    
    uint8_t _buf[MAX_PAYLOAD_LEN]; // The receiver/transmitter buffer
    uint8_t CS = 10;
    int address = 0x48;
    int _bufLen = 0;
    bool _rxBufValid = true;
    uint8_t mode = 1;
    // modes:
    // 000 - sleep
    // 001 - standby
    // 010 - frequency synthesis TX
    // 011 - transmit
    // 100 - frequency synthesis RX
    // 101 - continuous RX
    // 110 - receive single
    // 111 - channel activity detection

   // Ring Buffer to mimic a Stream class
      uint16_t _rxHead;
      uint16_t _rxTail;
      uint16_t _rxCount() const;
      uint8_t _rxBuffer[255];
};



#endif



