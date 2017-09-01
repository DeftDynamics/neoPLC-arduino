/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  Copyright (c) 2016 Sandeep Mistry All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "Uart.h"
#include "Arduino.h"
#include "wiring_private.h"

// Weak empty variant UART Interrupt.
// May be redefined by variant files.
void variant_UART_IRQ_Handler() __attribute__((weak));
void variant_UART_IRQ_Handler() { }

Uart::Uart(NRF_UART_Type *_nrfUart, IRQn_Type _IRQn, uint8_t _pinRX, uint8_t _pinTX)
{
  nrfUart = _nrfUart;
  IRQn = _IRQn;
  uc_pinRX = g_ADigitalPinMap[_pinRX];
  uc_pinTX = g_ADigitalPinMap[_pinTX];
}

void Uart::begin(unsigned long baudrate)
{
}

void Uart::begin(unsigned long baudrate, uint16_t /*config*/)
{
}

void Uart::end()
{
}

void Uart::core_begin(unsigned long baudrate)
{
  core_begin(baudrate, (uint8_t)SERIAL_8N1);
}

void Uart::core_begin(unsigned long baudrate, uint16_t /*config*/)
{
  nrfUart->PSELTXD = uc_pinTX;
  nrfUart->PSELRXD = uc_pinRX;

  nrfUart->CONFIG = (UART_CONFIG_PARITY_Excluded << UART_CONFIG_PARITY_Pos) | UART_CONFIG_HWFC_Disabled;

  uint32_t nrfBaudRate;

#ifdef NRF52
  if (baudrate <= 1200) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud1200;
  } else if (baudrate <= 2400) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud2400;
  } else if (baudrate <= 4800) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud4800;
  } else if (baudrate <= 9600) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud9600;
  } else if (baudrate <= 14400) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud14400;
  } else if (baudrate <= 19200) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud19200;
  } else if (baudrate <= 28800) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud28800;
  } else if (baudrate <= 38400) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud38400;
  } else if (baudrate <= 57600) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud57600;
  } else if (baudrate <= 76800) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud76800;
  } else if (baudrate <= 115200) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud115200;
  } else if (baudrate <= 230400) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud230400;
  } else if (baudrate <= 250000) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud250000;
  } else if (baudrate <= 460800) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud460800;
  } else if (baudrate <= 921600) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud921600;
  } else {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud1M;
  }
#else
  if (baudrate <= 1200) {
    nrfBaudRate = UART_BAUDRATE_BAUDRATE_Baud1200;
  } else if (baudrate <= 2400) {
    nrfBaudRate = UART_BAUDRATE_BAUDRATE_Baud2400;
  } else if (baudrate <= 4800) {
    nrfBaudRate = UART_BAUDRATE_BAUDRATE_Baud4800;
  } else if (baudrate <= 9600) {
    nrfBaudRate = UART_BAUDRATE_BAUDRATE_Baud9600;
  } else if (baudrate <= 14400) {
    nrfBaudRate = UART_BAUDRATE_BAUDRATE_Baud14400;
  } else if (baudrate <= 19200) {
    nrfBaudRate = UART_BAUDRATE_BAUDRATE_Baud19200;
  } else if (baudrate <= 28800) {
    nrfBaudRate = UART_BAUDRATE_BAUDRATE_Baud28800;
  } else if (baudrate <= 38400) {
    nrfBaudRate = UART_BAUDRATE_BAUDRATE_Baud38400;
  } else if (baudrate <= 57600) {
    nrfBaudRate = UART_BAUDRATE_BAUDRATE_Baud57600;
  } else if (baudrate <= 76800) {
    nrfBaudRate = UART_BAUDRATE_BAUDRATE_Baud76800;
  } else if (baudrate <= 115200) {
    nrfBaudRate = UART_BAUDRATE_BAUDRATE_Baud115200;
  } else if (baudrate <= 230400) {
    nrfBaudRate = UART_BAUDRATE_BAUDRATE_Baud230400;
  } else if (baudrate <= 250000) {
    nrfBaudRate = UART_BAUDRATE_BAUDRATE_Baud250000;
  } else if (baudrate <= 460800) {
    nrfBaudRate = UART_BAUDRATE_BAUDRATE_Baud460800;
  } else if (baudrate <= 921600) {
    nrfBaudRate = UART_BAUDRATE_BAUDRATE_Baud921600;
  } else {
    nrfBaudRate = UART_BAUDRATE_BAUDRATE_Baud1M;
  }
#endif

  nrfUart->BAUDRATE = nrfBaudRate;

  nrfUart->ENABLE = UART_ENABLE_ENABLE_Enabled;

  nrfUart->EVENTS_RXDRDY = 0x0UL;
  nrfUart->EVENTS_TXDRDY = 0x0UL;

  nrfUart->TASKS_STARTRX = 0x1UL;
  nrfUart->TASKS_STARTTX = 0x1UL;

  nrfUart->INTENSET = UART_INTENSET_RXDRDY_Msk;

  NVIC_ClearPendingIRQ(IRQn);
  NVIC_SetPriority(IRQn, 3);
  NVIC_EnableIRQ(IRQn);
}

void Uart::core_end()
{
//seriously dont call this function. it could ruin the bootloader.
  NVIC_DisableIRQ(IRQn);

  nrfUart->INTENCLR = UART_INTENCLR_RXDRDY_Msk;

  nrfUart->TASKS_STOPRX = 0x1UL;
  nrfUart->TASKS_STOPTX = 0x1UL;

  nrfUart->ENABLE = UART_ENABLE_ENABLE_Disabled;

  nrfUart->PSELTXD = 0xFFFFFFFF;
  nrfUart->PSELRXD = 0xFFFFFFFF;

	rxBuffer.clear();
}

void Uart::flush()
{
}

void Uart::IrqHandler()
{
  if (nrfUart->EVENTS_RXDRDY)
  {
    nrfUart->EVENTS_RXDRDY = 0x0UL;
	rxBuffer.store_char(nrfUart->RXD);
	variant_UART_IRQ_Handler();
  }
}

int Uart::available()
{
  return rxBuffer.available();
}

int Uart::peek()
{
  return rxBuffer.peek();
}

int Uart::peek_pos(int pos)
{
  return rxBuffer.peek_pos(pos);
}

int Uart::read()
{
  return rxBuffer.read_char();
}

size_t Uart::write(const uint8_t data)
{
  nrfUart->EVENTS_TXDRDY = 0x0UL;
  nrfUart->TXD = data;

  while(!nrfUart->EVENTS_TXDRDY);
  

  return 1;
}

#if defined(NRF52)
Uart Serial( NRF_UART0, UARTE0_UART0_IRQn, PIN_SERIAL_RX, PIN_SERIAL_TX );

extern "C"
{
  void UARTE0_UART0_IRQHandler()
  {
    Serial.IrqHandler();
  }
}
#elif defined(NRF51)
Uart Serial( NRF_UART0, UART0_IRQn, PIN_SERIAL_RX, PIN_SERIAL_TX );

extern "C"
{
  void UART0_IRQHandler()
  {
    Serial.IrqHandler();
  }
}
#endif
