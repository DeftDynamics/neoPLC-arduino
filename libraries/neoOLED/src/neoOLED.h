/***************************************************************************
neoPLC-OLED 

A library for SSD1306 by Soloman Systech

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

// elements of this library are based on code by William Greiman,
// released under the following license:

/* Arduino SSD1306Ascii Library
 * Copyright (C) 2015 by William Greiman
 *
 * This file is part of the Arduino SSD1306Ascii Library
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Arduino SSD1306Ascii Library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file SSD1306Ascii.h
 * @brief Base class for ssd1306 displays.
 */
#ifndef neoOLED_h
#define neoOLED_h
#include "Arduino.h"
#include "neoOLEDinit.h"
#include "fonts/allFonts.h"
#include <Wire.h>
//------------------------------------------------------------------------------
/** SSD1306Ascii version YYYYMMDD */
#define SDD1306_ASCII_VERSION 20160127
//------------------------------------------------------------------------------
// Configuration options.
/** Set Scrolling mode for new line.
 *
 * If INCLUDE_SCROLLING is defined to be zero, new line will not scroll
 * the display and code for scrolling will not be included.  This option 
 * will save some code space and one byte of RAM.
 *
 * If INCLUDE_SCROLLING is defined to be one, the scroll feature will
 * be included but not enabled.  A call to setScroll() will be required
 * to enable scrolling.
 *
 * If INCLUDE_SCROLLING is defined to be two, the scroll feature will
 * be included and enabled. A call to setScroll() will be required
 * to disable scrolling.
 */
#define INCLUDE_SCROLLING 1

//------------------------------------------------------------------------------
// Values for writeDisplay() mode paarameter.
/** Write to Command register. */
#define SSD1306_MODE_CMD     0
/** Write one byte to display RAM. */
#define SSD1306_MODE_RAM     1
/** Write to display RAM with possible buffering. */
#define SSD1306_MODE_RAM_BUF 2
//------------------------------------------------------------------------------
/**
 * @class SSD1306Ascii
 * @brief SSD1306 base class
 */
class neoOLED : public Print {
 public:
 
  neoOLED(uint8_t i2cAddr=0x3C);
    /**
   * @brief Initialize the display controller.
   *
   * @param[in] dev A device initialization structure.
   * @param[in] i2cAddr The I2C address of the display controller.
   */
  void begin() 
  {
	Wire.begin();
    init(&MicroOLED64x48);    
  }
  /**
   * @brief Determine the width of a character.
   *
   * @param[in] c Character code.
   * @return Width of the character in pixels.
   */
  uint8_t charWidth(uint8_t c);  
  /**
   * @brief Clear the display and set the cursor to (0, 0).
   */
  void clear();
  /**
   * @brief Clear a region of the display.
   *
   * @param[in] c0 Starting column.
   * @param[in] c1 Ending column.
   * @param[in] r0 Starting row;
   * @param[in] r1 Ending row;
   * @note The final cursor position will be (c0, r0).
   */
  void clear(uint8_t c0, uint8_t c1, uint8_t r0, uint8_t r1);
  /**
   * @brief Clear the display to the end of the current line.
   * @note The number of rows cleared will be determined by the height
   *       of the current font. 
   * @note The cursor will be returned to the original position.
   */
  void clearToEOL();
  /**
   * @return The current column in pixels.
   */
  uint8_t col() {return m_col;}
  /**
   * @return The display hight in pixels.
   */
  uint8_t displayHeight() {return m_displayHeight;}
  /**
   * @return The display height in rows with eight pixels to a row.
   */
  uint8_t displayRows() {return m_displayHeight/8;}
  /**
   * @return The display width in pixels.
   */
  uint8_t displayWidth() {return m_displayWidth;}
  /**
   * @return The current font height in pixels.
   */
  uint8_t fontHeight();
  /**
   * @return The number of eight pixel rows required to display a character
   *    in the current font.
   */
  uint8_t fontRows() {return (fontHeight() + 7)/8;}
  /**
   * @return The maximum width of characters in the current font.
   */
  uint8_t fontWidth();
  /**
   * @brief Set the cursor position to (0, 0).
   */
  void home() {setCursor(0, 0);}
  /**
   * @brief Initialize the display controller.
   *
   * @param[in] dev A display initialization structure.
   */
  void init(const DevType* dev);   
  /**
   * @return The character magnification factor.
   */
  uint8_t magFactor() {return m_magFactor;}
  /**
   * @brief Reset the display controller.
   *
   * @param[in] rst Reset pin number.
   */
  static void reset(uint8_t rst);
  /**
   * @return the current row number with eight pixels to a row.
   */
  uint8_t row() {return m_row;}
  /**
   * @brief Set the character magnification factor to one.
   */
  void set1X() {m_magFactor = 1;}
  /**
   * @brief Set the character magnification factor to two.
   */
  void set2X() {m_magFactor = 2;}
  /**
   * @brief Set the current column number.
   *
   * @param[in] col The desired column number in pixels.
   */
  void setCol(uint8_t col);
  /**
   * @brief Set the display contrast.
   *
   * @param[in] value The contrast level in th range 0 to 255.
   */
  void setContrast(uint8_t value);
  /**
   * @brief Set the cursor position.
   *
   * @param[in] col The column number in pixels.
   * @param[in] row the row number in eight pixel rows.
   */
  void setCursor(uint8_t col, uint8_t row);
  /**
   * @brief Set the current font.
   *
   * @param[in] font Pointer to a font table.
   */
  void setFont(const uint8_t* font) {m_font = font;}
  /**
   * @brief Set the current row number.
   *
   * @param[in] row the row number in eight pixel rows.
   */
  void setRow(uint8_t row);
#if INCLUDE_SCROLLING   
  /**
   * @brief Enable or disable scroll mode.
   *
   * @param[in] enable true enable scroll on new line false disable scroll.
   * @note Scroll mode is only supported on 64 pixel high displays.
   *       Using setRow() or setCursor() will be unpredictable in scroll mode.
   *       You must use a font with an integral number of line on
   *       the display.
   */
  void setScroll(bool enable);
#endif  // INCLUDE_SCROLLING   
  /**
   * @brief Write a command byte to the display controller.
   *
   * @param[in] c The command byte.
   * @note The byte will immediately be sent to the controller. 
   */
  void ssd1306WriteCmd(uint8_t c) {writeDisplay(c, SSD1306_MODE_CMD);}
  /**
   * @brief Write a byte to RAM in the display controller.
   *
   * @param[in] c The data byte.
   * @note The byte will immediately be sent to the controller.   
   */   
  void ssd1306WriteRam(uint8_t c);
  /**
   * @brief Write a byte to RAM in the display controller.
   *
   * @param[in] c The data byte.
   * @note The byte may be buffered until a call to ssd1306WriteCmd
   *       or ssd1306WriteRam.
   */    
  void ssd1306WriteRamBuf(uint8_t c);
  /**
   * @brief Display a character.
   *
   * @param[in] c The character to display.
   * @return the value one.
   */   
  size_t write(uint8_t c);
  /**
   * @brief Display a string.
   *
   * @param[in] s The string to display.
   * @return The length of the string.
   */
  size_t write(const char* s);
  
protected:
  void writeDisplay(uint8_t b, uint8_t mode) {  
    Wire.beginTransmission(m_i2cAddr);
    Wire.write(mode == SSD1306_MODE_CMD ? 0X00: 0X40);
    Wire.write(b);
    Wire.endTransmission();
  }
  
 private:
  uint8_t m_i2cAddr;
  //virtual void writeDisplay(uint8_t b, uint8_t mode) = 0;
  uint8_t m_col;            // Cursor column.
  uint8_t m_row;            // Cursor RAM row.
  uint8_t m_displayWidth;   // Display width. 
  uint8_t m_displayHeight;  // Display height.
  uint8_t m_colOffset;      // Column offset RAM to SEG
  uint8_t m_magFactor=1;      // Magnification factor.
#if INCLUDE_SCROLLING    
  uint8_t m_scroll;          // Scroll mode 
#endif  // INCLUDE_SCROLLING    
  const uint8_t* m_font=0;    // Current font.
};
#endif  // neoOLED_h
