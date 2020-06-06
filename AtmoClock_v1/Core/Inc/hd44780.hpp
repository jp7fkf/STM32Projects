/*
 * hd44780.hpp
 *
 *  Created on: May 30, 2020
 *      Author: Yudai Hashimoto
 *      Web   : https://jp7fkf.dev/
 *
 * MIT License
 *
 * Copyright (c) 2020 Yudai Hashimoto (JP7FKF)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef SRC_HD44780_HPP_
#define SRC_HD44780_HPP_

#include <string.h>
#include "stm32f0xx_hal.h"

// commands
constexpr uint8_t LCD_CLEARDISPLAY = 0x01;
constexpr uint8_t LCD_RETURNHOME = 0x02;
constexpr uint8_t LCD_ENTRYMODESET = 0x04;
constexpr uint8_t LCD_DISPLAYCONTROL = 0x08;
constexpr uint8_t LCD_CURSORSHIFT = 0x10;
constexpr uint8_t LCD_FUNCTIONSET = 0x20;
constexpr uint8_t LCD_SETCGRAMADDR = 0x40;
constexpr uint8_t LCD_SETDDRAMADDR = 0x80;

// display entry mode
constexpr uint8_t LCD_ENTRYRIGHT = 0x00;
constexpr uint8_t LCD_ENTRYLEFT = 0x02;
constexpr uint8_t LCD_ENTRYSHIFTINCREMENT = 0x01;
constexpr uint8_t LCD_ENTRYSHIFTDECREMENT = 0x00;

// display on/off control
constexpr uint8_t LCD_DISPLAYON = 0x04;
constexpr uint8_t LCD_DISPLAYOFF = 0x00;
constexpr uint8_t LCD_CURSORON = 0x02;
constexpr uint8_t LCD_CURSOROFF = 0x00;
constexpr uint8_t LCD_BLINKON = 0x01;
constexpr uint8_t LCD_BLINKOFF = 0x00;

// display/cursor shift
constexpr uint8_t LCD_DISPLAYMOVE = 0x08;
constexpr uint8_t LCD_CURSORMOVE = 0x00;
constexpr uint8_t LCD_MOVERIGHT = 0x04;
constexpr uint8_t LCD_MOVELEFT = 0x00;

// function set
constexpr uint8_t LCD_8BITMODE = 0x10;
constexpr uint8_t LCD_4BITMODE = 0x00;
constexpr uint8_t LCD_2LINE = 0x08;
constexpr uint8_t LCD_1LINE = 0x00;
constexpr uint8_t LCD_5x10DOTS = 0x04;
constexpr uint8_t LCD_5x8DOTS = 0x00;

class Hd44780 {
  public:
    Hd44780(uint8_t mode, GPIO_TypeDef* rsPort, uint16_t rsPin,
        GPIO_TypeDef* rwPort,uint16_t rwPin,
        GPIO_TypeDef* enablePort, uint16_t enablePin,
        GPIO_TypeDef** dataPorts, uint16_t* dataPins):
          mode_(mode),
          rsPort_(rsPort),
          rsPin_(rsPin),
          rwPort_(rwPort),
          rwPin_(rwPin),
          enablePort_(enablePort),
          enablePin_(enablePin),
          dataPorts_(dataPorts),
          dataPins_(dataPins){
    }
    void Init();
    void SendData(uint8_t data);
    void SendCommand(uint8_t command);
    void Puts(char* string);
    void SetCursor(uint8_t x, uint8_t y);
    void Clear();
    void Home();
  private:
    uint8_t mode_;
    GPIO_TypeDef* rsPort_;
    uint16_t rsPin_;
    GPIO_TypeDef* rwPort_;
    uint16_t rwPin_;
    GPIO_TypeDef* enablePort_;
    uint16_t enablePin_;
    GPIO_TypeDef** dataPorts_;
    uint16_t* dataPins_;
    void write(uint8_t data, uint8_t len);
};

#endif /* SRC_HD44780_HPP_ */
