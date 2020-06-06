/*
 * st7032.hpp
 *
 *  Created on: May 27, 2020
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

#ifndef INC_ST7032_HPP_
#define INC_ST7032_HPP_

#include "stm32f0xx_hal.h"

constexpr uint8_t Write_Address                 = 0x3E; //i2c address
constexpr uint8_t CNTRBIT                       = 0x00; //followed by command bytes
constexpr uint8_t CNTRBIT_CO                    = 0x80; //followed by 1 command byte
constexpr uint8_t CNTRBIT_RS                    = 0x40; //after last control byte, followed by DDRAM data byte(s)
constexpr uint8_t CLEAR_DISPLAY                 = 0x01; //Clear display
constexpr uint8_t RETURN_HOME                   = 0x02; //Cursor home to 00H
constexpr uint8_t ENTRY_MODE_SET                = 0x04; //Sets cursor move direction and specifies display shift.
constexpr uint8_t DISPLAY_ON_OFF                = 0x08; //display on, cursor on, cursor position on
constexpr uint8_t FUNCTION_SET                  = 0x20; //DL: interface data is 8/4 bits, N: number of line is 2/1 DH: double height font, IS: instruction table select
constexpr uint8_t SET_DDRAM_ADDRESS             = 0x80; //Set DDRAM address in address counter
constexpr uint8_t CURSOR_OR_DISPLAY_SHIFT       = 0x10; //Set cursor moving and display shift control bit, and the direction without changing DDRAM data.
constexpr uint8_t SET_CGRAM_ADDRESS             = 0x40; //Set CGRAM address in address counter
constexpr uint8_t INTERNAL_OSC_FREQ             = 0x10; //BS=1:1/4 bias, BS=0:1/5 bias, F2~0: adjust internal OSC frequency for FR frequency.
constexpr uint8_t POWER_ICON_BOST_CONTR         = 0x50; //Ion: ICON display on/off, Bon: set booster circuit on/off, C5,C4: Contrast set
constexpr uint8_t FOLLOWER_CONTROL              = 0x60; //Fon: set follower circuit on/off, Rab2~0: select follower amplified ratio.
constexpr uint8_t CONTRAST_SET                  = 0x70; //C0-C3: Contrast set
constexpr uint8_t LINE_1_ADR                    = 0x80;
constexpr uint8_t LINE_2_ADR                    = 0xC0;

constexpr uint8_t ENTRY_MODE_SET_S              = 0x01; //S: Shift of entire display, see data sheet
constexpr uint8_t ENTRY_MODE_SET_ID             = 0x02; //I/D : Increment / decrement of DDRAM address (cursor or blink), see  data sheet
constexpr uint8_t DISPLAY_ON_OFF_B              = 0x01; //cursor position on
constexpr uint8_t DISPLAY_ON_OFF_C              = 0x02; //cursor on
constexpr uint8_t DISPLAY_ON_OFF_D              = 0x04; //display on
constexpr uint8_t FUNCTION_SET_IS               = 0x01; //IS: instruction table select
constexpr uint8_t FUNCTION_SET_DH               = 0x04; //DH: double height font
constexpr uint8_t FUNCTION_SET_N                = 0x08; //N: number of line is 2/1
constexpr uint8_t FUNCTION_SET_DL               = 0x10; //DL: interface data is 8/4 bits
constexpr uint8_t CURSOR_OR_DISPLAY_SHIFT_RL    = 0x04;
constexpr uint8_t CURSOR_OR_DISPLAY_SHIFT_SC    = 0x08;
constexpr uint8_t INTERNAL_OSC_FREQ_F0          = 0x01; //F2~0: adjust internal OSC frequency for FR frequency.
constexpr uint8_t INTERNAL_OSC_FREQ_F1          = 0x02; //F2~0: adjust internal OSC frequency for FR frequency.
constexpr uint8_t INTERNAL_OSC_FREQ_F2          = 0x04; //F2~0: adjust internal OSC  frequency for FR frequency.
constexpr uint8_t INTERNAL_OSC_FREQ_BS          = 0x08; //BS=1:1/4 bias (BS=0:1/5 bias)
constexpr uint8_t POWER_ICON_BOST_CONTR_Bon     = 0x04; //Ion: ICON display on/off
constexpr uint8_t POWER_ICON_BOST_CONTR_Ion     = 0x08; //Bon: set booster circuit on/off
constexpr uint8_t FOLLOWER_CONTROL_Rab0         = 0x01; //Rab2~0: select follower amplified ratio
constexpr uint8_t FOLLOWER_CONTROL_Rab1         = 0x02; //Rab2~0: select follower amplified ratio
constexpr uint8_t FOLLOWER_CONTROL_Rab2         = 0x04; //Rab2~0: select follower amplified ratio
constexpr uint8_t FOLLOWER_CONTROL_Fon          = 0x08; //Fon: set follower circuit on/off

constexpr uint8_t CONTRAST_MAX                  = 0x3F; //limit range max value (= 0x00; - = 0x3F;)
constexpr uint8_t CONTRAST_MIN                  = 0x00; //limit range min value (= 0x00; - = 0x3F;)

constexpr unsigned int WRITE_DELAY_MS           =    1; //see data sheet
constexpr unsigned int HOME_CLEAR_DELAY_MS      =    2; //see data sheet

constexpr unsigned int ST7032_I2C_TIMEOUT       =  100; //timeout value of I2C transmit API

class St7032{
  public:
    St7032(I2C_HandleTypeDef hi2cx):
      hi2cx_(hi2cx){}
    void Init();
    void Puts(const char *p);
    void Clear();
    void Home();
    void Display();
    void NoDisplay();
    void SetCursor(uint8_t line, uint8_t pos);
    void Cursor();
    void NoCursor();
    void Blink();
    void NoBlink();
    void SetContrast(int val);
    void AdjustContrast(int val);
    uint8_t GetContrast();
    virtual size_t Write(uint8_t chr);
  protected:
    void WriteInstruction(uint8_t cmd);
    void WriteData(uint8_t data);
  private:
    I2C_HandleTypeDef hi2cx_;
    uint8_t contrast_ = 0x18;
    uint8_t display_setting_ = (DISPLAY_ON_OFF | DISPLAY_ON_OFF_D);
};

inline uint8_t St7032::GetContrast(){
  return contrast_;
}

#endif /* INC_ST7032_HPP_ */
