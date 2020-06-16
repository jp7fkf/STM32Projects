/*
 * st7032.cpp
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

#include "st7032.hpp"

void St7032::Init(){
  HAL_Delay(100);
  WriteInstruction(FUNCTION_SET | FUNCTION_SET_DL | FUNCTION_SET_N | FUNCTION_SET_IS);
  WriteInstruction(INTERNAL_OSC_FREQ | INTERNAL_OSC_FREQ_BS | INTERNAL_OSC_FREQ_F2);
  WriteInstruction(POWER_ICON_BOST_CONTR | POWER_ICON_BOST_CONTR_Ion);
  SetContrast(contrast_);
  WriteInstruction(FOLLOWER_CONTROL | FOLLOWER_CONTROL_Fon | FOLLOWER_CONTROL_Rab2);
  HAL_Delay(300);
  WriteInstruction(display_setting_);
  WriteInstruction(ENTRY_MODE_SET | ENTRY_MODE_SET_ID);
  this->Clear();
  this->Home();
}

void St7032::WriteInstruction(uint8_t cmd){
  uint8_t buf[] = {CNTRBIT_CO, cmd};
  HAL_I2C_Master_Transmit(&hi2cx_, Write_Address<<1, buf, 2, ST7032_I2C_TIMEOUT);
  HAL_Delay(WRITE_DELAY_MS);
}

void St7032::WriteData(uint8_t data){
  uint8_t buf[] = {CNTRBIT_RS, data};
  HAL_I2C_Master_Transmit(&hi2cx_, Write_Address<<1, buf, 2, ST7032_I2C_TIMEOUT);
  HAL_Delay(WRITE_DELAY_MS);
}

size_t St7032::Write(uint8_t chr){
  this->WriteData(chr);
  return 1;
}

void St7032::Puts(const char *p){
  for (; *p; p++){
    this->WriteData(*p);
  }
}

void St7032::Clear(){
  this->WriteInstruction(CLEAR_DISPLAY);
  HAL_Delay(HOME_CLEAR_DELAY_MS);
}

void St7032::Home(){
  this->WriteInstruction(RETURN_HOME);
  HAL_Delay(HOME_CLEAR_DELAY_MS);
}

void St7032::SetCursor(uint8_t line, uint8_t pos){
  uint8_t p;
  if(pos > 15) pos = 0;
  if(line == 0) p = LINE_1_ADR + pos;
  else p = LINE_2_ADR + pos;
  WriteInstruction(SET_DDRAM_ADDRESS | p);
}

void St7032::Display(){
  display_setting_ |= DISPLAY_ON_OFF_D;
  WriteInstruction(display_setting_);
}

void St7032::NoDisplay(){
  display_setting_ &= ~DISPLAY_ON_OFF_D;
  WriteInstruction(display_setting_);
}

void St7032::Cursor(){
  display_setting_ |= DISPLAY_ON_OFF_C;
  WriteInstruction(display_setting_);
}

void St7032::NoCursor(){
  display_setting_ &= ~DISPLAY_ON_OFF_C;
  WriteInstruction(display_setting_);
}

void St7032::Blink(){
  display_setting_ |= DISPLAY_ON_OFF_B;
  WriteInstruction(display_setting_);
}

void St7032::NoBlink(){
  display_setting_ &= ~DISPLAY_ON_OFF_B;
  WriteInstruction(display_setting_);
}

void St7032::SetContrast(int val){
  if (val > CONTRAST_MAX) val = CONTRAST_MIN;
  else if (val < CONTRAST_MIN) val = CONTRAST_MAX;
  WriteInstruction(CONTRAST_SET | (val & 0b00001111));
  WriteInstruction((val >> 4) | POWER_ICON_BOST_CONTR | POWER_ICON_BOST_CONTR_Bon);
  contrast_ = val;
}

void St7032::AdjustContrast(int val){
  SetContrast(val + contrast_);
}
