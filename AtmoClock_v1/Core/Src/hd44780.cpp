/*
 * hd44780.cpp
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

#include "hd44780.hpp"

void Hd44780::Init(){
  HAL_Delay(15);
  write(0x03, 4); //init sequence
  HAL_Delay(5);
  write(0x03, 4); //init sequence
  HAL_Delay(1);
  write(0x03, 4); //init sequence
  HAL_Delay(1);
  write(0x02, 4); //set to 4bit mode

  //  this command can also set above configuration successfully.
  //  SendCommand(0x33); //init sequence
  //  SendCommand(0x32); //set to 4bit mode

  SendCommand(LCD_FUNCTIONSET | LCD_2LINE);
  SendCommand(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
  SendCommand(LCD_CLEARDISPLAY);
  SendCommand(LCD_ENTRYMODESET | LCD_ENTRYLEFT);
}

void Hd44780::SendData(uint8_t data){
  HAL_GPIO_WritePin(rsPort_, rsPin_, GPIO_PIN_SET); // Write to data register

  if(mode_ == LCD_4BITMODE){
    write(data >> 4, 4);
    write(data & 0x0F, 4);
  }else{
    write(data, 8);
  }
}

void Hd44780::SendCommand(uint8_t command){
  HAL_GPIO_WritePin(rsPort_, rsPin_, GPIO_PIN_RESET); // Write to command register

  if(mode_ == LCD_4BITMODE){
    write(command >> 4, 4);
    write(command & 0x0F, 4);
  }else{
    write(command, 8);
  }
}

void Hd44780::Puts(char* string)
{
  for(uint8_t i=0; i<strlen(string); i++){
    SendData(string[i]);
  }
}

void Hd44780::SetCursor(uint8_t row, uint8_t column){
  SendCommand(LCD_SETDDRAMADDR + 0x40*row + column);
}

void Hd44780::Clear(){
  SendCommand(LCD_CLEARDISPLAY);
}

void Hd44780::Home(){
  SendCommand(LCD_RETURNHOME);
  HAL_Delay(2);
}

void Hd44780::write(uint8_t data, uint8_t len)
{
  for(uint8_t i=0; i<len; i++){
    HAL_GPIO_WritePin(dataPorts_[i], dataPins_[i], (data >> i) & 0x01 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }

  HAL_GPIO_WritePin(enablePort_, enablePin_, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(enablePort_, enablePin_, GPIO_PIN_RESET);
}
