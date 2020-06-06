/*
 * sht3x.cpp
 *
 *  Created on: May 28, 2020
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

#ifndef SHT3X_HPP_
#define SHT3X_HPP_

#include "stm32f0xx_hal.h"

constexpr uint8_t  SHT_I2C_ADDR0         = 0x44;
constexpr uint8_t  SHT_I2C_ADDR1         = 0x45;
constexpr uint8_t  CLOCK_STRETCH_ENABLE  = 0x2C;
constexpr uint8_t  CLOCK_STRETCH_DISABLE = 0x24;
constexpr unsigned int SHT_I2C_TIMEOUT   = 1000; //timeout value of I2C transmit API

class Sht3x {
  public:
    Sht3x(I2C_HandleTypeDef hi2cx):
      hi2cx_(hi2cx){}
    void GetTemperatureHumiditySingleShot(float *temp, float *humid);
  private:
    I2C_HandleTypeDef hi2cx_;
    float calculateTemperature(uint8_t msb, uint8_t lsb);
    float calculateHumidity(uint8_t msb, uint8_t lsb);
};

#endif /* SHT3X_HPP_ */
