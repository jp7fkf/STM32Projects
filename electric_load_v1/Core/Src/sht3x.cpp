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

#include "sht3x.hpp"

void Sht3x::GetTemperatureHumiditySingleShot(float *temp, float *humid){
  uint8_t rxbuf[6];
  uint8_t settings[] = {0x2C, 0x06};

  HAL_I2C_Master_Transmit(&hi2cx_, (0x45<<1)|0x00, settings, 2, SHT_I2C_TIMEOUT);
  HAL_I2C_Master_Receive(&hi2cx_, (0x45<<1)|0x01, rxbuf, 6, SHT_I2C_TIMEOUT);

  if (temp != NULL)
    *temp = calculateTemperature(rxbuf[0], rxbuf[1]);
  if (humid != NULL)
    *humid = calculateHumidity(rxbuf[3], rxbuf[4]);
}

float Sht3x::calculateTemperature(uint8_t msb, uint8_t lsb){
  return (float)(msb << 8 | lsb) * 175 / 65535 - 45;
}

float Sht3x::calculateHumidity(uint8_t msb, uint8_t lsb){
  return (float)(msb << 8 | lsb) / 65535 * 100;
}
