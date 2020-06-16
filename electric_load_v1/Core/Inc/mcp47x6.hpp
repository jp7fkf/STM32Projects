/*
 * mcp47x6.hpp
 *
 *  Created on: May 29, 2020
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

#ifndef SRC_MCP47X6_HPP_
#define SRC_MCP47X6_HPP_

#include "stm32f0xx_hal.h"

constexpr unsigned int MCP47x6_I2C_ADDRESS   = 0x60; //timeout value of I2C transmit API
constexpr unsigned int MCP47x6_I2C_TIMEOUT   = 1000; //timeout value of I2C transmit API

class Mcp47x6 {
  public:
    Mcp47x6(I2C_HandleTypeDef hi2cx):
      hi2cx_(hi2cx){}
    void SetDacOut(uint16_t val);
  private:
    I2C_HandleTypeDef hi2cx_;
};

#endif /* SRC_MCP47X6_HPP_ */
