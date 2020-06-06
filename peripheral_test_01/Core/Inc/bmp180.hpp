/*
 * bmp180.hpp
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

#ifndef BMP180_HPP_
#define BMP180_HPP_

#include "stm32f0xx_hal.h"
#include <math.h>

constexpr uint8_t BMP180_ADDRESS            = 0x77; // I2C Address
constexpr uint8_t BMP180_RA_AC1_MSB         = 0xAA; // AC1_MSB
constexpr uint8_t BMP180_RA_AC1_LSB         = 0xAB; // AC1_LSB
constexpr uint8_t BMP180_RA_AC2_MSB         = 0xAC; // AC2_MSB
constexpr uint8_t BMP180_RA_AC2_LSB         = 0xAD; // AC2_LSB
constexpr uint8_t BMP180_RA_AC3_MSB         = 0xAE; // AC3_MSB
constexpr uint8_t BMP180_RA_AC3_LSB         = 0xAF; // AC3_LSB
constexpr uint8_t BMP180_RA_AC4_MSB         = 0xB0; // AC4_MSB
constexpr uint8_t BMP180_RA_AC4_LSB         = 0xB1; // AC4_LSB
constexpr uint8_t BMP180_RA_AC5_MSB         = 0xB2; // AC5_MSB
constexpr uint8_t BMP180_RA_AC5_LSB         = 0xB3; // AC5_LSB
constexpr uint8_t BMP180_RA_AC6_MSB         = 0xB4; // AC6_MSB
constexpr uint8_t BMP180_RA_AC6_LSB         = 0xB5; // AC6_LSB
constexpr uint8_t BMP180_RA_B1_MSB          = 0xB6; // B1_MSB
constexpr uint8_t BMP180_RA_B1_LSB          = 0xB7; // B1_LSB
constexpr uint8_t BMP180_RA_B2_MSB          = 0xB8; // B2_MSB
constexpr uint8_t BMP180_RA_B2_LSB          = 0xB9; // B2_LSB
constexpr uint8_t BMP180_RA_MB_MSB          = 0xBA; // MB_MSB
constexpr uint8_t BMP180_RA_MB_LSB          = 0xBB; // MB_LSB
constexpr uint8_t BMP180_RA_MC_MSB          = 0xBC; // MC_MSB
constexpr uint8_t BMP180_RA_MC_LSB          = 0xBD; // MC_LSB
constexpr uint8_t BMP180_RA_MD_MSB          = 0xBE; // MD_MSB
constexpr uint8_t BMP180_RA_MD_LSB          = 0xBF; // MD_LSB
constexpr uint8_t BMP180_RA_CONTROL         = 0xF4; // CONTROL
constexpr uint8_t BMP180_RA_RESULT_MSB      = 0xF6; // MSB
constexpr uint8_t BMP180_RA_RESULT_LSB      = 0xF7; // LSB
constexpr uint8_t BMP180_RA_RESULT_XLSB     = 0xF8; // XLSB
constexpr uint8_t BMP180_RA_SOFTRESET       = 0xE0; // software reset if set 0xB6
constexpr uint8_t BMP180_RA_ID              = 0xD0;
constexpr uint8_t BMP180_MODE_TEMPERATURE   = 0x2E;
constexpr uint8_t BMP180_MODE_PRESSURE_0    = 0x34;
constexpr uint8_t BMP180_MODE_PRESSURE_1    = 0x74;
constexpr uint8_t BMP180_MODE_PRESSURE_2    = 0xB4;
constexpr uint8_t BMP180_MODE_PRESSURE_3    = 0xF4;

constexpr unsigned int BMP180_I2C_TIMEOUT   = 1000; //timeout value of I2C transmit API

class Bmp180 {
  public:
    Bmp180(I2C_HandleTypeDef hi2cx):
        hi2cx_(hi2cx),
        calibrationLoaded_(false),
        mode_(BMP180_MODE_TEMPERATURE){}
    void Init();
    bool TestConnection();
    void LoadCalibrationParams();
    uint8_t  GetRegister(uint8_t reg);
    float GetTemperatureCelsius();
    float GetTemperatureFahrenheit();
    float GetPressure(uint8_t oss);
    float CalcAltitude(float pressure, float seaLevelPressure);
  private:
    I2C_HandleTypeDef hi2cx_;
    bool calibrationLoaded_ = false;
    uint8_t  mode_;
    int16_t  ac1_;
    int16_t  ac2_;
    int16_t  ac3_;
    uint16_t ac4_;
    uint16_t ac5_;
    uint16_t ac6_;
    int16_t  b1_;
    int16_t  b2_;
    int32_t  b5_;
    int16_t  mb_;
    int16_t  mc_;
    int16_t  md_;
    int16_t  getAC1();
    int16_t  getAC2();
    int16_t  getAC3();
    uint16_t getAC4();
    uint16_t getAC5();
    uint16_t getAC6();
    int16_t  getB1();
    int16_t  getB2();
    int16_t  getMB();
    int16_t  getMC();
    int16_t  getMD();
    uint8_t  getMeasureDelayMilliseconds();
    uint16_t getMeasureDelayMicroseconds();
    void     setRegister(uint8_t reg, uint8_t val);
    void     setMode(uint8_t mode);
    uint16_t getRawTemperature();
    uint32_t getRawPressure();
};


#endif /* BMP180_HPP_ */
