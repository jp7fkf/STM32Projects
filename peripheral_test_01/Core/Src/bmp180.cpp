/*
 * bmp180.cpp
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

#include "bmp180.hpp"

void Bmp180::Init(){
  this->LoadCalibrationParams();
}

bool Bmp180::TestConnection(){
    uint8_t buf[2];
    HAL_I2C_Master_Transmit(&hi2cx_, BMP180_ADDRESS << 1, (uint8_t *)&BMP180_RA_AC1_MSB, 1, BMP180_I2C_TIMEOUT);
    if (HAL_I2C_Master_Receive(&hi2cx_, (BMP180_ADDRESS << 1)|0x01, buf, 2, BMP180_I2C_TIMEOUT) == HAL_OK)
      return true;
    else
      return false;
}

void Bmp180::LoadCalibrationParams(){
  uint8_t buf[22];

  HAL_I2C_Master_Transmit(&hi2cx_, BMP180_ADDRESS << 1, (uint8_t *)&BMP180_RA_AC1_MSB, 1, BMP180_I2C_TIMEOUT);
  HAL_I2C_Master_Receive(&hi2cx_, (BMP180_ADDRESS << 1)|0x01, buf, 22, BMP180_I2C_TIMEOUT);

  ac1_ = ((int16_t)buf[0]   << 8) + buf[1];
  ac2_ = ((int16_t)buf[2]   << 8) + buf[3];
  ac3_ = ((int16_t)buf[4]   << 8) + buf[5];
  ac4_ = ((uint16_t)buf[6]  << 8) + buf[7];
  ac5_ = ((uint16_t)buf[8]  << 8) + buf[9];
  ac6_ = ((uint16_t)buf[10] << 8) + buf[11];
  b1_  = ((int16_t)buf[12]  << 8) + buf[13];
  b2_  = ((int16_t)buf[14]  << 8) + buf[15];
  mb_  = ((int16_t)buf[16]  << 8) + buf[17];
  mc_  = ((int16_t)buf[18]  << 8) + buf[19];
  md_  = ((int16_t)buf[20]  << 8) + buf[21];

  calibrationLoaded_ = true;
}

int16_t Bmp180::getAC1(){
  uint8_t buf[2];
  if (calibrationLoaded_)
    return ac1_;
  HAL_I2C_Master_Transmit(&hi2cx_, BMP180_ADDRESS << 1, (uint8_t *)&BMP180_RA_AC1_MSB, 1, BMP180_I2C_TIMEOUT);
  HAL_I2C_Master_Receive(&hi2cx_, (BMP180_ADDRESS << 1)|0x01, buf, 2, BMP180_I2C_TIMEOUT);
  return ((int16_t)buf[1] << 8) + buf[0];
}

int16_t Bmp180::getAC2(){
  uint8_t buf[2];
  if (calibrationLoaded_)
    return ac2_;
  HAL_I2C_Master_Transmit(&hi2cx_, BMP180_ADDRESS << 1, (uint8_t *)&BMP180_RA_AC2_MSB, 1, BMP180_I2C_TIMEOUT);
  HAL_I2C_Master_Receive(&hi2cx_, (BMP180_ADDRESS << 1)|0x01, buf, 2, BMP180_I2C_TIMEOUT);
  return ((int16_t)buf[1] << 8) + buf[0];
}

int16_t Bmp180::getAC3(){
  uint8_t buf[2];
  if (calibrationLoaded_)
    return ac3_;
  HAL_I2C_Master_Transmit(&hi2cx_, BMP180_ADDRESS << 1, (uint8_t *)&BMP180_RA_AC3_MSB, 1, BMP180_I2C_TIMEOUT);
  HAL_I2C_Master_Receive(&hi2cx_, (BMP180_ADDRESS << 1)|0x01, buf, 2, BMP180_I2C_TIMEOUT);
  return ((int16_t)buf[1] << 8) + buf[0];
}

uint16_t Bmp180::getAC4(){
  uint8_t buf[2];
  if (calibrationLoaded_)
    return ac4_;
  HAL_I2C_Master_Transmit(&hi2cx_, BMP180_ADDRESS << 1, (uint8_t *)&BMP180_RA_AC4_MSB, 1, BMP180_I2C_TIMEOUT);
  HAL_I2C_Master_Receive(&hi2cx_, (BMP180_ADDRESS << 1)|0x01, buf, 2, BMP180_I2C_TIMEOUT);
  return ((int16_t)buf[1] << 8) + buf[0];
}

uint16_t Bmp180::getAC5(){
  uint8_t buf[2];
  if (calibrationLoaded_)
    return ac5_;
  HAL_I2C_Master_Transmit(&hi2cx_, BMP180_ADDRESS << 1, (uint8_t *)&BMP180_RA_AC5_MSB, 1, BMP180_I2C_TIMEOUT);
  HAL_I2C_Master_Receive(&hi2cx_, (BMP180_ADDRESS << 1)|0x01, buf, 2, BMP180_I2C_TIMEOUT);
  return ((int16_t)buf[1] << 8) + buf[0];
}

uint16_t Bmp180::getAC6(){
  uint8_t buf[2];
  if (calibrationLoaded_)
    return ac6_;
  HAL_I2C_Master_Transmit(&hi2cx_, BMP180_ADDRESS << 1, (uint8_t *)&BMP180_RA_AC6_MSB, 1, BMP180_I2C_TIMEOUT);
  HAL_I2C_Master_Receive(&hi2cx_, (BMP180_ADDRESS << 1)|0x01, buf, 2, BMP180_I2C_TIMEOUT);
  return ((int16_t)buf[1] << 8) + buf[0];
}

int16_t Bmp180::getB1(){
  uint8_t buf[2];
  if (calibrationLoaded_)
    return b1_;
  HAL_I2C_Master_Transmit(&hi2cx_, BMP180_ADDRESS << 1, (uint8_t *)&BMP180_RA_B1_MSB, 1, BMP180_I2C_TIMEOUT);
  HAL_I2C_Master_Receive(&hi2cx_, (BMP180_ADDRESS << 1)|0x01, buf, 2, BMP180_I2C_TIMEOUT);
  return ((int16_t)buf[1] << 8) + buf[0];
}

int16_t Bmp180::getB2(){
  uint8_t buf[2];
  if (calibrationLoaded_)
    return b2_;
  HAL_I2C_Master_Transmit(&hi2cx_, BMP180_ADDRESS << 1, (uint8_t *)&BMP180_RA_B2_MSB, 1, BMP180_I2C_TIMEOUT);
  HAL_I2C_Master_Receive(&hi2cx_, (BMP180_ADDRESS << 1)|0x01, buf, 2, BMP180_I2C_TIMEOUT);
  return ((int16_t)buf[1] << 8) + buf[0];
}

int16_t Bmp180::getMB(){
  uint8_t buf[2];
  if (calibrationLoaded_)
    return mb_;
  HAL_I2C_Master_Transmit(&hi2cx_, BMP180_ADDRESS << 1, (uint8_t *)&BMP180_RA_MB_MSB, 1, BMP180_I2C_TIMEOUT);
  HAL_I2C_Master_Receive(&hi2cx_, (BMP180_ADDRESS << 1)|0x01, buf, 2, BMP180_I2C_TIMEOUT);
  return ((int16_t)buf[1] << 8) + buf[0];
}

int16_t Bmp180::getMC(){
  uint8_t buf[2];
  if (calibrationLoaded_)
    return mc_;
  HAL_I2C_Master_Transmit(&hi2cx_, BMP180_ADDRESS << 1, (uint8_t *)&BMP180_RA_MC_MSB, 1, BMP180_I2C_TIMEOUT);
  HAL_I2C_Master_Receive(&hi2cx_, (BMP180_ADDRESS << 1)|0x01, buf, 2, BMP180_I2C_TIMEOUT);
  return ((int16_t)buf[1] << 8) + buf[0];
}

int16_t Bmp180::getMD(){
  uint8_t buf[2];
  if (calibrationLoaded_)
    return md_;
  HAL_I2C_Master_Transmit(&hi2cx_, BMP180_ADDRESS << 1, (uint8_t *)&BMP180_RA_MD_MSB, 1, BMP180_I2C_TIMEOUT);
  HAL_I2C_Master_Receive(&hi2cx_, (BMP180_ADDRESS << 1)|0x01, buf, 2, BMP180_I2C_TIMEOUT);
  return ((int16_t)buf[1] << 8) + buf[0];
}

uint8_t Bmp180::getMeasureDelayMilliseconds(){
  if (mode_ == BMP180_MODE_TEMPERATURE)
    return 5;
  else if (mode_ == BMP180_MODE_PRESSURE_0)
    return 5;
  else if (mode_ == BMP180_MODE_PRESSURE_1)
    return 8;
  else if (mode_ == BMP180_MODE_PRESSURE_2)
    return 14;
  else if (mode_ == BMP180_MODE_PRESSURE_3)
    return 26;
  return 0;
}

uint16_t Bmp180::getMeasureDelayMicroseconds(){
  if (mode_ == BMP180_MODE_TEMPERATURE)
    return 4500;
  else if (mode_ == BMP180_MODE_PRESSURE_0)
    return 4500;
  else if (mode_ == BMP180_MODE_PRESSURE_1)
    return 7500;
  else if (mode_ == BMP180_MODE_PRESSURE_2)
    return 13500;
  else if (mode_ == BMP180_MODE_PRESSURE_3)
    return 25500;
  return 0;
}

uint8_t Bmp180::GetRegister(uint8_t reg){
  uint8_t buf;
  HAL_I2C_Master_Transmit(&hi2cx_, BMP180_ADDRESS << 1, &reg, 1, BMP180_I2C_TIMEOUT);
  HAL_I2C_Master_Receive(&hi2cx_, (BMP180_ADDRESS << 1)|0x01, &buf, 1, BMP180_I2C_TIMEOUT);
  return buf;
}

void Bmp180::setRegister(uint8_t reg, uint8_t val){
  uint8_t buf[2];
  buf[0] = reg;
  buf[1] = val;
  HAL_I2C_Master_Transmit(&hi2cx_, BMP180_ADDRESS << 1, buf, 2, BMP180_I2C_TIMEOUT);
}

void Bmp180::setMode(uint8_t mode){
  mode_ = mode;
  setRegister(BMP180_RA_CONTROL, mode_);
}

uint16_t Bmp180::getRawTemperature(){
  uint8_t buf[2];
  if (mode_ == BMP180_MODE_TEMPERATURE){
    HAL_I2C_Master_Transmit(&hi2cx_, BMP180_ADDRESS << 1, (uint8_t *)&BMP180_RA_RESULT_MSB, 1, BMP180_I2C_TIMEOUT);
    HAL_I2C_Master_Receive(&hi2cx_, (BMP180_ADDRESS << 1)|0x01, buf, 2, BMP180_I2C_TIMEOUT);
    return ((uint16_t)buf[0] << 8) + buf[1];
  }
  return 0;
}

uint32_t Bmp180::getRawPressure(){
  uint8_t buf[3];
  if ((mode_ & BMP180_MODE_PRESSURE_0) == BMP180_MODE_PRESSURE_0){
    HAL_I2C_Master_Transmit(&hi2cx_, BMP180_ADDRESS << 1, (uint8_t *)&BMP180_RA_RESULT_MSB, 1, BMP180_I2C_TIMEOUT);
    HAL_I2C_Master_Receive(&hi2cx_, (BMP180_ADDRESS << 1)|0x01, buf, 3, BMP180_I2C_TIMEOUT);
    return (((uint32_t)buf[0] << 16) + ((uint16_t)buf[1] << 8) + buf[2]) >> (8 - ((mode_ & 0xC0) >> 6));
  }
  return 0;
}

float Bmp180::GetTemperatureCelsius(){
  setMode(BMP180_MODE_TEMPERATURE);
  HAL_Delay(getMeasureDelayMilliseconds());
  uint16_t ut = getRawTemperature();
  int32_t x1 = ((ut - (int32_t)ac6_) * (int32_t)ac5_) >> 15;
  int32_t x2 = ((int32_t)mc_ << 11) / (x1 + md_);
  b5_ = x1 + x2;
  return (float)((b5_ + 8) >> 4) / 10.0f;
}

float Bmp180::GetTemperatureFahrenheit(){
   return GetTemperatureCelsius() * 9.0f / 5.0f + 32;
}

float Bmp180::GetPressure(uint8_t oss){
  switch(oss){
    case 0:
      setMode(BMP180_MODE_PRESSURE_0);
      break;
    case 1:
      setMode(BMP180_MODE_PRESSURE_1);
      break;
    case 2:
      setMode(BMP180_MODE_PRESSURE_2);
      break;
    case 3:
      setMode(BMP180_MODE_PRESSURE_3);
      break;
    default:
      setMode(BMP180_MODE_PRESSURE_3);
  }

  HAL_Delay(getMeasureDelayMilliseconds());
  uint32_t up = getRawPressure();

  int32_t p;
  int32_t b6 = b5_ - 4000;
  int32_t x1 = ((int32_t)b2_ * ((b6 * b6) >> 12)) >> 11;
  int32_t x2 = ((int32_t)ac2_ * b6) >> 11;
  int32_t x3 = x1 + x2;
  int32_t b3 = ((((int32_t)ac1_ * 4 + x3) << oss) + 2) >> 2;
  x1 = ((int32_t)ac3_ * b6) >> 13;
  x2 = ((int32_t)b1_ * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  uint32_t b4 = ((uint32_t)ac4_ * (uint32_t)(x3 + 32768)) >> 15;
  uint32_t b7 = ((uint32_t)up - b3) * (uint32_t)(50000UL >> oss);
  if (b7 < 0x80000000){
    p = (b7 << 1) / b4;
  }else{
    p = (b7 / b4) << 1;
  }
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  return p + ((x1 + x2 + (int32_t)3791) >> 4);
}

float Bmp180::CalcAltitude(float pressure, float seaLevelPressure)
{
    if (seaLevelPressure == 0) seaLevelPressure = 101325;
    return 44330 * (1.0 - pow(pressure / seaLevelPressure, 0.1903));
}

