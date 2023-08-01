/**
 ******************************************************************************
 * @file    LIS2DUXS12Sensor.cpp
 * @author  CLab
 * @version V1.0.0
 * @date    15 November 2018
 * @brief   Implementation of an LIS2DUXS12 Inertial Measurement Unit (IMU) 3 axes
 *          sensor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "LIS2DUXS12Sensor.h"
/* Class Implementation ------------------------------------------------------*/
/** Constructor
 * @param i2c object of an helper class which handles the I2C peripheral
 * @param address the address of the component's instance
 */
LIS2DUXS12Sensor::LIS2DUXS12Sensor(TwoWire *i2c, uint8_t address) : dev_i2c(i2c), address(address)
{
  dev_spi = NULL;
  reg_ctx.write_reg = LIS2DUXS12_io_write;
  reg_ctx.read_reg = LIS2DUXS12_io_read;
  reg_ctx.handle = (void *)this;
  X_isInitialized = 0;
  X_isEnabled = 0;
}

/** Constructor
 * @param spi object of an helper class which handles the SPI peripheral
 * @param cs_pin the chip select pin
 * @param spi_speed the SPI speed
 */
LIS2DUXS12Sensor::LIS2DUXS12Sensor(SPIClass *spi, int cs_pin, uint32_t spi_speed) : dev_spi(spi), cs_pin(cs_pin), spi_speed(spi_speed)
{
  reg_ctx.write_reg = LIS2DUXS12_io_write;
  reg_ctx.read_reg = LIS2DUXS12_io_read;
  reg_ctx.handle = (void *)this;
  dev_i2c = NULL;
  address = 0;
  X_isInitialized = 0;
  X_isEnabled = 0;
}

/**
  * @brief  Exit from deep power down in SPI
  * @param  pObj the device pObj
  * @retval 0 in case of success, an error code otherwise
  */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor:: ExitDeepPowerDownSPI(void)
{
  /* Write IF_WAKE_UP register to exit from deep power down in SPI mode*/
  (void)lis2duxs12_exit_deep_power_down(&(reg_ctx));
  /* Wait for 25 ms based on datasheet */
  delay(25);
  return LIS2DUXS12_STATUS_OK;
}

/**
  * @brief  Exit from deep power down in I2C
  * @param  pObj the device pObj
  * @retval 0 in case of success, an error code otherwise
  */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor:: ExitDeepPowerDownI2C(void)
{
  /* Perform dummy read */
  uint8_t id;
  lis2duxs12_device_id_get(&reg_ctx, &id);
  /* Wait for 25 ms based on datasheet */
  delay(25);
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Configure the sensor in order to be used
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::begin()
{
  if (dev_spi) {
    // Configure CS pin
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);
    /* Exit from deep power down only the first time in SPI mode */
    ExitDeepPowerDownSPI();
  }
  if (dev_i2c) {
    /* Exit from deep power down only the first time in I2C mode */
    ExitDeepPowerDownI2C();
  }
  /* Enable register address automatically incremented during a multiple byte
  access with a serial interface. Enable BDU. */
  if (lis2duxs12_init_set(&(reg_ctx), LIS2DUXS12_SENSOR_ONLY_ON) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* FIFO mode selection */
  lis2duxs12_fifo_mode_t fifo_mode = {
    .operation = LIS2DUXS12_BYPASS_MODE,
    .store     = LIS2DUXS12_FIFO_1X,
    .xl_only = 1,
    .watermark = 0,
  };
  if (lis2duxs12_fifo_mode_set(&(reg_ctx), fifo_mode) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Select default output data rate. */
  X_Last_ODR = 100.0f;
  /* Select default ultra low power (disabled). */
  X_Last_Operating_Mode = LIS2DUXS12_LOW_POWER;
  /* Output data rate: power down, full scale: 2g */
  lis2duxs12_md_t mode = {
    .odr = LIS2DUXS12_OFF,
    .fs  = LIS2DUXS12_2g,
  };
  if (lis2duxs12_mode_set(&(reg_ctx), &mode) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  X_isInitialized = 1;
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Disable the sensor and relative resources
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::end()
{
  /* Disable acc */
  if (Disable_X() != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Reset CS configuration */
  if (dev_spi) {
    // Configure CS pin
    pinMode(cs_pin, INPUT);
  }
  /* Select default output data rate. */
  X_Last_ODR = 0.0f;
  /* Select default ultra low power (disabled). */
  X_Last_Operating_Mode = LIS2DUXS12_LOW_POWER;
  X_isInitialized = 0;
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Enable LIS2DUXS12 Accelerator
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Enable_X(void)
{
  /* Check if the component is already enabled */
  if (X_isEnabled == 1) {
    return LIS2DUXS12_STATUS_OK;
  }

  /* Output data rate selection. */
  if (Set_X_ODR_When_Enabled(X_Last_ODR, X_Last_Operating_Mode) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }

  X_isEnabled = 1;

  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Disable LIS2DUXS12 Accelerator
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Disable_X(void)
{
  float_t Odr;
  /* Check if the component is already disabled */
  if (X_isEnabled == 0) {
    return LIS2DUXS12_STATUS_OK;
  }
  if (Get_X_ODR(&Odr) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (Odr == 800.0f) {
    if (Set_X_ODR(400.0f) != LIS2DUXS12_STATUS_OK) {
      return LIS2DUXS12_STATUS_ERROR;
    }
    /* Wait for 3 ms based on datasheet */
    delay(3);
  }
  /* Output data rate selection - power down. */
  lis2duxs12_md_t mode;
  if (lis2duxs12_mode_get(&(reg_ctx), &mode) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  mode.odr = LIS2DUXS12_OFF;
  if (lis2duxs12_mode_set(&(reg_ctx), &mode) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }

  X_isEnabled = 0;

  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Read ID of LIS2DUXS12 Accelerometer and Gyroscope
 * @param  id the pointer where the ID of the device is stored
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::ReadID(uint8_t *id)
{
  if (!id) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Read WHO AM I register */
  if (lis2duxs12_device_id_get(&reg_ctx, id) != 0) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Read data from LIS2DUXS12 Accelerometer
 * @param  Acceleration the pointer where the accelerometer data are stored
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Get_X_Axes(int32_t *Acceleration)
{
  lis2duxs12_md_t mode;
  lis2duxs12_xl_data_t data;
  if (lis2duxs12_mode_get(&(reg_ctx), &mode) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_xl_data_get(&(reg_ctx), &mode, &data) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  Acceleration[0] = (int32_t)data.mg[0];
  Acceleration[1] = (int32_t)data.mg[1];
  Acceleration[2] = (int32_t)data.mg[2];
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Read Accelerometer Sensitivity
 * @param  Sensitivity the pointer where the accelerometer Sensitivity is stored
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Get_X_Sensitivity(float_t *Sensitivity)
{
  LIS2DUXS12StatusTypeDef ret = LIS2DUXS12_STATUS_OK;
  lis2duxs12_md_t mode;
  if (lis2duxs12_mode_get(&(reg_ctx), &mode) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  switch (mode.fs) {
    case LIS2DUXS12_2g:
      *Sensitivity = LIS2DUXS12_X_SENSITIVITY_FOR_FS_2G;
      break;
    case LIS2DUXS12_4g:
      *Sensitivity = LIS2DUXS12_X_SENSITIVITY_FOR_FS_4G;
      break;
    case LIS2DUXS12_8g:
      *Sensitivity = LIS2DUXS12_X_SENSITIVITY_FOR_FS_8G;
      break;
    case LIS2DUXS12_16g:
      *Sensitivity = LIS2DUXS12_X_SENSITIVITY_FOR_FS_16G;
      break;
    default:
      *Sensitivity = -1.0f;
      ret = LIS2DUXS12_STATUS_ERROR;
      break;
  }
  return ret;
}

/**
 * @brief  Read raw data from LIS2DUXS12 Accelerometer
 * @param  Value the pointer where the accelerometer raw data are stored
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Get_X_AxesRaw(int32_t *Value)
{
  lis2duxs12_md_t mode;
  lis2duxs12_xl_data_t data;
  if (lis2duxs12_mode_get(&(reg_ctx), &mode) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_xl_data_get(&(reg_ctx), &mode, &data) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  Value[0] = data.raw[0];
  Value[1] = data.raw[1];
  Value[2] = data.raw[2];
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Read LIS2DUXS12 Accelerometer output data rate
 * @param  odr the pointer to the output data rate
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Get_X_ODR(float *odr)
{
  LIS2DUXS12StatusTypeDef ret = LIS2DUXS12_STATUS_OK;
  lis2duxs12_md_t mode;
  /* Read actual output data rate from sensor. */
  if (lis2duxs12_mode_get(&(reg_ctx), &mode) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  switch (mode.odr) {
    case LIS2DUXS12_OFF:
    case LIS2DUXS12_TRIG_PIN:
    case LIS2DUXS12_TRIG_SW:
      *odr = 0.0f;
      break;
    case LIS2DUXS12_1Hz6_ULP:
      *odr = 1.6f;
      break;
    case LIS2DUXS12_3Hz_ULP:
      *odr = 3.0f;
      break;
    case LIS2DUXS12_6Hz_LP:
    case LIS2DUXS12_6Hz_HP:
      *odr = 6.0f;
      break;
    case LIS2DUXS12_12Hz5_LP:
    case LIS2DUXS12_12Hz5_HP:
      *odr = 12.5f;
      break;
    case LIS2DUXS12_25Hz_ULP:
    case LIS2DUXS12_25Hz_LP:
    case LIS2DUXS12_25Hz_HP:
      *odr = 25.0f;
      break;
    case LIS2DUXS12_50Hz_LP:
    case LIS2DUXS12_50Hz_HP:
      *odr = 50.0f;
      break;
    case LIS2DUXS12_100Hz_LP:
    case LIS2DUXS12_100Hz_HP:
      *odr = 100.0f;
      break;
    case LIS2DUXS12_200Hz_LP:
    case LIS2DUXS12_200Hz_HP:
      *odr = 200.0f;
      break;
    case LIS2DUXS12_400Hz_LP:
    case LIS2DUXS12_400Hz_HP:
      *odr = 400.0f;
      break;
    case LIS2DUXS12_800Hz_LP:
    case LIS2DUXS12_800Hz_HP:
      *odr = 800.0f;
      break;
    default:
      *odr = -1.0f;
      ret = LIS2DUXS12_STATUS_ERROR;
      break;
  }
  return ret;
}

/**
 * @brief  Set LIS2DUXS12 Accelerometer output data rate
 * @param  odr the output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Set_X_ODR(float_t odr)
{
  return Set_X_ODR_With_Mode(odr, LIS2DUXS12_LOW_POWER);
}

/**
 * @brief  Set LIS2DUXS12 Accelerometer output data rate
 * @param  odr the output data rate to be set
 * @param  mode the operating mode to be used
 * @param  noise the low noise option
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Set_X_ODR_With_Mode(float_t odr, LIS2DUXS12_Power_Mode_t Power)
{
  if (X_isEnabled == 1) {
    if (Set_X_ODR_When_Enabled(odr, Power) != LIS2DUXS12_STATUS_OK) {
      return LIS2DUXS12_STATUS_ERROR;
    }
  } else {
    if (Set_X_ODR_When_Disabled(odr, Power) != LIS2DUXS12_STATUS_OK) {
      return LIS2DUXS12_STATUS_ERROR;
    }
  }

  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Set LIS2DUXS12 Accelerometer output data rate when enabled
 * @param  odr the output data rate to be set
 * @param  mode the operating mode to be used
 * @param  noise the low noise option
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Set_X_ODR_When_Enabled(float_t Odr, LIS2DUXS12_Power_Mode_t Power)
{
  lis2duxs12_md_t mode;
  if (lis2duxs12_mode_get(&(reg_ctx), &mode) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (Power == LIS2DUXS12_ULTRA_LOW_POWER) {
    mode.odr = (Odr <= 1.6f) ? LIS2DUXS12_1Hz6_ULP
               : (Odr <= 3.0f) ? LIS2DUXS12_3Hz_ULP
               :                 LIS2DUXS12_25Hz_ULP;
  } else if (Power == LIS2DUXS12_LOW_POWER) {
    mode.odr = (Odr <=   6.0f) ? LIS2DUXS12_6Hz_LP
               : (Odr <=  12.5f) ? LIS2DUXS12_12Hz5_LP
               : (Odr <=  25.0f) ? LIS2DUXS12_25Hz_LP
               : (Odr <=  50.0f) ? LIS2DUXS12_50Hz_LP
               : (Odr <= 100.0f) ? LIS2DUXS12_100Hz_LP
               : (Odr <= 200.0f) ? LIS2DUXS12_200Hz_LP
               : (Odr <= 400.0f) ? LIS2DUXS12_400Hz_LP
               :                   LIS2DUXS12_800Hz_LP;
  } else if (Power == LIS2DUXS12_HIGH_PERFORMANCE) {
    mode.odr = (Odr <=   6.0f) ? LIS2DUXS12_6Hz_HP
               : (Odr <=  12.5f) ? LIS2DUXS12_12Hz5_HP
               : (Odr <=  25.0f) ? LIS2DUXS12_25Hz_HP
               : (Odr <=  50.0f) ? LIS2DUXS12_50Hz_HP
               : (Odr <= 100.0f) ? LIS2DUXS12_100Hz_HP
               : (Odr <= 200.0f) ? LIS2DUXS12_200Hz_HP
               : (Odr <= 400.0f) ? LIS2DUXS12_400Hz_HP
               :                   LIS2DUXS12_800Hz_HP;
  } else {
    /* Do nothing */
  }
  if (lis2duxs12_mode_set(&(reg_ctx), &mode) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Store the current Odr Value */
  X_Last_ODR = (mode.odr == LIS2DUXS12_1Hz6_ULP) ?   1.6f
               : (mode.odr == LIS2DUXS12_3Hz_ULP)  ?   3.0f
               : (mode.odr == LIS2DUXS12_6Hz_LP)      ?   6.0f
               : (mode.odr == LIS2DUXS12_6Hz_HP)   ?   6.0f
               : (mode.odr == LIS2DUXS12_12Hz5_LP)    ?  12.5f
               : (mode.odr == LIS2DUXS12_12Hz5_HP) ?  12.5f
               : (mode.odr == LIS2DUXS12_25Hz_ULP) ?  25.0f
               : (mode.odr == LIS2DUXS12_25Hz_LP)     ?  25.0f
               : (mode.odr == LIS2DUXS12_25Hz_HP)  ?  25.0f
               : (mode.odr == LIS2DUXS12_50Hz_LP)     ?  50.0f
               : (mode.odr == LIS2DUXS12_50Hz_HP)  ?  50.0f
               : (mode.odr == LIS2DUXS12_100Hz_LP)    ? 100.0f
               : (mode.odr == LIS2DUXS12_100Hz_HP) ? 100.0f
               : (mode.odr == LIS2DUXS12_200Hz_LP)    ? 200.0f
               : (mode.odr == LIS2DUXS12_200Hz_HP) ? 200.0f
               : (mode.odr == LIS2DUXS12_400Hz_LP)    ? 400.0f
               : (mode.odr == LIS2DUXS12_400Hz_HP) ? 400.0f
               : (mode.odr == LIS2DUXS12_800Hz_LP)    ? 800.0f
               : (mode.odr == LIS2DUXS12_800Hz_HP) ? 800.0f
               :                                     -1.0f;
  if (X_Last_ODR == -1.0f) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Store the current Power Value */
  X_Last_Operating_Mode = Power;
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Set LIS2DUXS12 Accelerometer output data rate when disabled
 * @param  odr the output data rate to be set
 * @param  mode the operating mode to be used
 * @param  noise the low noise option
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Set_X_ODR_When_Disabled(float_t Odr, LIS2DUXS12_Power_Mode_t Power)
{
  /* Store the new Odr Value */
  if (Power == LIS2DUXS12_ULTRA_LOW_POWER) {
    X_isEnabled = (Odr <= 1.5f) ? 1.5f
                  : (Odr <= 3.0f) ? 3.0f
                  :                25.0f;
  } else if ((Power == LIS2DUXS12_LOW_POWER) || (Power == LIS2DUXS12_HIGH_PERFORMANCE)) {
    X_isEnabled = (Odr <=   6.0f) ?   6.0f
                  : (Odr <=  12.5f) ?  12.5f
                  : (Odr <=  25.0f) ?  25.0f
                  : (Odr <=  50.0f) ?  50.0f
                  : (Odr <= 100.0f) ? 100.0f
                  : (Odr <= 200.0f) ? 200.0f
                  : (Odr <= 400.0f) ? 400.0f
                  :                   800.0f;
  } else {
    /* Do nothing */
  }
  /* Store the new Power Value */
  X_Last_Operating_Mode = Power;
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Read LIS2DUXS12 Accelerometer full scale
 * @param  full_scale the pointer to the full scale
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Get_X_FS(int32_t *FullScale)
{
  LIS2DUXS12StatusTypeDef ret  = LIS2DUXS12_STATUS_OK;
  lis2duxs12_md_t mode;
  /* Read actual full scale selection from sensor. */
  if (lis2duxs12_mode_get(&(reg_ctx), &mode) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  switch (mode.fs) {
    case LIS2DUXS12_2g:
      *FullScale =  2;
      break;
    case LIS2DUXS12_4g:
      *FullScale =  4;
      break;
    case LIS2DUXS12_8g:
      *FullScale =  8;
      break;
    case LIS2DUXS12_16g:
      *FullScale = 16;
      break;
    default:
      *FullScale = -1;
      ret = LIS2DUXS12_STATUS_ERROR;
      break;
  }
  return ret;
}

/**
 * @brief  Set LIS2DUXS12 Accelerometer full scale
 * @param  full_scale the full scale to be set
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Set_X_FS(int32_t FullScale)
{
  lis2duxs12_md_t mode;
  if (lis2duxs12_mode_get(&(reg_ctx), &mode) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Seems like MISRA C-2012 rule 14.3a violation but only from single file statical analysis point of view because
     the parameter passed to the function is not known at the moment of analysis */
  mode.fs = (FullScale <= 2) ? LIS2DUXS12_2g
            : (FullScale <= 4) ? LIS2DUXS12_4g
            : (FullScale <= 8) ? LIS2DUXS12_8g
            :                    LIS2DUXS12_16g;
  if (lis2duxs12_mode_set(&(reg_ctx), &mode) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Read LIS2DUXS12 DRDY status
 * @param  val pointer where the value is written
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Get_X_DRDY_Status(uint8_t *val)
{
  lis2duxs12_status_register_t status;
  if (lis2duxs12_read_reg(&reg_ctx, LIS2DUXS12_STATUS, (uint8_t *)&status, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  *val = status.drdy;
  return LIS2DUXS12_STATUS_OK;
}


/**
 * @brief  Get the LIS2DUXS12 FIFO number of samples
 * @param  NumSamples number of samples
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Get_FIFO_Num_Samples(uint16_t *NumSamples)
{
  if (lis2duxs12_fifo_data_level_get(&reg_ctx, NumSamples) != 0) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Get the LIS2DUXS12 FIFO full status
 * @param  Status FIFO full status
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Get_FIFO_Full_Status(uint8_t *Status)
{
  uint16_t NumSamples;
  if (lis2duxs12_fifo_data_level_get(&reg_ctx, &NumSamples) != 0) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  NumSamples == 128 ? *Status = 1 : *Status = 0;
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Get the LIS2DUXS12 FIFO overrun status
 * @param  Status FIFO overrun status
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Get_FIFO_Overrun_Status(uint8_t *Status)
{
  lis2duxs12_fifo_status1_t status;
  if (lis2duxs12_read_reg(&reg_ctx, LIS2DUXS12_FIFO_STATUS1, (uint8_t *)&status, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  *Status = status.fifo_ovr_ia;
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Get the LIS2DUXS12 FIFO watermark status
 * @param  Status FIFO watermark status
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Get_FIFO_Watermark_Status(uint8_t *Status)
{
  LIS2DUXS12StatusTypeDef ret = LIS2DUXS12_STATUS_OK;
  if (lis2duxs12_fifo_wtm_flag_get(&reg_ctx, Status) != 0) {
    ret = LIS2DUXS12_STATUS_ERROR;
  }
  return ret;
}

/**
 * @brief  Set the LIS2DUXS12 FIFO full interrupt on INT1 pin
 * @param  Status FIFO full interrupt on INT1 pin status
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Set_FIFO_INT1_FIFO_Full(uint8_t Status)
{
  lis2duxs12_pin_int_route_t val;
  lis2duxs12_int_config_t int_conf;
  LIS2DUXS12StatusTypeDef ret = LIS2DUXS12_STATUS_OK;
  if (lis2duxs12_pin_int1_route_get(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  val.fifo_full = Status;
  if (lis2duxs12_pin_int1_route_set(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_int_config_get(&reg_ctx, &int_conf) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  int_conf.int_cfg = LIS2DUXS12_INT_LEVEL;
  if (lis2duxs12_int_config_set(&reg_ctx, &int_conf) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  return ret;
}

/**
 * @brief  Set the LIS2DUXS12 FIFO overrun interrupt on INT2 pin
 * @param  Status FIFO overrun interrupt on INT2 pin status
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Set_FIFO_INT1_FIFO_Overrun(uint8_t Status)
{
  lis2duxs12_pin_int_route_t val;
  lis2duxs12_int_config_t int_conf;
  LIS2DUXS12StatusTypeDef ret = LIS2DUXS12_STATUS_OK;
  if (lis2duxs12_pin_int1_route_get(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  val.fifo_ovr = Status;
  if (lis2duxs12_pin_int1_route_set(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_int_config_get(&reg_ctx, &int_conf) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  int_conf.int_cfg = LIS2DUXS12_INT_LEVEL;
  if (lis2duxs12_int_config_set(&reg_ctx, &int_conf) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  return ret;
}

/**
 * @brief  Set the LIS2DUXS12 FIFO threshold interrupt on INT1 pin
 * @param  Status FIFO threshold interrupt on INT1 pin status
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Set_FIFO_INT1_FIFO_Threshold(uint8_t Status)
{
  lis2duxs12_pin_int_route_t val;
  lis2duxs12_int_config_t int_conf;
  LIS2DUXS12StatusTypeDef ret = LIS2DUXS12_STATUS_OK;
  if (lis2duxs12_pin_int1_route_get(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  val.fifo_th = Status;
  if (lis2duxs12_pin_int1_route_set(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_int_config_get(&reg_ctx, &int_conf) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  int_conf.int_cfg = LIS2DUXS12_INT_LEVEL;
  if (lis2duxs12_int_config_set(&reg_ctx, &int_conf) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  return ret;
}

/**
 * @brief  Set the LIS2DUXS12 FIFO full interrupt on INT2 pin
 * @param  Status FIFO full interrupt on INT2 pin status
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Set_FIFO_INT2_FIFO_Full(uint8_t Status)
{
  lis2duxs12_pin_int_route_t val;
  lis2duxs12_int_config_t int_conf;

  LIS2DUXS12StatusTypeDef ret = LIS2DUXS12_STATUS_OK;
  if (lis2duxs12_pin_int2_route_get(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  val.fifo_full = Status;
  if (lis2duxs12_pin_int2_route_set(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_int_config_get(&reg_ctx, &int_conf) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  int_conf.int_cfg = LIS2DUXS12_INT_LEVEL;
  if (lis2duxs12_int_config_set(&reg_ctx, &int_conf) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  return ret;
}

/**
 * @brief  Set the LIS2DUXS12 FIFO overrun interrupt on INT2 pin
 * @param  Status FIFO overrun interrupt on INT2 pin status
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Set_FIFO_INT2_FIFO_Overrun(uint8_t Status)
{
  lis2duxs12_pin_int_route_t val;
  lis2duxs12_int_config_t int_conf;

  LIS2DUXS12StatusTypeDef ret = LIS2DUXS12_STATUS_OK;
  if (lis2duxs12_pin_int2_route_get(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  val.fifo_full = Status;
  if (lis2duxs12_pin_int2_route_set(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_int_config_get(&reg_ctx, &int_conf) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  int_conf.int_cfg = LIS2DUXS12_INT_LEVEL;
  if (lis2duxs12_int_config_set(&reg_ctx, &int_conf) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  return ret;
}

/**
 * @brief  Set the LIS2DUXS12 FIFO threshold interrupt on INT2 pin
 * @param  Status FIFO threshold interrupt on INT2 pin status
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Set_FIFO_INT2_FIFO_Threshold(uint8_t Status)
{
  lis2duxs12_pin_int_route_t val;
  lis2duxs12_int_config_t int_conf;

  LIS2DUXS12StatusTypeDef ret = LIS2DUXS12_STATUS_OK;
  if (lis2duxs12_pin_int2_route_get(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  val.fifo_th = Status;
  if (lis2duxs12_pin_int2_route_set(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_int_config_get(&reg_ctx, &int_conf) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  int_conf.int_cfg = LIS2DUXS12_INT_LEVEL;
  if (lis2duxs12_int_config_set(&reg_ctx, &int_conf) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  return ret;
}

/**
 * @brief  Set the LIS2DUXS12 FIFO mode
 * @param  Mode FIFO mode
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Set_FIFO_Watermark_Level(uint8_t Watermark)
{
  lis2duxs12_fifo_wtm_t fifo_wtm;
  if (lis2duxs12_read_reg(&reg_ctx, LIS2DUXS12_FIFO_WTM, (uint8_t *)&fifo_wtm, 1) != 0) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  fifo_wtm.fth = Watermark;
  if (lis2duxs12_write_reg(&reg_ctx, LIS2DUXS12_FIFO_WTM, (uint8_t *)&fifo_wtm, 1) != 0) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Set the LIS2DUXS12 FIFO stop on watermark
 * @param  Status FIFO stop on watermark status
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Set_FIFO_Stop_On_Fth(uint8_t Status)
{
  lis2duxs12_fifo_ctrl_t fifo_ctrl;
  if (lis2duxs12_read_reg(&reg_ctx, LIS2DUXS12_FIFO_CTRL, (uint8_t *)&fifo_ctrl, 1) != 0) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  fifo_ctrl.stop_on_fth = Status;
  if (lis2duxs12_write_reg(&reg_ctx, LIS2DUXS12_FIFO_CTRL, (uint8_t *)&fifo_ctrl, 1) != 0) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Set the FIFO mode
 * @param  mode FIFO mode
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Set_FIFO_Mode(uint8_t op)
{
  LIS2DUXS12StatusTypeDef ret = LIS2DUXS12_STATUS_OK;
  lis2duxs12_fifo_mode_t mode ;
  if (lis2duxs12_fifo_mode_get(&reg_ctx, &mode) != 0) {
    ret = LIS2DUXS12_STATUS_ERROR;
  }
  /* Verify that the passed parameter contains one of the valid Values. */
  switch (op) {
    case LIS2DUXS12_BYPASS_MODE:
      mode.operation = LIS2DUXS12_BYPASS_MODE;
      break;
    case LIS2DUXS12_FIFO_MODE:
      mode.operation = LIS2DUXS12_FIFO_MODE;
      break;
    case LIS2DUXS12_STREAM_TO_FIFO_MODE:
      mode.operation = LIS2DUXS12_STREAM_TO_FIFO_MODE;
      break;
    case LIS2DUXS12_BYPASS_TO_STREAM_MODE:
      mode.operation = LIS2DUXS12_BYPASS_TO_STREAM_MODE;
      break;
    case LIS2DUXS12_STREAM_MODE:
      mode.operation = LIS2DUXS12_STREAM_MODE;
      break;
    default:
      ret = LIS2DUXS12_STATUS_ERROR;
      break;
  }
  if (ret == LIS2DUXS12_STATUS_ERROR) {
    return ret;
  }
  mode.watermark = 0x0U;

  if (lis2duxs12_fifo_mode_set(&reg_ctx, mode) != 0) {
    ret = LIS2DUXS12_STATUS_ERROR;
  }

  return ret;
}

/**
 * @brief  Get the LIS2DUXS12 FIFO tag
 * @param  Tag FIFO tag
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Get_FIFO_Tag(uint8_t *tag)
{
  lis2duxs12_fifo_sensor_tag_t tmp;
  if (lis2duxs12_fifo_sensor_tag_get(&reg_ctx, &tmp) != 0) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  *tag = (uint8_t) tmp;
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Set the LIS2DUXS12 FIFO accelero BDR value
 * @param  Bdr FIFO accelero BDR value
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Set_FIFO_X_BDR(uint8_t Bdr)
{
  LIS2DUXS12StatusTypeDef ret = LIS2DUXS12_STATUS_OK;
  lis2duxs12_fifo_mode_t mode ;
  if (lis2duxs12_fifo_mode_get(&reg_ctx, &mode) != 0) {
    ret = LIS2DUXS12_STATUS_ERROR;
  }
  switch (Bdr) {
    case LIS2DUXS12_BDR_XL_ODR:
      mode.batch.bdr_xl = LIS2DUXS12_BDR_XL_ODR;
      break;
    case LIS2DUXS12_BDR_XL_ODR_DIV_2:
      mode.batch.bdr_xl = LIS2DUXS12_BDR_XL_ODR_DIV_2;
      break;
    case LIS2DUXS12_BDR_XL_ODR_DIV_4:
      mode.batch.bdr_xl = LIS2DUXS12_BDR_XL_ODR_DIV_4;
      break;
    case LIS2DUXS12_BDR_XL_ODR_DIV_8:
      mode.batch.bdr_xl = LIS2DUXS12_BDR_XL_ODR_DIV_8;
    case LIS2DUXS12_BDR_XL_ODR_DIV_16:
      mode.batch.bdr_xl = LIS2DUXS12_BDR_XL_ODR_DIV_16;
    case LIS2DUXS12_BDR_XL_ODR_DIV_32:
      mode.batch.bdr_xl = LIS2DUXS12_BDR_XL_ODR_DIV_32;
      break;
    case LIS2DUXS12_BDR_XL_ODR_DIV_64:
      mode.batch.bdr_xl = LIS2DUXS12_BDR_XL_ODR_DIV_64;
      break;
    case LIS2DUXS12_BDR_XL_ODR_OFF:
      mode.batch.bdr_xl = LIS2DUXS12_BDR_XL_ODR_OFF;
    default:
      ret = LIS2DUXS12_STATUS_ERROR;
      break;
  }
  if (ret != LIS2DUXS12_STATUS_ERROR) {
    mode.watermark = 0x0U;
    if (lis2duxs12_fifo_mode_set(&reg_ctx, mode) != LIS2DUXS12_STATUS_OK) {
      ret = LIS2DUXS12_STATUS_ERROR;
    }
  }
  return ret;
}

/**
 * @brief  Get the LIS2DUXS12 FIFO accelero single sample (16-bit data per 3 axes) and calculate acceleration [mg]
 * @param  Acceleration FIFO accelero axes [mg]
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Get_FIFO_X_Axes(int32_t *Data)
{
  LIS2DUXS12StatusTypeDef ret = LIS2DUXS12_STATUS_OK;
  lis2duxs12_fifo_mode_t mode ;
  lis2duxs12_fifo_data_t data;
  lis2duxs12_md_t md;
  if (lis2duxs12_mode_get(&reg_ctx, &md) != 0) {
    ret = LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_fifo_mode_get(&reg_ctx, &mode) != 0) {
    ret = LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_fifo_data_get(&reg_ctx, &md, &mode, &data) != 0) {
    ret = LIS2DUXS12_STATUS_ERROR;
  }
  Data[0] = (int32_t)data.xl[0].mg[0];
  Data[1] = (int32_t)data.xl[0].mg[1];
  Data[2] = (int32_t)data.xl[0].mg[2];
  return ret;
}

/**
 * @brief Get the status of all hardware events for LIS2DUXS12 accelerometer sensor
 * @param Status the pointer to the status of all hardware events
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Get_X_Event_Status(LIS2DUXS12_Event_Status_t *Status)
{
  lis2duxs12_all_int_src_t all_int_src;
  lis2duxs12_md1_cfg_t md1_cfg;
  lis2duxs12_md2_cfg_t md2_cfg;
  lis2duxs12_embedded_status_t func_src;
  lis2duxs12_emb_pin_int_route_t emb_pin_int1;
  lis2duxs12_emb_pin_int_route_t emb_pin_int2;
  lis2duxs12_sixd_src_t sixd_src_reg;
  (void)memset((void *)Status, 0x0, sizeof(LIS2DUXS12_Event_Status_t));
  if (lis2duxs12_read_reg(&(reg_ctx), LIS2DUXS12_ALL_INT_SRC, (uint8_t *)&all_int_src, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_read_reg(&(reg_ctx), LIS2DUXS12_MD1_CFG, (uint8_t *)&md1_cfg, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_read_reg(&(reg_ctx), LIS2DUXS12_MD2_CFG, (uint8_t *)&md2_cfg, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_embedded_status_get(&(reg_ctx), &func_src) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_emb_pin_int1_route_get(&(reg_ctx), &emb_pin_int1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_emb_pin_int2_route_get(&(reg_ctx), &emb_pin_int2) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if ((md1_cfg.int1_ff == 1U) || (md2_cfg.int2_ff == 1U)) {
    if (all_int_src.ff_ia_all == 1U) {
      Status->FreeFallStatus = 1;
    }
  }
  if ((md1_cfg.int1_wu == 1U) || (md2_cfg.int2_wu == 1U)) {
    if (all_int_src.wu_ia_all == 1U) {
      Status->WakeUpStatus = 1;
    }
  }
  if ((md1_cfg.int1_tap == 1U) || (md2_cfg.int2_tap == 1U)) {
    if (all_int_src.single_tap_all == 1U) {
      Status->TapStatus = 1;
    }
    if (all_int_src.double_tap_all == 1U) {
      Status->DoubleTapStatus = 1;
    }
    if (all_int_src.triple_tap_all == 1U) {
      Status->TripleTapStatus = 1;
    }
  }
  if ((md1_cfg.int1_6d == 1U) || (md2_cfg.int2_6d == 1U)) {
    if (all_int_src.d6d_ia_all == 1U) {
      Status->D6DOrientationStatus = 1;
    }
  }
  if ((emb_pin_int1.step_det) || (emb_pin_int2.step_det)) {
    if (func_src.is_step_det == 1U) {
      Status->StepStatus = 1;
    }
  }
  if ((emb_pin_int1.tilt) || (emb_pin_int2.tilt)) {
    if (func_src.is_tilt == 1U) {
      Status->TiltStatus = 1;
    }
  }
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief Enable the wake up detection for LIS2DUXS12 accelerometer sensor
 * @note  This function sets the LIS2DUXS12 accelerometer ODR to 200Hz and the LIS2DUXS12 accelerometer full scale to 2g
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Enable_Wake_Up_Detection(LIS2DUXS12_SensorIntPin_t IntPin)
{
  LIS2DUXS12StatusTypeDef ret  = LIS2DUXS12_STATUS_OK;
  lis2duxs12_md1_cfg_t val1;
  lis2duxs12_md2_cfg_t val2;
  lis2duxs12_interrupt_cfg_t interrupt_cfg;
  lis2duxs12_ctrl1_t ctrl1;
  /* Output Data Rate selection */
  if (Set_X_ODR(200.0f) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Full scale selection */
  if (Set_X_FS(2) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Set wake-up threshold */
  if (Set_Wake_Up_Threshold(63) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Set wake-up duration */
  if (Set_Wake_Up_Duration(0) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Enable wake-up event on the 3-axis */
  if (lis2duxs12_read_reg(&(reg_ctx), LIS2DUXS12_CTRL1, (uint8_t *)&ctrl1, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  ctrl1.wu_x_en = PROPERTY_ENABLE;
  ctrl1.wu_y_en = PROPERTY_ENABLE;
  ctrl1.wu_z_en = PROPERTY_ENABLE;
  if (lis2duxs12_write_reg(&(reg_ctx), LIS2DUXS12_CTRL1, (uint8_t *)&ctrl1, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Enable wake up event on either INT1 or INT2 pin */
  switch (IntPin) {
    case LIS2DUXS12_INT1_PIN:
      if (lis2duxs12_read_reg(&(reg_ctx), LIS2DUXS12_MD1_CFG, (uint8_t *)&val1, 1) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      val1.int1_wu = PROPERTY_ENABLE;
      if (lis2duxs12_write_reg(&(reg_ctx), LIS2DUXS12_MD1_CFG, (uint8_t *)&val1, 1) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      if (lis2duxs12_read_reg(&(reg_ctx), LIS2DUXS12_INTERRUPT_CFG, (uint8_t *)&interrupt_cfg, 1) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      interrupt_cfg.interrupts_enable = PROPERTY_ENABLE;
      if (lis2duxs12_write_reg(&(reg_ctx), LIS2DUXS12_INTERRUPT_CFG, (uint8_t *)&interrupt_cfg, 1) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      break;
    case LIS2DUXS12_INT2_PIN:
      if (lis2duxs12_read_reg(&(reg_ctx), LIS2DUXS12_MD2_CFG, (uint8_t *)&val2, 1) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      val2.int2_wu = PROPERTY_ENABLE;
      if (lis2duxs12_write_reg(&(reg_ctx), LIS2DUXS12_MD2_CFG, (uint8_t *)&val2, 1) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      if (lis2duxs12_read_reg(&(reg_ctx), LIS2DUXS12_INTERRUPT_CFG, (uint8_t *)&interrupt_cfg, 1) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      interrupt_cfg.interrupts_enable = PROPERTY_ENABLE;
      if (lis2duxs12_write_reg(&(reg_ctx), LIS2DUXS12_INTERRUPT_CFG, (uint8_t *)&interrupt_cfg, 1) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      break;
    default:
      ret = LIS2DUXS12_STATUS_ERROR;
      break;
  }
  return ret;
}

/**
 * @brief Disable the wake up detection for LIS2DUXS12 accelerometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Disable_Wake_Up_Detection(void)
{
  lis2duxs12_md1_cfg_t val1;
  lis2duxs12_md2_cfg_t val2;
  lis2duxs12_ctrl1_t ctrl1;
  /* Disable wake up event on both INT1 and INT2 pins */
  if (lis2duxs12_read_reg(&(reg_ctx), LIS2DUXS12_MD1_CFG, (uint8_t *)&val1, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  val1.int1_wu = PROPERTY_DISABLE;
  if (lis2duxs12_write_reg(&(reg_ctx), LIS2DUXS12_MD1_CFG, (uint8_t *)&val1, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_read_reg(&(reg_ctx), LIS2DUXS12_MD2_CFG, (uint8_t *)&val2, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  val2.int2_wu = PROPERTY_DISABLE;
  if (lis2duxs12_write_reg(&(reg_ctx), LIS2DUXS12_MD2_CFG, (uint8_t *)&val2, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Disable wake-up event on the 3-axis */
  if (lis2duxs12_read_reg(&(reg_ctx), LIS2DUXS12_CTRL1, (uint8_t *)&ctrl1, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  ctrl1.wu_x_en = PROPERTY_DISABLE;
  ctrl1.wu_y_en = PROPERTY_DISABLE;
  ctrl1.wu_z_en = PROPERTY_DISABLE;
  if (lis2duxs12_write_reg(&(reg_ctx), LIS2DUXS12_CTRL1, (uint8_t *)&ctrl1, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Reset wake-up threshold */
  if (Set_Wake_Up_Threshold(0) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Reset wake-up duration */
  if (Set_Wake_Up_Duration(0) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief Set the wake up threshold for LIS2DUXS12 accelerometer sensor
 * @param thr the threshold to be set
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Set_Wake_Up_Threshold(uint32_t Threshold)
{
  int32_t fs;
  LIS2DUXS12StatusTypeDef ret  = LIS2DUXS12_STATUS_OK;
  float_t tmp;
  lis2duxs12_interrupt_cfg_t interrupt_cfg;
  lis2duxs12_wake_up_ths_t wup_ths;
  if (Get_X_FS(&fs) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_read_reg(&(reg_ctx), LIS2DUXS12_INTERRUPT_CFG, (uint8_t *)&interrupt_cfg, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_read_reg(&(reg_ctx), LIS2DUXS12_WAKE_UP_THS, (uint8_t *)&wup_ths, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  switch (fs) {
    case 2:
      if (Threshold < (uint32_t)(7.8125f * 63.0f)) {
        interrupt_cfg.wake_ths_w = 1;
        tmp = (float_t)Threshold / 7.8125f;
        wup_ths.wk_ths = (uint8_t)tmp;
      } else if (Threshold < (uint32_t)(31.25f * 63.0f)) {
        interrupt_cfg.wake_ths_w = 0;
        tmp = (float_t)Threshold / 31.25f;
        wup_ths.wk_ths = (uint8_t)tmp;
      } else { // Out of limit, we set it to the maximum possible Value
        interrupt_cfg.wake_ths_w = 0;
        wup_ths.wk_ths = 63;
      }
      break;
    case 4:
      if (Threshold < (uint32_t)(15.625f * 63.0f)) {
        interrupt_cfg.wake_ths_w = 1;
        tmp = (float_t)Threshold / 15.625f;
        wup_ths.wk_ths = (uint8_t)tmp;
      } else if (Threshold < (uint32_t)(62.5f * 63.0f)) {
        interrupt_cfg.wake_ths_w = 0;
        tmp = (float_t)Threshold / 62.5f;
        wup_ths.wk_ths = (uint8_t)tmp;
      } else { // Out of limit, we set it to the maximum possible Value
        interrupt_cfg.wake_ths_w = 0;
        wup_ths.wk_ths = 63;
      }
      break;
    case 8:
      if (Threshold < (uint32_t)(31.25f * 63.0f)) {
        interrupt_cfg.wake_ths_w = 1;
        tmp = (float_t)Threshold / 31.25f;
        wup_ths.wk_ths = (uint8_t)tmp;
      } else if (Threshold < (uint32_t)(125.0f * 63.0f)) {
        interrupt_cfg.wake_ths_w = 0;
        tmp = (float_t)Threshold / 125.0f;
        wup_ths.wk_ths = (uint8_t)tmp;
      } else { // Out of limit, we set it to the maximum possible Value
        interrupt_cfg.wake_ths_w = 0;
        wup_ths.wk_ths = 63;
      }
      break;
    case 16:
      if (Threshold < (uint32_t)(62.5f * 63.0f)) {
        interrupt_cfg.wake_ths_w = 1;
        tmp = (float_t)Threshold / 62.5f;
        wup_ths.wk_ths = (uint8_t)tmp;
      } else if (Threshold < (uint32_t)(250.0f * 63.0f)) {
        interrupt_cfg.wake_ths_w = 0;
        tmp = (float_t)Threshold / 250.0f;
        wup_ths.wk_ths = (uint8_t)tmp;
      } else { // Out of limit, we set it to the maximum possible Value
        interrupt_cfg.wake_ths_w = 0;
        wup_ths.wk_ths = 63;
      }
      break;
    default:
      ret = LIS2DUXS12_STATUS_ERROR;
      break;
  }
  if (ret != LIS2DUXS12_STATUS_ERROR) {
    if (lis2duxs12_write_reg(&(reg_ctx), LIS2DUXS12_INTERRUPT_CFG, (uint8_t *)&interrupt_cfg, 1) != LIS2DUXS12_STATUS_OK) {
      return LIS2DUXS12_STATUS_ERROR;
    }
    if (lis2duxs12_write_reg(&(reg_ctx), LIS2DUXS12_WAKE_UP_THS, (uint8_t *)&wup_ths, 1) != LIS2DUXS12_STATUS_OK) {
      return LIS2DUXS12_STATUS_ERROR;
    }
  }
  return ret;
}

/**
 * @brief Set the wake up duration for LIS2DUXS12 accelerometer sensor
 * @param dur the duration to be set
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Set_Wake_Up_Duration(uint8_t Duration)
{
  lis2duxs12_wake_up_dur_t wup_dur;
  lis2duxs12_wake_up_dur_ext_t wup_dur_ext;
  /* Check if the duration is one of the possible Values */
  if (Duration != 0 && Duration != 1 && Duration != 2 && Duration != 3 && Duration != 7 && Duration != 11 && Duration != 15) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_read_reg(&(reg_ctx), LIS2DUXS12_WAKE_UP_DUR, (uint8_t *)&wup_dur, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_read_reg(&(reg_ctx), LIS2DUXS12_WAKE_UP_DUR_EXT, (uint8_t *)&wup_dur_ext, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (Duration == 0 || Duration == 1 || Duration == 2) {
    wup_dur_ext.wu_dur_extended = 0;
    wup_dur.wake_dur = Duration;
  } else {
    wup_dur_ext.wu_dur_extended = 1;
    if (Duration == 3) {
      wup_dur.wake_dur = 0;
    } else if (Duration == 7) {
      wup_dur.wake_dur = 1;
    } else if (Duration == 11) {
      wup_dur.wake_dur = 2;
    } else { // Duration = 15
      wup_dur.wake_dur = 3;
    }
  }
  if (lis2duxs12_write_reg(&(reg_ctx), LIS2DUXS12_WAKE_UP_DUR, (uint8_t *)&wup_dur, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_write_reg(&(reg_ctx), LIS2DUXS12_WAKE_UP_DUR_EXT, (uint8_t *)&wup_dur_ext, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief Enable the 6D orientation detection for LIS2DUXS12 accelerometer sensor
 * @note  This function sets the LIS2DUXS12 accelerometer ODR to 200Hz and the LIS2DUXS12 accelerometer full scale to 2g
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Enable_6D_Orientation(LIS2DUXS12_SensorIntPin_t IntPin)
{
  LIS2DUXS12StatusTypeDef ret  = LIS2DUXS12_STATUS_OK;
  lis2duxs12_md1_cfg_t val1;
  lis2duxs12_md2_cfg_t val2;
  lis2duxs12_interrupt_cfg_t interrupt_cfg;
  /* Output Data Rate selection */
  if (Set_X_ODR(200.0f) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Full scale selection */
  if (Set_X_FS(2) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Threshold selection*/
  if (Set_6D_Orientation_Threshold(2) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Enable 6D orientation event on either INT1 or INT2 pin */
  switch (IntPin) {
    case LIS2DUXS12_INT1_PIN:
      if (lis2duxs12_read_reg(&(reg_ctx), LIS2DUXS12_MD1_CFG, (uint8_t *)&val1, 1) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      val1.int1_6d = PROPERTY_ENABLE;
      if (lis2duxs12_write_reg(&(reg_ctx), LIS2DUXS12_MD1_CFG, (uint8_t *)&val1, 1) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      if (lis2duxs12_read_reg(&(reg_ctx), LIS2DUXS12_INTERRUPT_CFG, (uint8_t *)&interrupt_cfg, 1) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      interrupt_cfg.interrupts_enable = PROPERTY_ENABLE;
      if (lis2duxs12_write_reg(&(reg_ctx), LIS2DUXS12_INTERRUPT_CFG, (uint8_t *)&interrupt_cfg, 1) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      break;
    case LIS2DUXS12_INT2_PIN:
      if (lis2duxs12_read_reg(&(reg_ctx), LIS2DUXS12_MD2_CFG, (uint8_t *)&val2, 1) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      val2.int2_6d = PROPERTY_ENABLE;
      if (lis2duxs12_write_reg(&(reg_ctx), LIS2DUXS12_MD2_CFG, (uint8_t *)&val2, 1) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      if (lis2duxs12_read_reg(&(reg_ctx), LIS2DUXS12_INTERRUPT_CFG, (uint8_t *)&interrupt_cfg, 1) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      interrupt_cfg.interrupts_enable = PROPERTY_ENABLE;
      if (lis2duxs12_write_reg(&(reg_ctx), LIS2DUXS12_INTERRUPT_CFG, (uint8_t *)&interrupt_cfg, 1) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      break;
    default:
      ret = LIS2DUXS12_STATUS_ERROR;
      break;
  }
  return ret;
}

/**
 * @brief Disable the 6D orientation detection for LIS2DUXS12 accelerometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Disable_6D_Orientation(void)
{
  lis2duxs12_md1_cfg_t val1;
  lis2duxs12_md2_cfg_t val2;
  /* Reset threshold */
  if (Set_6D_Orientation_Threshold(0) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Disable 6D orientation event on both INT1 and INT2 pins */
  if (lis2duxs12_read_reg(&(reg_ctx), LIS2DUXS12_MD1_CFG, (uint8_t *)&val1, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  val1.int1_6d = PROPERTY_DISABLE;
  if (lis2duxs12_write_reg(&(reg_ctx), LIS2DUXS12_MD1_CFG, (uint8_t *)&val1, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_read_reg(&(reg_ctx), LIS2DUXS12_MD2_CFG, (uint8_t *)&val2, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  val2.int2_6d = PROPERTY_DISABLE;
  if (lis2duxs12_write_reg(&(reg_ctx), LIS2DUXS12_MD2_CFG, (uint8_t *)&val2, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief Set the 6D orientation threshold for LIS2DUXS12 accelerometer sensor
 * @param thr the threshold to be set
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Set_6D_Orientation_Threshold(uint8_t Threshold)
{
  lis2duxs12_sixd_t sixd;
  if (Threshold > 3) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_read_reg(&(reg_ctx), LIS2DUXS12_SIXD, (uint8_t *)&sixd, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  sixd.d6d_ths = Threshold;
  if (lis2duxs12_write_reg(&(reg_ctx), LIS2DUXS12_SIXD, (uint8_t *)&sixd, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief Get the 6D orientation XL axis for LIS2DUXS12 accelerometer sensor
 * @param xl the pointer to the 6D orientation XL axis
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Get_6D_Orientation_XL(uint8_t *XLow)
{
  lis2duxs12_sixd_src_t data;
  if (lis2duxs12_read_reg(&(reg_ctx), LIS2DUXS12_SIXD_SRC, (uint8_t *)&data, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  *XLow = data.xl;
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief Get the 6D orientation XH axis for LIS2DUXS12 accelerometer sensor
 * @param xh the pointer to the 6D orientation XH axis
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Get_6D_Orientation_XH(uint8_t *XHigh)
{
  lis2duxs12_sixd_src_t data;
  if (lis2duxs12_read_reg(&(reg_ctx), LIS2DUXS12_SIXD_SRC, (uint8_t *)&data, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  *XHigh = data.xh;
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief Get the 6D orientation YL axis for LIS2DUXS12 accelerometer sensor
 * @param yl the pointer to the 6D orientation YL axis
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Get_6D_Orientation_YL(uint8_t *YLow)
{
  lis2duxs12_sixd_src_t data;
  if (lis2duxs12_read_reg(&(reg_ctx), LIS2DUXS12_SIXD_SRC, (uint8_t *)&data, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  *YLow = data.yl;
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief Get the 6D orientation YH axis for LIS2DUXS12 accelerometer sensor
 * @param yh the pointer to the 6D orientation YH axis
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Get_6D_Orientation_YH(uint8_t *YHigh)
{
  lis2duxs12_sixd_src_t data;
  if (lis2duxs12_read_reg(&(reg_ctx), LIS2DUXS12_SIXD_SRC, (uint8_t *)&data, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  *YHigh = data.yh;
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief Get the 6D orientation ZL axis for LIS2DUXS12 accelerometer sensor
 * @param zl the pointer to the 6D orientation ZL axis
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Get_6D_Orientation_ZL(uint8_t *ZLow)
{
  lis2duxs12_sixd_src_t data;
  if (lis2duxs12_read_reg(&(reg_ctx), LIS2DUXS12_SIXD_SRC, (uint8_t *)&data, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  *ZLow = data.zl;
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief Get the 6D orientation ZH axis for LIS2DUXS12 accelerometer sensor
 * @param zh the pointer to the 6D orientation ZH axis
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Get_6D_Orientation_ZH(uint8_t *ZHigh)
{
  lis2duxs12_sixd_src_t data;
  if (lis2duxs12_read_reg(&(reg_ctx), LIS2DUXS12_SIXD_SRC, (uint8_t *)&data, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  *ZHigh = data.zh;
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Enable free fall detection for LIS2DUXS12 accelerometer sensor
 * @param  IntPin interrupt pin line to be used
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Enable_Free_Fall_Detection(LIS2DUXS12_SensorIntPin_t IntPin)
{
  LIS2DUXS12StatusTypeDef ret = LIS2DUXS12_STATUS_OK;
  lis2duxs12_pin_int_route_t val;
  lis2duxs12_int_config_t int_conf;

  /* Output Data Rate selection */
  if (Set_X_ODR(200) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Full scale selection */
  if (Set_X_FS(2) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /*  Set free fall duration.*/
  if (Set_Free_Fall_Duration(3) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Set free fall threshold. */
  if (Set_Free_Fall_Threshold(3) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Enable free fall event on either INT1 or INT2 pin */
  switch (IntPin) {
    case LIS2DUXS12_INT1_PIN:
      if (lis2duxs12_pin_int1_route_get(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      val.free_fall = PROPERTY_ENABLE;
      if (lis2duxs12_pin_int1_route_set(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      if (lis2duxs12_int_config_get(&reg_ctx, &int_conf) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      int_conf.int_cfg = LIS2DUXS12_INT_LEVEL;
      if (lis2duxs12_int_config_set(&reg_ctx, &int_conf) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      break;
    case LIS2DUXS12_INT2_PIN:
      if (lis2duxs12_pin_int2_route_get(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      val.free_fall = PROPERTY_ENABLE;
      if (lis2duxs12_pin_int2_route_set(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      if (lis2duxs12_int_config_get(&reg_ctx, &int_conf) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      int_conf.int_cfg = LIS2DUXS12_INT_LEVEL;
      if (lis2duxs12_int_config_set(&reg_ctx, &int_conf) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      break;

    default:
      ret = LIS2DUXS12_STATUS_ERROR;
      break;
  }
  return ret;
}

/**
 * @brief  Disable free fall detection for LIS2DUXS12 accelerometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Disable_Free_Fall_Detection()
{
  lis2duxs12_md1_cfg_t val1;
  lis2duxs12_md2_cfg_t val2;
  /* Disable free fall event on both INT1 and INT2 pins */
  if (lis2duxs12_read_reg(&reg_ctx, LIS2DUXS12_MD1_CFG, (uint8_t *)&val1, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  val1.int1_ff = PROPERTY_DISABLE;
  if (lis2duxs12_write_reg(&reg_ctx, LIS2DUXS12_MD1_CFG, (uint8_t *)&val1, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_read_reg(&reg_ctx, LIS2DUXS12_MD2_CFG, (uint8_t *)&val2, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  val2.int2_ff = PROPERTY_DISABLE;
  if (lis2duxs12_write_reg(&reg_ctx, LIS2DUXS12_MD2_CFG, (uint8_t *)&val2, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Reset free fall threshold. */
  if (Set_Free_Fall_Threshold(0) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Reset free fall duration */
  if (Set_Free_Fall_Duration(0) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Set free fall threshold for LIS2DUXS12 accelerometer sensor
 * @param  Threshold free fall detection threshold to be used
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Set_Free_Fall_Threshold(uint8_t Threshold)
{
  lis2duxs12_ff_thresholds_t val;
  switch (Threshold) {
    case LIS2DUXS12_156_mg:
      val = LIS2DUXS12_156_mg;
      break;
    case LIS2DUXS12_219_mg:
      val = LIS2DUXS12_219_mg;
      break;
    case LIS2DUXS12_250_mg:
      val = LIS2DUXS12_250_mg;
      break;
    case LIS2DUXS12_312_mg:
      val = LIS2DUXS12_312_mg;
      break;
    case LIS2DUXS12_344_mg:
      val = LIS2DUXS12_344_mg;
      break;
    case LIS2DUXS12_406_mg:
      val = LIS2DUXS12_406_mg;
      break;
    case LIS2DUXS12_469_mg:
      val = LIS2DUXS12_469_mg;
      break;
    case LIS2DUXS12_500_mg:
      val = LIS2DUXS12_500_mg;
      break;
    default:
      val = LIS2DUXS12_156_mg;
      break;
  }
  /* Set free fall threshold. */
  if (lis2duxs12_ff_thresholds_set(&reg_ctx, val) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Set free fall duration for LIS2DUXS12 accelerometer sensor
 * @param  Duration free fall detection duration to be used
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Set_Free_Fall_Duration(uint8_t Duration)
{
  if (lis2duxs12_ff_duration_set(&reg_ctx, Duration) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Enable single tap detection for LIS2DUXS12 accelerometer sensor
 * @param  IntPin interrupt pin line to be used
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Enable_Single_Tap_Detection(LIS2DUXS12_SensorIntPin_t IntPin)
{
  LIS2DUXS12StatusTypeDef ret = LIS2DUXS12_STATUS_OK;
  lis2duxs12_pin_int_route_t val;
  lis2duxs12_int_config_t int_conf;
  lis2duxs12_tap_config_t tap_cfg;
  /* Output Data Rate selection */
  if (Set_X_ODR(400) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Full scale selection */
  if (Set_X_FS(8) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_tap_config_get(&reg_ctx, &tap_cfg) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  tap_cfg.axis = LIS2DUXS12_TAP_ON_Z;
  tap_cfg.inverted_peak_time = 4;
  tap_cfg.pre_still_ths = 2;
  tap_cfg.post_still_time = 8;
  tap_cfg.shock_wait_time = 6;
  tap_cfg.post_still_ths = 8;
  tap_cfg.latency = 4;
  tap_cfg.wait_end_latency = 1;
  tap_cfg.peak_ths = 8;
  tap_cfg.single_tap_on = PROPERTY_ENABLE;
  tap_cfg.pre_still_n = 10;
  /* Set tap mode. */
  if (lis2duxs12_tap_config_set(&reg_ctx, tap_cfg) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Enable single tap event on either INT1 or INT2 pin */
  switch (IntPin) {
    case LIS2DUXS12_INT1_PIN:
      if (lis2duxs12_pin_int1_route_get(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      val.tap = PROPERTY_ENABLE;
      if (lis2duxs12_pin_int1_route_set(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      if (lis2duxs12_int_config_get(&reg_ctx, &int_conf) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      int_conf.int_cfg = LIS2DUXS12_INT_LEVEL;
      if (lis2duxs12_int_config_set(&reg_ctx, &int_conf) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      break;
    case LIS2DUXS12_INT2_PIN:
      if (lis2duxs12_pin_int2_route_get(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      val.tap = PROPERTY_ENABLE;
      if (lis2duxs12_pin_int2_route_set(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      if (lis2duxs12_int_config_get(&reg_ctx, &int_conf) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      int_conf.int_cfg = LIS2DUXS12_INT_LEVEL;
      if (lis2duxs12_int_config_set(&reg_ctx, &int_conf) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      break;
  }
  return ret;
}

/**
 * @brief  Disable single tap detection for LIS2DUXS12 accelerometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Disable_Single_Tap_Detection()
{
  LIS2DUXS12StatusTypeDef ret = LIS2DUXS12_STATUS_OK;
  lis2duxs12_pin_int_route_t val;
  lis2duxs12_tap_config_t tap_cfg;
  if (lis2duxs12_tap_config_get(&reg_ctx, &tap_cfg) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  tap_cfg.single_tap_on = PROPERTY_DISABLE;
  /* Set tap mode. */
  if (lis2duxs12_tap_config_set(&reg_ctx, tap_cfg) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Disable single tap event on both INT1 and INT2 pin */
  if (lis2duxs12_pin_int1_route_get(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  val.tap = PROPERTY_DISABLE;
  if (lis2duxs12_pin_int1_route_set(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_pin_int2_route_get(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  val.tap = PROPERTY_DISABLE;
  if (lis2duxs12_pin_int2_route_set(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  return ret;
}

/**
 * @brief  Enable double tap detection for LIS2DUXS12 accelerometer sensor
 * @param  IntPin interrupt pin line to be used
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Enable_Double_Tap_Detection(LIS2DUXS12_SensorIntPin_t IntPin)
{
  LIS2DUXS12StatusTypeDef ret = LIS2DUXS12_STATUS_OK;
  lis2duxs12_pin_int_route_t val;
  lis2duxs12_int_config_t int_conf;
  lis2duxs12_tap_config_t tap_cfg;
  /* Output Data Rate selection */
  if (Set_X_ODR(400) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Full scale selection */
  if (Set_X_FS(8) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_tap_config_get(&reg_ctx, &tap_cfg) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Enable Z direction in tap recognition. */
  tap_cfg.axis = LIS2DUXS12_TAP_ON_Z;
  tap_cfg.inverted_peak_time = 4;
  tap_cfg.pre_still_ths = 2;
  tap_cfg.post_still_time = 8;
  tap_cfg.shock_wait_time = 6;
  tap_cfg.post_still_ths = 8;
  tap_cfg.latency = 4;
  tap_cfg.wait_end_latency = 1;
  tap_cfg.peak_ths = 8;
  /* Enable double tap event */
  tap_cfg.double_tap_on = PROPERTY_ENABLE;
  tap_cfg.pre_still_n = 10;
  if (lis2duxs12_tap_config_set(&reg_ctx, tap_cfg) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Enable double tap event on either INT1 or INT2 pin */
  switch (IntPin) {
    case LIS2DUXS12_INT1_PIN:
      if (lis2duxs12_pin_int1_route_get(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      val.tap = PROPERTY_ENABLE;
      if (lis2duxs12_pin_int1_route_set(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      if (lis2duxs12_int_config_get(&reg_ctx, &int_conf) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      int_conf.int_cfg = LIS2DUXS12_INT_LEVEL;
      if (lis2duxs12_int_config_set(&reg_ctx, &int_conf) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      break;
    case LIS2DUXS12_INT2_PIN:
      if (lis2duxs12_pin_int2_route_get(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      val.tap = PROPERTY_ENABLE;
      if (lis2duxs12_pin_int2_route_set(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      if (lis2duxs12_int_config_get(&reg_ctx, &int_conf) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      int_conf.int_cfg = LIS2DUXS12_INT_LEVEL;
      if (lis2duxs12_int_config_set(&reg_ctx, &int_conf) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      break;
  }
  return ret;
}

/**
 * @brief  Disable double tap detection for LIS2DUXS12 accelerometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Disable_Double_Tap_Detection()
{
  LIS2DUXS12StatusTypeDef ret = LIS2DUXS12_STATUS_OK;
  lis2duxs12_pin_int_route_t val;
  lis2duxs12_tap_config_t tap_cfg;
  if (lis2duxs12_tap_config_get(&reg_ctx, &tap_cfg) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  tap_cfg.double_tap_on = PROPERTY_DISABLE;
  /* Set tap mode. */
  if (lis2duxs12_tap_config_set(&reg_ctx, tap_cfg) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Disable double tap event on both INT1 and INT2 pin */
  if (lis2duxs12_pin_int1_route_get(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  val.tap = PROPERTY_DISABLE;
  if (lis2duxs12_pin_int1_route_set(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_pin_int2_route_get(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  val.tap = PROPERTY_DISABLE;
  if (lis2duxs12_pin_int2_route_set(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  return ret;
}

/**
 * @brief  Enable triple tap detection for LIS2DUXS12 accelerometer sensor
 * @param  IntPin interrupt pin line to be used
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Enable_Triple_Tap_Detection(LIS2DUXS12_SensorIntPin_t IntPin)
{
  LIS2DUXS12StatusTypeDef ret = LIS2DUXS12_STATUS_OK;
  lis2duxs12_pin_int_route_t val;
  lis2duxs12_int_config_t int_conf;
  lis2duxs12_tap_config_t tap_cfg;
  /* Output Data Rate selection */
  if (Set_X_ODR(400) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Full scale selection */
  if (Set_X_FS(8) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_tap_config_get(&reg_ctx, &tap_cfg) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  tap_cfg.axis = LIS2DUXS12_TAP_ON_Z;
  tap_cfg.inverted_peak_time = 4;
  tap_cfg.pre_still_ths = 2;
  tap_cfg.post_still_time = 8;
  tap_cfg.shock_wait_time = 6;
  tap_cfg.post_still_ths = 8;
  tap_cfg.latency = 4;
  tap_cfg.wait_end_latency = 1;
  tap_cfg.peak_ths = 8;
  tap_cfg.triple_tap_on = PROPERTY_ENABLE;
  tap_cfg.pre_still_n = 10;
  /* Set tap mode. */
  if (lis2duxs12_tap_config_set(&reg_ctx, tap_cfg) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Enable triple tap event on either INT1 or INT2 pin */
  switch (IntPin) {
    case LIS2DUXS12_INT1_PIN:
      if (lis2duxs12_pin_int1_route_get(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      val.tap = PROPERTY_ENABLE;
      if (lis2duxs12_pin_int1_route_set(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      if (lis2duxs12_int_config_get(&reg_ctx, &int_conf) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      int_conf.int_cfg = LIS2DUXS12_INT_LEVEL;
      if (lis2duxs12_int_config_set(&reg_ctx, &int_conf) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      break;
    case LIS2DUXS12_INT2_PIN:
      if (lis2duxs12_pin_int2_route_get(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      val.tap = PROPERTY_ENABLE;
      if (lis2duxs12_pin_int2_route_set(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      if (lis2duxs12_int_config_get(&reg_ctx, &int_conf) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      int_conf.int_cfg = LIS2DUXS12_INT_LEVEL;
      if (lis2duxs12_int_config_set(&reg_ctx, &int_conf) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      break;
  }
  return ret;
}

/**
 * @brief  Disable triple tap detection for LIS2DUXS12 accelerometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Disable_Triple_Tap_Detection()
{
  lis2duxs12_pin_int_route_t val;
  lis2duxs12_tap_config_t tap_cfg;
  if (lis2duxs12_tap_config_get(&reg_ctx, &tap_cfg) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  tap_cfg.triple_tap_on = PROPERTY_DISABLE;
  /* Set tap mode. */
  if (lis2duxs12_tap_config_set(&reg_ctx, tap_cfg) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Disable triple tap event on both INT1 and INT2 pin */
  if (lis2duxs12_pin_int1_route_get(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  val.tap = PROPERTY_DISABLE;
  if (lis2duxs12_pin_int1_route_set(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_pin_int2_route_get(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  val.tap = PROPERTY_DISABLE;
  if (lis2duxs12_pin_int2_route_set(&reg_ctx, &val) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Get Tap configuration for LIS2DUXS12 accelerometer sensor
 * @param  TapConf tap configuration pointer where data is written
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Get_Tap_Configuration(LIS2DUXS12_Tap_Config_t *cfg)
{
  lis2duxs12_tap_config_t tap_config;
  if (lis2duxs12_tap_config_get(&reg_ctx, &tap_config)) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  cfg->axis = tap_config.axis;
  cfg->inverted_peak_time = tap_config.inverted_peak_time;
  cfg->pre_still_ths = tap_config.pre_still_ths;
  cfg->post_still_ths = tap_config.post_still_ths;
  cfg->post_still_time = tap_config.post_still_time;
  cfg->shock_wait_time = tap_config.shock_wait_time;
  cfg->latency = tap_config.latency;
  cfg->wait_end_latency = tap_config.wait_end_latency;
  cfg->peak_ths = tap_config.peak_ths;
  cfg->rebound = tap_config.rebound;
  cfg->pre_still_start = tap_config.pre_still_start;
  cfg->pre_still_n = tap_config.pre_still_n;
  cfg->single_tap_on = tap_config.single_tap_on;
  cfg->double_tap_on = tap_config.double_tap_on;
  cfg->triple_tap_on = tap_config.triple_tap_on;
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Get Tap configuration for LIS2DUXS12 accelerometer sensor
 * @param  TapConf tap configuration
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Set_Tap_Configuration(LIS2DUXS12_Tap_Config_t cfg)
{
  lis2duxs12_tap_config_t tap_config;
  if (lis2duxs12_tap_config_get(&reg_ctx, &tap_config)) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  tap_config.axis = (lis2duxs12_axis_t)cfg.axis;
  tap_config.inverted_peak_time = cfg.inverted_peak_time;
  tap_config.pre_still_ths = cfg.pre_still_ths;
  tap_config.post_still_ths = cfg.post_still_ths;
  tap_config.post_still_time = cfg.post_still_time;
  tap_config.shock_wait_time = cfg.shock_wait_time;
  tap_config.latency = cfg.latency;
  tap_config.wait_end_latency = cfg.wait_end_latency;
  tap_config.peak_ths = cfg.peak_ths;
  tap_config.rebound = cfg.rebound;
  tap_config.pre_still_start = cfg.pre_still_start;
  tap_config.pre_still_n = cfg.pre_still_n;
  tap_config.single_tap_on = cfg.single_tap_on;
  tap_config.double_tap_on = cfg.double_tap_on;
  tap_config.triple_tap_on = cfg.triple_tap_on;
  if (lis2duxs12_tap_config_set(&reg_ctx, tap_config)) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  return LIS2DUXS12_STATUS_OK;
}
/**
 * @brief  Enable tilt detection for LIS2DUXS12 accelerometer sensor
 * @param  IntPin interrupt pin line to be used
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Enable_Tilt_Detection(LIS2DUXS12_SensorIntPin_t IntPin)
{
  LIS2DUXS12StatusTypeDef ret = LIS2DUXS12_STATUS_OK;
  lis2duxs12_emb_pin_int_route_t emb_pin_int;
  /* Output Data Rate selection */
  if (Set_X_ODR(25) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Full scale selection */
  if (Set_X_FS(2) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_init_set(&(reg_ctx), LIS2DUXS12_SENSOR_EMB_FUNC_ON) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_tilt_mode_set(&reg_ctx, PROPERTY_ENABLE) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  switch (IntPin) {
    case LIS2DUXS12_INT1_PIN:
      if (lis2duxs12_emb_pin_int1_route_get(&reg_ctx, &emb_pin_int) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      emb_pin_int.tilt = PROPERTY_ENABLE;
      if (lis2duxs12_emb_pin_int1_route_set(&reg_ctx, &emb_pin_int) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      break;
    case LIS2DUXS12_INT2_PIN:
      if (lis2duxs12_emb_pin_int2_route_get(&reg_ctx, &emb_pin_int) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      emb_pin_int.tilt = PROPERTY_ENABLE;
      if (lis2duxs12_emb_pin_int2_route_set(&reg_ctx, &emb_pin_int) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      break;
    default:
      ret = LIS2DUXS12_STATUS_ERROR;
      break;
  }
  return ret;
}

/**
 * @brief  Disable tilt detection for LIS2DUXS12 accelerometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Disable_Tilt_Detection()
{
  LIS2DUXS12StatusTypeDef ret = LIS2DUXS12_STATUS_OK;
  lis2duxs12_emb_pin_int_route_t emb_pin_int;
  if (lis2duxs12_tilt_mode_set(&reg_ctx, PROPERTY_DISABLE) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_emb_pin_int1_route_get(&reg_ctx, &emb_pin_int) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  emb_pin_int.tilt = PROPERTY_DISABLE;
  if (lis2duxs12_emb_pin_int1_route_set(&reg_ctx, &emb_pin_int) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_emb_pin_int2_route_get(&reg_ctx, &emb_pin_int) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  emb_pin_int.tilt = PROPERTY_DISABLE;
  if (lis2duxs12_emb_pin_int2_route_set(&reg_ctx, &emb_pin_int) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  return ret;
}

/**
 * @brief  Enable pedometer
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Enable_Pedometer(LIS2DUXS12_SensorIntPin_t IntPin)
{
  LIS2DUXS12StatusTypeDef ret = LIS2DUXS12_STATUS_OK;
  lis2duxs12_stpcnt_mode_t mode;
  lis2duxs12_emb_pin_int_route_t emb_pin_int;
  /* Output Data Rate selection */
  if (Set_X_ODR(25) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Full scale selection */
  if (Set_X_FS(4) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_init_set(&(reg_ctx), LIS2DUXS12_SENSOR_EMB_FUNC_ON) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_stpcnt_mode_get(&reg_ctx, &mode) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Enable pedometer algorithm. */
  mode.step_counter_enable = PROPERTY_ENABLE;
  mode.false_step_rej = PROPERTY_DISABLE;
  mode.step_counter_in_fifo = PROPERTY_DISABLE;
  /* Turn on embedded features */
  if (lis2duxs12_stpcnt_mode_set(&reg_ctx, mode) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  switch (IntPin) {
    case LIS2DUXS12_INT1_PIN:
      if (lis2duxs12_emb_pin_int1_route_get(&reg_ctx, &emb_pin_int) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      emb_pin_int.step_det = PROPERTY_ENABLE;
      if (lis2duxs12_emb_pin_int1_route_set(&reg_ctx, &emb_pin_int) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      break;
    case LIS2DUXS12_INT2_PIN:
      if (lis2duxs12_emb_pin_int2_route_get(&reg_ctx, &emb_pin_int) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      emb_pin_int.step_det = PROPERTY_ENABLE;
      if (lis2duxs12_emb_pin_int2_route_set(&reg_ctx, &emb_pin_int) != LIS2DUXS12_STATUS_OK) {
        return LIS2DUXS12_STATUS_ERROR;
      }
      break;
    default:
      ret = LIS2DUXS12_STATUS_ERROR;
      break;
  }
  return ret;
}


/**
 * @brief  Disable pedometer
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Disable_Pedometer()
{
  lis2duxs12_emb_pin_int_route_t emb_pin_int;
  lis2duxs12_stpcnt_mode_t mode;
  if (lis2duxs12_stpcnt_mode_get(&reg_ctx, &mode) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  /* Enable pedometer algorithm. */
  mode.step_counter_enable = PROPERTY_DISABLE;
  mode.false_step_rej = PROPERTY_DISABLE;
  mode.step_counter_in_fifo = PROPERTY_DISABLE;

  /* Turn off embedded features */
  if (lis2duxs12_stpcnt_mode_set(&reg_ctx, mode) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }

  if (lis2duxs12_emb_pin_int1_route_get(&reg_ctx, &emb_pin_int) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  emb_pin_int.tilt = PROPERTY_DISABLE;
  if (lis2duxs12_emb_pin_int1_route_set(&reg_ctx, &emb_pin_int) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_emb_pin_int2_route_get(&reg_ctx, &emb_pin_int) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  emb_pin_int.tilt = PROPERTY_DISABLE;
  if (lis2duxs12_emb_pin_int2_route_set(&reg_ctx, &emb_pin_int) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Get step count
 * @param  StepCount step counter
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Get_Step_Count(uint16_t *StepCount)
{
  if (lis2duxs12_stpcnt_steps_get(&reg_ctx, StepCount) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Enable step counter reset
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Step_Counter_Reset()
{
  if (lis2duxs12_stpcnt_rst_step_set(&reg_ctx) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Enable the LIS2DUXS12 QVAR feature
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Enable_QVAR()
{
  lis2duxs12_ah_qvar_mode_t qvar_mode;
  qvar_mode.ah_qvar_en = 1;
  qvar_mode.ah_qvar_zin = LIS2DUXS12_520MOhm;
  qvar_mode.ah_qvar_gain = LIS2DUXS12_GAIN_0_5;
  if (lis2duxs12_ah_qvar_mode_set(&reg_ctx, qvar_mode) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Disable the LIS2DUXS12 QVAR feature
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Disable_QVAR()
{
  lis2duxs12_ah_qvar_mode_t qvar_mode;
  qvar_mode.ah_qvar_en = 0;
  if (lis2duxs12_ah_qvar_mode_set(&reg_ctx, qvar_mode) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Read LIS2DUXS12 QVAR output data
 * @param  Data pointer where the value is written
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Get_QVAR_Data(float_t *Data)
{
  lis2duxs12_ah_qvar_data_t qvar_data;
  lis2duxs12_md_t mode;
  lis2duxs12_xl_data_t data;
  if (lis2duxs12_mode_get(&reg_ctx, &mode) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_xl_data_get(&(reg_ctx), &mode, &data) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  if (lis2duxs12_ah_qvar_data_get(&reg_ctx, &mode, &qvar_data)) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  *Data = qvar_data.mv;
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Get LIS2DUXS12 QVAR equivalent input impedance
 * @param  val pointer where the value is written
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Get_QVAR_Impedance(uint8_t *val)
{
  LIS2DUXS12StatusTypeDef ret = LIS2DUXS12_STATUS_OK;
  lis2duxs12_ah_qvar_mode_t qvar_mode;
  if (lis2duxs12_ah_qvar_mode_get(&reg_ctx, &qvar_mode) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  switch (qvar_mode.ah_qvar_zin) {
    case LIS2DUXS12_520MOhm:
      *val = LIS2DUXS12_520MOhm;
      break;
    case LIS2DUXS12_175MOhm:
      *val = LIS2DUXS12_175MOhm;
      break;
    case LIS2DUXS12_310MOhm:
      *val = LIS2DUXS12_310MOhm;
      break;
    case LIS2DUXS12_75MOhm:
      *val = LIS2DUXS12_75MOhm;
      break;
    default:
      ret = LIS2DUXS12_STATUS_ERROR;
      break;
  }
  return ret;
}

/**
 * @brief  Set LIS2DUXS12 QVAR equivalent input impedance
 * @param  val impedance in MOhm (520MOhm, 310MOhm, 175MOhm, 75MOhm)
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Set_QVAR_Impedance(uint8_t val)
{
  LIS2DUXS12StatusTypeDef ret = LIS2DUXS12_STATUS_OK;
  lis2duxs12_ah_qvar_mode_t qvar_mode;
  if (lis2duxs12_ah_qvar_mode_get(&reg_ctx, &qvar_mode) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  switch (val) {
    case LIS2DUXS12_520MOhm:
      qvar_mode.ah_qvar_zin = LIS2DUXS12_520MOhm;
      break;
    case LIS2DUXS12_175MOhm:
      qvar_mode.ah_qvar_zin = LIS2DUXS12_175MOhm;
      break;
    case LIS2DUXS12_310MOhm:
      qvar_mode.ah_qvar_zin = LIS2DUXS12_310MOhm;
      break;
    case LIS2DUXS12_75MOhm:
      qvar_mode.ah_qvar_zin = LIS2DUXS12_75MOhm;
      break;
    default:
      ret = LIS2DUXS12_STATUS_ERROR;
      break;
  }
  if (ret != LIS2DUXS12_STATUS_ERROR) {
    if (lis2duxs12_ah_qvar_mode_set(&reg_ctx, qvar_mode) != LIS2DUXS12_STATUS_OK) {
      ret = LIS2DUXS12_STATUS_ERROR;
    }
  }
  return ret;
}

/**
 * @brief  Get LIS2DUXS12 QVAR input-output gain
 * @param  val pointer where the value is written
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Get_QVAR_Gain(uint8_t *val)
{
  LIS2DUXS12StatusTypeDef ret = LIS2DUXS12_STATUS_OK;
  lis2duxs12_ah_qvar_mode_t qvar_mode;
  if (lis2duxs12_ah_qvar_mode_get(&reg_ctx, &qvar_mode) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  switch (qvar_mode.ah_qvar_gain) {
    case LIS2DUXS12_GAIN_0_5:
      *val = LIS2DUXS12_GAIN_0_5;
      break;
    case LIS2DUXS12_GAIN_1:
      *val = LIS2DUXS12_GAIN_1;
      break;
    case LIS2DUXS12_GAIN_2:
      *val = LIS2DUXS12_GAIN_2;
      break;
    case LIS2DUXS12_GAIN_4:
      *val = LIS2DUXS12_GAIN_4;
      break;
    default:
      ret = LIS2DUXS12_STATUS_ERROR;
      break;
  }
  return ret;
}

/**
 * @brief  Set LIS2DUXS12 QVAR equivalent input-output gain
 * @param  val  input-output gain (0,1,2,4)
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Set_QVAR_Gain(uint8_t val)
{
  LIS2DUXS12StatusTypeDef ret = LIS2DUXS12_STATUS_OK;
  lis2duxs12_ah_qvar_mode_t qvar_mode;
  if (lis2duxs12_ah_qvar_mode_get(&reg_ctx, &qvar_mode) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  switch (val) {
    case LIS2DUXS12_GAIN_0_5:
      qvar_mode.ah_qvar_gain = LIS2DUXS12_GAIN_0_5;
      break;
    case LIS2DUXS12_GAIN_1:
      qvar_mode.ah_qvar_gain = LIS2DUXS12_GAIN_1;
      break;
    case LIS2DUXS12_GAIN_2:
      qvar_mode.ah_qvar_gain = LIS2DUXS12_GAIN_2;
      break;
    case LIS2DUXS12_GAIN_4:
      qvar_mode.ah_qvar_gain = LIS2DUXS12_GAIN_4;
      break;
    default:
      ret = LIS2DUXS12_STATUS_ERROR;
      break;
  }
  if (ret != LIS2DUXS12_STATUS_ERROR) {
    if (lis2duxs12_ah_qvar_mode_set(&reg_ctx, qvar_mode) != LIS2DUXS12_STATUS_OK) {
      ret = LIS2DUXS12_STATUS_ERROR;
    }
  }
  return ret;
}

/**
 * @brief  Read LIS2DUXS12 QVAR status
 * @param  val pointer where the value is written
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Get_QVAR_Status(uint8_t *val)
{

  lis2duxs12_status_register_t status;
  if (lis2duxs12_read_reg(&reg_ctx, LIS2DUXS12_STATUS, (uint8_t *)&status, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  *val = status.drdy;
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Get MLC status for LIS2DUXS12 accelerometer sensor
 * @param  status pointer where the MLC status is written
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Get_MLC_Status(lis2duxs12_mlc_status_mainpage_t *status)
{
  if (lis2duxs12_mlc_status_get(&reg_ctx, status) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Get MLC output for LIS2DUXS12 accelerometer sensor
 * @param  output pointer where the MLC output is written
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Get_MLC_Output(uint8_t *output)
{
  if (lis2duxs12_mlc_out_get(&reg_ctx, output) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Get the LIS2DUXS12 register value
 * @param  Reg address to be read
 * @param  Data pointer where the value is written
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Read_Reg(uint8_t Reg, uint8_t *Data)
{
  if (lis2duxs12_read_reg(&reg_ctx, Reg, Data, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  return LIS2DUXS12_STATUS_OK;
}

/**
 * @brief  Set the LIS2DUXS12 register value
 * @param  Reg address to be written
 * @param  Data value to be written
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DUXS12StatusTypeDef LIS2DUXS12Sensor::Write_Reg(uint8_t Reg, uint8_t Data)
{
  if (lis2duxs12_write_reg(&reg_ctx, Reg, &Data, 1) != LIS2DUXS12_STATUS_OK) {
    return LIS2DUXS12_STATUS_ERROR;
  }
  return LIS2DUXS12_STATUS_OK;
}
int32_t LIS2DUXS12_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite)
{
  return ((LIS2DUXS12Sensor *)handle)->IO_Write(pBuffer, WriteAddr, nBytesToWrite);
}
int32_t LIS2DUXS12_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead)
{
  return ((LIS2DUXS12Sensor *)handle)->IO_Read(pBuffer, ReadAddr, nBytesToRead);
}

