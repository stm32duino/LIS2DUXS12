/**
 ******************************************************************************
 * @file    LIS2DUXS12Sensor.h
 * @author  CLab
 * @version V1.0.0
 * @date    15 November 2018
 * @brief   Abstract Class of an LIS2DUXS12 Inertial Measurement Unit (IMU) 3 axes
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


/* Prevent recursive inclusion -----------------------------------------------*/

#ifndef __LIS2DUXS12Sensor_H__
#define __LIS2DUXS12Sensor_H__


/* Includes ------------------------------------------------------------------*/

#include "Wire.h"
#include "SPI.h"
#include "lis2duxs12_reg.h"

/* Defines -------------------------------------------------------------------*/

#define LIS2DUXS12_I2C_BUS                 0U
#define LIS2DUXS12_SPI_4WIRES_BUS          1U
#define LIS2DUXS12_SPI_3WIRES_BUS          2U
#define LIS2DUXS12_I3C_BUS                 3U

#define LIS2DUXS12_X_SENSITIVITY_FOR_FS_2G   0.061f  /**< Sensitivity value for 2g full scale, Low-power1 mode [mg/LSB] */
#define LIS2DUXS12_X_SENSITIVITY_FOR_FS_4G   0.122f  /**< Sensitivity value for 4g full scale, Low-power1 mode [mg/LSB] */
#define LIS2DUXS12_X_SENSITIVITY_FOR_FS_8G   0.244f  /**< Sensitivity value for 8g full scale, Low-power1 mode [mg/LSB] */
#define LIS2DUXS12_X_SENSITIVITY_FOR_FS_16G  0.488f  /**< Sensitivity value for 16g full scale, Low-power1 mode [mg/LSB] */

#define LIS2DUXS12_QVAR_GAIN  74.4f  /**< Gain value for bits AH_QVAR_GAIN_[1:0] == 1 in AH_QVAR_CFG register, 16 bits [LSB/mV] */

#define LIS2DUXS12_QVAR_GAIN_MULTIPL_0_5X  0.5f  /**< Gain multiplier for bits AH_QVAR_GAIN_[1:0] == 0 in AH_QVAR_CFG register [-] */
#define LIS2DUXS12_QVAR_GAIN_MULTIPL_1X    1.0f  /**< Gain multiplier for bits AH_QVAR_GAIN_[1:0] == 1 in AH_QVAR_CFG register [-] */
#define LIS2DUXS12_QVAR_GAIN_MULTIPL_2X    2.0f  /**< Gain multiplier for bits AH_QVAR_GAIN_[1:0] == 2 in AH_QVAR_CFG register [-] */
#define LIS2DUXS12_QVAR_GAIN_MULTIPL_4X    4.0f  /**< Gain multiplier for bits AH_QVAR_GAIN_[1:0] == 3 in AH_QVAR_CFG register [-] */

/* Typedefs ------------------------------------------------------------------*/
typedef enum {
  LIS2DUXS12_STATUS_OK = 0,
  LIS2DUXS12_STATUS_ERROR
} LIS2DUXS12StatusTypeDef;

typedef enum {
  LIS2DUXS12_INT1_PIN,
  LIS2DUXS12_INT2_PIN,
} LIS2DUXS12_SensorIntPin_t;

typedef struct {
  unsigned int FreeFallStatus : 1;
  unsigned int TapStatus : 1;
  unsigned int DoubleTapStatus : 1;
  unsigned int TripleTapStatus : 1;
  unsigned int WakeUpStatus : 1;
  unsigned int StepStatus : 1;
  unsigned int TiltStatus : 1;
  unsigned int D6DOrientationStatus : 1;
  unsigned int SleepStatus : 1;
} LIS2DUXS12_Event_Status_t;

typedef struct {
  uint8_t axis                         : 3; /* axes on which enable tap */
  uint8_t inverted_peak_time           : 5; /* 1 LSB == 1 sample */
  uint8_t pre_still_ths                : 4; /* 1 LSB == 62.5 mg */
  uint8_t post_still_ths               : 4; /* 1 LSB == 62.5 mg */
  uint8_t post_still_time              : 6; /* samples num during stationary condition */
  uint8_t shock_wait_time              : 6; /* samples num during shock condition */
  uint8_t latency                      : 4; /* samples max num between taps */
  uint8_t wait_end_latency             : 1; /* wait end of latency time to generate tap events */
  uint8_t peak_ths                     : 6; /* 1 LSB == 62.5 mg */
  uint8_t rebound                      : 5; /* samples num during rebound condition */
  uint8_t pre_still_start              : 4; /* pre still start */
  uint8_t pre_still_n                  : 4; /* pre still n */
  uint8_t single_tap_on                : 1; /* enable single tap */
  uint8_t double_tap_on                : 1; /* enable double tap */
  uint8_t triple_tap_on                : 1; /* enable triple tap */
} LIS2DUXS12_Tap_Config_t;

typedef enum {
  LIS2DUXS12_ULTRA_LOW_POWER,
  LIS2DUXS12_LOW_POWER,
  LIS2DUXS12_HIGH_PERFORMANCE,
} LIS2DUXS12_Power_Mode_t;

typedef union {
  int16_t i16bit[3];
  uint8_t u8bit[6];
} lis2duxs12_axis3bit16_t;

typedef union {
  int16_t i16bit;
  uint8_t u8bit[2];
} lis2duxs12_axis1bit16_t;

typedef union {
  int32_t i32bit[3];
  uint8_t u8bit[12];
} lis2duxs12_axis3bit32_t;

typedef union {
  int32_t i32bit;
  uint8_t u8bit[4];
} lis2duxs12_axis1bit32_t;

/* Class Declaration ---------------------------------------------------------*/

/**
 * Abstract class of an LIS2DUXS12 Inertial Measurement Unit (IMU) 3 axes
 * sensor.
 */
class LIS2DUXS12Sensor {
  public:
    LIS2DUXS12Sensor(TwoWire *i2c, uint8_t address = LIS2DUXS12_I2C_ADD_H);
    LIS2DUXS12Sensor(SPIClass *spi, int cs_pin, uint32_t spi_speed = 2000000);

    LIS2DUXS12StatusTypeDef begin();
    LIS2DUXS12StatusTypeDef end();

    LIS2DUXS12StatusTypeDef Enable_X(void);
    LIS2DUXS12StatusTypeDef Disable_X(void);

    LIS2DUXS12StatusTypeDef ReadID(uint8_t *id);
    LIS2DUXS12StatusTypeDef Get_X_Axes(int32_t *Acceleration);
    LIS2DUXS12StatusTypeDef Get_X_Sensitivity(float *sensitivity);
    LIS2DUXS12StatusTypeDef Get_X_AxesRaw(int32_t *Value);
    LIS2DUXS12StatusTypeDef Get_X_ODR(float *odr);
    LIS2DUXS12StatusTypeDef Set_X_ODR(float odr);
    LIS2DUXS12StatusTypeDef Set_X_ODR_With_Mode(float odr, LIS2DUXS12_Power_Mode_t Power);
    LIS2DUXS12StatusTypeDef Get_X_FS(int32_t *full_scale);
    LIS2DUXS12StatusTypeDef Set_X_FS(int32_t full_scale);

    LIS2DUXS12StatusTypeDef LIS2DUXS12_X_Get_DRDY_Status(uint8_t *val);

    LIS2DUXS12StatusTypeDef Get_FIFO_Num_Samples(uint16_t *NumSamples);
    LIS2DUXS12StatusTypeDef Get_FIFO_Full_Status(uint8_t *Status);
    LIS2DUXS12StatusTypeDef Get_FIFO_Overrun_Status(uint8_t *Status);
    LIS2DUXS12StatusTypeDef Get_FIFO_Watermark_Status(uint8_t *Status);
    LIS2DUXS12StatusTypeDef Set_FIFO_INT1_FIFO_Full(uint8_t Status);
    LIS2DUXS12StatusTypeDef Set_FIFO_INT1_FIFO_Overrun(uint8_t Status);
    LIS2DUXS12StatusTypeDef Set_FIFO_INT1_FIFO_Threshold(uint8_t Status);
    LIS2DUXS12StatusTypeDef Set_FIFO_INT2_FIFO_Full(uint8_t Status);
    LIS2DUXS12StatusTypeDef Set_FIFO_INT2_FIFO_Overrun(uint8_t Status);
    LIS2DUXS12StatusTypeDef Set_FIFO_INT2_FIFO_Threshold(uint8_t Status);
    LIS2DUXS12StatusTypeDef Set_FIFO_Watermark_Level(uint8_t Watermark);
    LIS2DUXS12StatusTypeDef Set_FIFO_Stop_On_Fth(uint8_t Status);
    LIS2DUXS12StatusTypeDef Set_FIFO_Mode(uint8_t Mode);
    LIS2DUXS12StatusTypeDef Get_FIFO_Tag(uint8_t *Tag);
    LIS2DUXS12StatusTypeDef Get_FIFO_X_Axes(int32_t *Data);
    LIS2DUXS12StatusTypeDef Set_FIFO_X_BDR(uint8_t Bdr);

    LIS2DUXS12StatusTypeDef Get_X_Event_Status(LIS2DUXS12_Event_Status_t *status);

    LIS2DUXS12StatusTypeDef Enable_Wake_Up_Detection(LIS2DUXS12_SensorIntPin_t IntPin);
    LIS2DUXS12StatusTypeDef Disable_Wake_Up_Detection(void);
    LIS2DUXS12StatusTypeDef Set_Wake_Up_Threshold(uint32_t Threshold);
    LIS2DUXS12StatusTypeDef Set_Wake_Up_Duration(uint8_t Duration);

    LIS2DUXS12StatusTypeDef Enable_6D_Orientation(LIS2DUXS12_SensorIntPin_t IntPin);
    LIS2DUXS12StatusTypeDef Disable_6D_Orientation(void);
    LIS2DUXS12StatusTypeDef Set_6D_Orientation_Threshold(uint8_t thr);
    LIS2DUXS12StatusTypeDef Get_6D_Orientation_XL(uint8_t *XLow);
    LIS2DUXS12StatusTypeDef Get_6D_Orientation_XH(uint8_t *XHigh);
    LIS2DUXS12StatusTypeDef Get_6D_Orientation_YL(uint8_t *ZLow);
    LIS2DUXS12StatusTypeDef Get_6D_Orientation_YH(uint8_t *YHigh);
    LIS2DUXS12StatusTypeDef Get_6D_Orientation_ZL(uint8_t *ZLow);
    LIS2DUXS12StatusTypeDef Get_6D_Orientation_ZH(uint8_t *ZHigh);

    LIS2DUXS12StatusTypeDef Enable_Free_Fall_Detection(LIS2DUXS12_SensorIntPin_t IntPin);
    LIS2DUXS12StatusTypeDef Disable_Free_Fall_Detection(void);
    LIS2DUXS12StatusTypeDef Set_Free_Fall_Threshold(uint8_t Threshold);
    LIS2DUXS12StatusTypeDef Set_Free_Fall_Duration(uint8_t Duration);

    LIS2DUXS12StatusTypeDef Enable_Single_Tap_Detection(LIS2DUXS12_SensorIntPin_t IntPin);
    LIS2DUXS12StatusTypeDef Disable_Single_Tap_Detection();
    LIS2DUXS12StatusTypeDef Enable_Double_Tap_Detection(LIS2DUXS12_SensorIntPin_t IntPin);
    LIS2DUXS12StatusTypeDef Disable_Double_Tap_Detection();
    LIS2DUXS12StatusTypeDef Enable_Triple_Tap_Detection(LIS2DUXS12_SensorIntPin_t IntPin);
    LIS2DUXS12StatusTypeDef Disable_Triple_Tap_Detection();
    LIS2DUXS12StatusTypeDef Set_Tap_Configuration(LIS2DUXS12_Tap_Config_t tap_config);
    LIS2DUXS12StatusTypeDef Get_Tap_Configuration(LIS2DUXS12_Tap_Config_t *tap_config);

    LIS2DUXS12StatusTypeDef Enable_Tilt_Detection(LIS2DUXS12_SensorIntPin_t IntPin);
    LIS2DUXS12StatusTypeDef Disable_Tilt_Detection(void);

    LIS2DUXS12StatusTypeDef Enable_Pedometer(LIS2DUXS12_SensorIntPin_t IntPin);
    LIS2DUXS12StatusTypeDef Disable_Pedometer();
    LIS2DUXS12StatusTypeDef Get_Step_Count(uint16_t *StepCount);
    LIS2DUXS12StatusTypeDef Step_Counter_Reset();

    LIS2DUXS12StatusTypeDef Enable_QVAR();
    LIS2DUXS12StatusTypeDef Disable_QVAR(void);
    LIS2DUXS12StatusTypeDef Get_QVAR_Status(uint8_t *val);
    LIS2DUXS12StatusTypeDef Get_QVAR_Data(float *Data);
    LIS2DUXS12StatusTypeDef Get_QVAR_Impedance(uint8_t *val);
    LIS2DUXS12StatusTypeDef Set_QVAR_Impedance(uint8_t val);
    LIS2DUXS12StatusTypeDef Get_QVAR_Gain(uint8_t *val);
    LIS2DUXS12StatusTypeDef Set_QVAR_Gain(uint8_t val);

    LIS2DUXS12StatusTypeDef Get_MLC_Status(lis2duxs12_mlc_status_mainpage_t *status);
    LIS2DUXS12StatusTypeDef Get_MLC_Output(uint8_t *output);

    LIS2DUXS12StatusTypeDef Read_Reg(uint8_t Reg, uint8_t *Data);
    LIS2DUXS12StatusTypeDef Write_Reg(uint8_t Reg, uint8_t Data);

    /**
     * @brief Utility function to read data.
     * @param  pBuffer: pointer to data to be read.
     * @param  RegisterAddr: specifies internal address register to be read.
     * @param  NumByteToRead: number of bytes to be read.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Read(uint8_t *pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead)
    {
      if (dev_spi) {
        dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));

        digitalWrite(cs_pin, LOW);

        /* Write Reg Address */
        dev_spi->transfer(RegisterAddr | 0x80);
        /* Read the data */
        for (uint16_t i = 0; i < NumByteToRead; i++) {
          *(pBuffer + i) = dev_spi->transfer(0x00);
        }

        digitalWrite(cs_pin, HIGH);

        dev_spi->endTransaction();

        return 0;
      }

      if (dev_i2c) {
        dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));
        dev_i2c->write(RegisterAddr);
        dev_i2c->endTransmission(false);

        dev_i2c->requestFrom(((uint8_t)(((address) >> 1) & 0x7F)), (uint8_t) NumByteToRead);

        int i = 0;
        while (dev_i2c->available()) {
          pBuffer[i] = dev_i2c->read();
          i++;
        }

        return 0;
      }

      return 1;
    }

    /**
     * @brief Utility function to write data.
     * @param  pBuffer: pointer to data to be written.
     * @param  RegisterAddr: specifies internal address register to be written.
     * @param  NumByteToWrite: number of bytes to write.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Write(uint8_t *pBuffer, uint8_t RegisterAddr, uint16_t NumByteToWrite)
    {
      if (dev_spi) {
        dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));

        digitalWrite(cs_pin, LOW);

        /* Write Reg Address */
        dev_spi->transfer(RegisterAddr);
        /* Write the data */
        for (uint16_t i = 0; i < NumByteToWrite; i++) {
          dev_spi->transfer(pBuffer[i]);
        }

        digitalWrite(cs_pin, HIGH);

        dev_spi->endTransaction();

        return 0;
      }

      if (dev_i2c) {
        dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));

        dev_i2c->write(RegisterAddr);
        for (uint16_t i = 0 ; i < NumByteToWrite ; i++) {
          dev_i2c->write(pBuffer[i]);
        }

        dev_i2c->endTransmission(true);

        return 0;
      }

      return 1;
    }

  private:
    LIS2DUXS12StatusTypeDef Set_X_ODR_When_Enabled(float odr, LIS2DUXS12_Power_Mode_t Power);
    LIS2DUXS12StatusTypeDef Set_X_ODR_When_Disabled(float odr, LIS2DUXS12_Power_Mode_t Power);
    LIS2DUXS12StatusTypeDef ExitDeepPowerDownSPI(void);
    LIS2DUXS12StatusTypeDef ExitDeepPowerDownI2C(void);
    /* Helper classes. */
    TwoWire *dev_i2c;
    SPIClass *dev_spi;

    /* Configuration */
    uint8_t address;
    int cs_pin;
    uint32_t spi_speed;

    uint8_t X_isInitialized;
    uint8_t X_isEnabled;
    float X_Last_ODR;
    LIS2DUXS12_Power_Mode_t X_Last_Operating_Mode;

    lis2duxs12_ctx_t reg_ctx;
};

#ifdef __cplusplus
extern "C" {
#endif
int32_t LIS2DUXS12_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite);
int32_t LIS2DUXS12_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead);
#ifdef __cplusplus
}
#endif

#endif
