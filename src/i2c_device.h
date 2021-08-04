#ifndef _I2c_DEVICE_H_
#define _I2c_DEVICE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <esp_types.h>
#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/xtensa_api.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "driver/gpio.h"
#include "hal/i2c_types.h"
#include "soc/i2c_caps.h"


#define BH1750_SENSOR_ADDR      CONFIG_BH1750_ADDR /*!< slave address for BH1750 sensor */
#define BH1750_CMD_START        CONFIG_BH1750_OPMODE /*!< Operation mode */


/**************************************************************************/
/*!
    @brief  calibration data
*/
/**************************************************************************/
typedef struct {
  uint16_t dig_T1; ///< temperature compensation value
  int16_t dig_T2;  ///< temperature compensation value
  int16_t dig_T3;  ///< temperature compensation value

  uint16_t dig_P1; ///< pressure compensation value
  int16_t dig_P2;  ///< pressure compensation value
  int16_t dig_P3;  ///< pressure compensation value
  int16_t dig_P4;  ///< pressure compensation value
  int16_t dig_P5;  ///< pressure compensation value
  int16_t dig_P6;  ///< pressure compensation value
  int16_t dig_P7;  ///< pressure compensation value
  int16_t dig_P8;  ///< pressure compensation value
  int16_t dig_P9;  ///< pressure compensation value

  uint8_t dig_H1; ///< humidity compensation value
  int16_t dig_H2; ///< humidity compensation value
  uint8_t dig_H3; ///< humidity compensation value
  int16_t dig_H4; ///< humidity compensation value
  int16_t dig_H5; ///< humidity compensation value
  int8_t dig_H6;  ///< humidity compensation value

  float temp;
} bme280_calib_data;
/*=========================================================================*/

bme280_calib_data _bme280_calib;

int32_t t_fine;


esp_err_t i2c_master_sensor_BMP280(i2c_port_t i2c_num);

void i2c_BMP280_ReadData(void *arg);

esp_err_t i2c_master_sensor_BH1750(i2c_port_t i2c_num, uint8_t *data_h,uint8_t *data_l);

void i2c_BH1750_ReadData(void *arg);



#ifdef __cplusplus
}
#endif

#endif /*_I2c_DEVICE_H_*/