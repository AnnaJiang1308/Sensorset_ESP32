#ifndef _MAIN_H_
#define _MAIN_H_

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
#include "i2c_msg.h"

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512    /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128 /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS                                            \
  1000 /*!< delay time between different test items */



#define I2C_MASTER_SCL_IO  22                                                 \
 /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO  21                                                    \
/*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM                                                         \
  I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev   \
                                          */
#define I2C_MASTER_FREQ_HZ                                                     \
  CONFIG_I2C_MASTER_FREQUENCY       /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */

#define BMP280_SENSOR_ADDR    0x76 /*!< slave address for BMP280 sensor */
#define BMP280_CMD_START      0xB6 /*!< Operation mode */


#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0       /*!< I2C ack value */
#define NACK_VAL 0x1      /*!< I2C nack value */



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

#ifdef __cplusplus
}
#endif

#endif /*_MAIN_H_*/