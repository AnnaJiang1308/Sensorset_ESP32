#ifndef _I2c_MSG_H_
#define _I2c_MSG_H_

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


#define BMP280_RESET        0xE0
#define BMP280_REG_CONFIG   0xF5
#define BMP280_REG_CTRL_HUM 0xF2
#define BMP280_REG_CTRL     0xF4
#define BMP280_RESET_VALUE  0xB6

#define STANDBY     3
#define FILTER      2
#define MODE        2
#define OVERSAMPLING_PRESSURE       3
#define OVERSAMPLING_TEMPERATURE    3
#define OVERSAMPLING_HUMIDITY       3

esp_err_t i2c_master_init(void);
/**
 * @brief Initation I2C
 *
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */

esp_err_t i2c_Read_Registor(i2c_port_t i2c_num, uint8_t ADD, int32_t *data);
/**
 * @brief get I2C data of specific Registor
 *
 * @param i2c_num I2C port number
 * @param ADD address of the registor need to be readen
 * @param data  address of the data to be read
 * 
 *
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */


esp_err_t i2c_Write_Registor(i2c_port_t i2c_num, uint8_t ADD, int8_t data);
/**
 * @brief get I2C data of specific Registor
 *
 * @param i2c_num I2C port number
 * @param ADD address of the registor need to be readen
 * @param data  I2C one byte command to write to bus
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
*/

#ifdef __cplusplus
}
#endif

#endif /*_I2c_MSG_H_*/