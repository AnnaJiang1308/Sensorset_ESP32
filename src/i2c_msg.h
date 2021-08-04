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

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512    /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128 /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS                                            \
  1000 /*!< delay time between different test items */



#define I2C_MASTER_SCL_IO  CONFIG_I2C_MASTER_SCL                                                \
 /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO  CONFIG_I2C_MASTER_SDA                                                    \
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

esp_err_t i2c_Read_Registor_24(i2c_port_t i2c_num, uint8_t ADD, int32_t *data);
/**
 * @brief get I2C data of specific Registor in 3byte
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

esp_err_t i2c_Read_Registor_16(i2c_port_t i2c_num, uint8_t ADD, uint16_t *data);
/**
 * @brief get I2C data of specific Registor in 2byte
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