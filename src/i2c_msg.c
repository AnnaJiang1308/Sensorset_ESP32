#include "driver/i2c.h"
#include "main.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <stdio.h>


esp_err_t i2c_master_init(void)
{
  int i2c_master_port = I2C_MASTER_NUM;
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_MASTER_SDA_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = I2C_MASTER_SCL_IO;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
  i2c_param_config(i2c_master_port, &conf);
  return i2c_driver_install(i2c_master_port, conf.mode,
                            I2C_MASTER_RX_BUF_DISABLE,
                            I2C_MASTER_TX_BUF_DISABLE, 0);  
}

esp_err_t i2c_Read_Registor(i2c_port_t i2c_num, uint8_t ADD, int32_t *data)
{
  int ret;
  uint8_t value_1 = 0;
  uint8_t value_2 = 0;
  uint8_t value_3 = 0;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, BMP280_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, ADD, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, BMP280_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, &value_1, ACK_VAL);
  i2c_master_read_byte(cmd, &value_2, ACK_VAL);
  i2c_master_read_byte(cmd, &value_3, NACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  *data = (value_1 << 16) | (value_2 << 8) | value_3;
  if (ret != ESP_OK)
  {
    return ret;
  }
  vTaskDelay(30 / portTICK_RATE_MS);
  return ret;
}

esp_err_t i2c_Write_Registor(i2c_port_t i2c_num, uint8_t ADD, int8_t data)
{
  int ret;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, BMP280_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, ADD, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK)
  {
    return ret;
  }
  vTaskDelay(30 / portTICK_RATE_MS);
  return ret;
}