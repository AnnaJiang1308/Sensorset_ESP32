/* i2c - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "driver/i2c.h"
#include "main.h"
#include "i2c_msg.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <stdio.h>

static const char *TAG = "i2c-example";

SemaphoreHandle_t print_mux = NULL;



/**
 * @brief Returns the temperature from the sensor
 */
float readTemperature(i2c_port_t i2c_num)
{
  int32_t var1, var2, t_fine;

  int32_t adc_T;
  i2c_Read_Registor(i2c_num, 250, &adc_T);
  printf("adc_T=%d\n", adc_T);
  adc_T >>= 4;
  var1 = ((((adc_T >> 3) - ((int32_t)_bme280_calib.dig_T1 << 1))) *((int32_t)_bme280_calib.dig_T2)) >>11;

  var2 = (((((adc_T >> 4) - ((int32_t)_bme280_calib.dig_T1)) *((adc_T >> 4) - ((int32_t)_bme280_calib.dig_T1))) >>12) *((int32_t)_bme280_calib.dig_T3)) >>14;

  t_fine = var1 + var2;

  float T = (t_fine * 5 + 128) >> 8;
  T /= 100;
  return T;
}

//TODO:Returns the pressure from the sensor

//TODO:Returns the humidity from the sensor




static esp_err_t i2c_master_sensor_BMP280(i2c_port_t i2c_num)
{

  int ret;
  uint8_t dig_T1[2], dig_T2[2], dig_T3[2];
  
  uint8_t config=(STANDBY<<5)|(FILTER<<2);
  uint8_t ctrl=(OVERSAMPLING_TEMPERATURE<<5)|(OVERSAMPLING_PRESSURE<<2)|MODE;
  uint8_t ctrl_hum=OVERSAMPLING_HUMIDITY;
  i2c_cmd_handle_t cmd;

  
  i2c_Write_Registor(i2c_num,BMP280_RESET,BMP280_RESET_VALUE);
  i2c_Write_Registor(i2c_num,BMP280_REG_CONFIG,config);
  i2c_Write_Registor(i2c_num,BMP280_REG_CTRL_HUM,ctrl_hum);
  i2c_Write_Registor(i2c_num,BMP280_REG_CTRL,ctrl);
  /*
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, BMP280_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, 0xE0, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, BMP280_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, BMP280_CMD_START, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK)
  {
    return ret;
  }
  vTaskDelay(30 / portTICK_RATE_MS);
  
*/
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, BMP280_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, 0x88, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, BMP280_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);

  i2c_master_read(cmd, dig_T1, 2, ACK_VAL);
  i2c_master_read(cmd, dig_T2, 2, ACK_VAL);
  i2c_master_read(cmd, dig_T3, 2, NACK_VAL);

  /*
  i2c_master_read(cmd,&_bme280_calib.dig_T1,2,ACK_VAL);
  i2c_master_read(cmd,&_bme280_calib.dig_P1,2,ACK_VAL);
  i2c_master_read(cmd,&_bme280_calib.dig_P2,2,ACK_VAL);
  i2c_master_read(cmd,&_bme280_calib.dig_P3,2,ACK_VAL);
  i2c_master_read(cmd,&_bme280_calib.dig_P4,2,ACK_VAL);
  i2c_master_read(cmd,&_bme280_calib.dig_P5,2,ACK_VAL);
  i2c_master_read(cmd,&_bme280_calib.dig_P6,2,ACK_VAL);
  i2c_master_read(cmd,&_bme280_calib.dig_P7,2,ACK_VAL);
  i2c_master_read(cmd,&_bme280_calib.dig_P8,2,ACK_VAL);
  i2c_master_read(cmd,&_bme280_calib.dig_P9,2,ACK_VAL);
  //FIXME:the way reading dig_H2,H4,H5 is false
  i2c_master_read_byte(cmd,&_bme280_calib.dig_H1,ACK_VAL);
  i2c_master_read(cmd,&_bme280_calib.dig_H2,2,ACK_VAL);
  i2c_master_read_byte(cmd,&_bme280_calib.dig_H3,ACK_VAL);
  i2c_master_read(cmd,&_bme280_calib.dig_H4,2,ACK_VAL);
  i2c_master_read(cmd,&_bme280_calib.dig_H5,2,ACK_VAL);
  i2c_master_read_byte(cmd,&_bme280_calib.dig_H6,NACK_VAL);
  */

  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  _bme280_calib.dig_T1 = (uint16_t)(dig_T1[1] << 8) | dig_T1[0];
  _bme280_calib.dig_T2 = (int16_t)(dig_T2[1] << 8) | dig_T2[0];
  _bme280_calib.dig_T3 = (int16_t)(dig_T3[1] << 8) | dig_T3[0];

  printf("T1:%d\tT2:%d\tT3:%d\t\n",_bme280_calib.dig_T1,_bme280_calib.dig_T2,_bme280_calib.dig_T3);

  return ret;
}


static void i2c_test_task(void *arg)
{
  int ret;
  uint32_t task_idx = (uint32_t)arg;

  //float value_Temperature;
  //uint8_t sensor_data_h, sensor_data_l;
  int cnt = 0;
  float temperature = 0;

  while (1)
  {

    ESP_LOGI(TAG, "TASK[%d] test cnt: %d", task_idx, cnt++);
    ret = i2c_master_sensor_BMP280(I2C_MASTER_NUM);
    xSemaphoreTake(print_mux, portMAX_DELAY);

    
    temperature = readTemperature(I2C_MASTER_NUM);

    if (ret == ESP_ERR_TIMEOUT)
    {
      ESP_LOGE(TAG, "I2C Timeout");
    }
    else if (ret == ESP_OK)
    {

      printf("*******************\n");
      printf("TASK[%d]  MASTER READ SENSOR( BMP280 )\n", task_idx);
      printf("*******************\n");
      printf("The temperature is %f \n", temperature);
     
    }
    else
    {
      ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...",
               esp_err_to_name(ret));
    }
    xSemaphoreGive(print_mux);
    vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS * (task_idx + 1)) /
               portTICK_RATE_MS);
  }

  vSemaphoreDelete(print_mux);
  vTaskDelete(NULL);
}

void app_main(void)
{
  print_mux = xSemaphoreCreateMutex();
  ESP_ERROR_CHECK(i2c_master_init());
  xTaskCreate(i2c_test_task, "i2c_test_task_0", 1024 * 2, (void *)0, 10, NULL);
  
  
}
