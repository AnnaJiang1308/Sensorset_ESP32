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
  int32_t var1, var2;

  int32_t adc_T;
  i2c_Read_Registor_24(i2c_num, 250, &adc_T);
  
  adc_T >>= 4;
  var1 = ((((adc_T >> 3) - ((int32_t)_bme280_calib.dig_T1 << 1))) *((int32_t)_bme280_calib.dig_T2)) >>11;

  var2 = (((((adc_T >> 4) - ((int32_t)_bme280_calib.dig_T1)) *((adc_T >> 4) - ((int32_t)_bme280_calib.dig_T1))) >>12) *((int32_t)_bme280_calib.dig_T3)) >>14;

  t_fine = var1 + var2;

  float T = (t_fine * 5 + 128) >> 8;
  T /= 100;
  return T;
}


float readPressure(i2c_port_t i2c_num) {
  int64_t var1, var2, p;

  readTemperature( i2c_num); // must be done first to get t_fine

  int32_t adc_P;
  
  i2c_Read_Registor_24(i2c_num, 0xF7, &adc_P);

  //FIXME: test code
  printf("adc_P%d\n", adc_P);
  

  adc_P >>= 4;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)_bme280_calib.dig_P6;
  var2 = var2 + ((var1 * (int64_t)_bme280_calib.dig_P5) << 17);
  var2 = var2 + (((int64_t)_bme280_calib.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)_bme280_calib.dig_P3) >> 8) +
         ((var1 * (int64_t)_bme280_calib.dig_P2) << 12);
  var1 =
      (((((int64_t)1) << 47) + var1)) * ((int64_t)_bme280_calib.dig_P1) >> 33;

  if (var1 == 0) {
    return 0; // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)_bme280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)_bme280_calib.dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)_bme280_calib.dig_P7) << 4);
  return (float)p / 256;
}

float readHumidity(i2c_port_t i2c_num) {
  readTemperature(i2c_num); // must be done first to get t_fine

  int32_t adc_H;
  uint16_t adc_H_read;
  i2c_Read_Registor_16(i2c_num, 0xFD, &adc_H_read);
  adc_H=adc_H_read;

  printf("adc_H=%d\n", adc_H);

  int32_t v_x1_u32r;

  v_x1_u32r = (t_fine - ((int32_t)76800));

  v_x1_u32r = (((((adc_H << 14) - (((int32_t)_bme280_calib.dig_H4) << 20) -
                  (((int32_t)_bme280_calib.dig_H5) * v_x1_u32r)) +
                 ((int32_t)16384)) >>
                15) *
               (((((((v_x1_u32r * ((int32_t)_bme280_calib.dig_H6)) >> 10) *
                    (((v_x1_u32r * ((int32_t)_bme280_calib.dig_H3)) >> 11) +
                     ((int32_t)32768))) >>
                   10) +
                  ((int32_t)2097152)) *
                     ((int32_t)_bme280_calib.dig_H2) +
                 8192) >>
                14));

  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                             ((int32_t)_bme280_calib.dig_H1)) >>
                            4));

  v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
  v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
  float h = (v_x1_u32r >> 12);
  return h / 1024.0;
}



static esp_err_t i2c_master_sensor_BMP280(i2c_port_t i2c_num)
{

  int ret;
  uint8_t dig_T1[2], dig_T2[2], dig_T3[2];
  uint8_t dig_P1[2],dig_P2[2],dig_P3[2],dig_P4[2],dig_P5[2],dig_P6[2],dig_P7[2],dig_P8[2],dig_P9[2];
  uint8_t dig_H2[2],dig_H5[2];
  uint8_t dig_H4,dig_H6;

  uint8_t config=(STANDBY<<5)|(FILTER<<2);
  uint8_t ctrl=(OVERSAMPLING_TEMPERATURE<<5)|(OVERSAMPLING_PRESSURE<<2)|MODE;
  uint8_t ctrl_hum=OVERSAMPLING_HUMIDITY;
  i2c_cmd_handle_t cmd;

  
  i2c_Write_Registor(i2c_num,BMP280_RESET,BMP280_RESET_VALUE);
  i2c_Write_Registor(i2c_num,BMP280_REG_CONFIG,config);
  i2c_Write_Registor(i2c_num,BMP280_REG_CTRL_HUM,ctrl_hum);
  i2c_Write_Registor(i2c_num,BMP280_REG_CTRL,ctrl);

  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, BMP280_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, 0x88, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, BMP280_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);

  i2c_master_read(cmd, dig_T1, 2, ACK_VAL);
  i2c_master_read(cmd, dig_T2, 2, ACK_VAL);
  i2c_master_read(cmd, dig_T3, 2, ACK_VAL);

  i2c_master_read(cmd,dig_P1,2,ACK_VAL);
  //i2c_master_read(cmd,(uint8_t *)&(_bme280_calib.dig_P1),2,ACK_VAL);
  i2c_master_read(cmd,dig_P2,2,ACK_VAL);
  i2c_master_read(cmd,dig_P3,2,ACK_VAL);
  i2c_master_read(cmd,dig_P4,2,ACK_VAL);
  i2c_master_read(cmd,dig_P5,2,ACK_VAL);
  i2c_master_read(cmd,dig_P6,2,ACK_VAL);
  i2c_master_read(cmd,dig_P7,2,ACK_VAL);
  i2c_master_read(cmd,dig_P8,2,ACK_VAL);
  i2c_master_read(cmd,dig_P9,2,ACK_VAL);
  
  i2c_master_read_byte(cmd,&_bme280_calib.dig_H1,NACK_VAL);
  

  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);



  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, BMP280_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, 0xE1, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, BMP280_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);

  i2c_master_read(cmd,dig_H2,2,ACK_VAL);
  i2c_master_read_byte(cmd,&_bme280_calib.dig_H3,ACK_VAL);
  i2c_master_read_byte(cmd,&dig_H4,ACK_VAL);
  i2c_master_read(cmd,dig_H5,2,ACK_VAL);
  i2c_master_read_byte(cmd,&dig_H6,NACK_VAL);
  

  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);


  //TODO: fixing of the coeffience
  _bme280_calib.dig_T1 = (uint16_t)(dig_T1[1] << 8) | dig_T1[0];
  _bme280_calib.dig_T2 = (int16_t)(dig_T2[1] << 8) | dig_T2[0];
  _bme280_calib.dig_T3 = (int16_t)(dig_T3[1] << 8) | dig_T3[0];

  _bme280_calib.dig_P1 = (uint16_t)(dig_P1[1] << 8) | dig_P1[0];
  _bme280_calib.dig_P2 = (int16_t)(dig_P2[1] << 8) | dig_P2[0];
  _bme280_calib.dig_P3 = (int16_t)(dig_P3[1] << 8) | dig_P3[0];
  _bme280_calib.dig_P4 = (int16_t)(dig_P4[1] << 8) | dig_P4[0];
  _bme280_calib.dig_P5 = (int16_t)(dig_P5[1] << 8) | dig_P5[0];
  _bme280_calib.dig_P6 = (int16_t)(dig_P6[1] << 8) | dig_P6[0];
  _bme280_calib.dig_P7 = (int16_t)(dig_P7[1] << 8) | dig_P7[0];
  _bme280_calib.dig_P8 = (int16_t)(dig_P8[1] << 8) | dig_P8[0];
  _bme280_calib.dig_P9 = (int16_t)(dig_P9[1] << 8) | dig_P9[0];

  _bme280_calib.dig_H2 = (int16_t)(dig_H2[1] << 8) | dig_H2[0];
  _bme280_calib.dig_H4 = ((int8_t)dig_H4 << 4)  |(dig_H5[0] & 0xF);
  _bme280_calib.dig_H5 = ((int8_t)dig_H5[1] << 4)  |(dig_H5[0]  >> 4);
  _bme280_calib.dig_H6 = (int8_t)dig_H6;



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
  float Pressure=0;
  float Humidity=0;

  while (1)
  {

    ESP_LOGI(TAG, "TASK[%d] test cnt: %d", task_idx, cnt++);
    ret = i2c_master_sensor_BMP280(I2C_MASTER_NUM);
    xSemaphoreTake(print_mux, portMAX_DELAY);

    
    temperature = readTemperature(I2C_MASTER_NUM);
    Pressure=readPressure(I2C_MASTER_NUM)/100.0F;
    Humidity=readHumidity(I2C_MASTER_NUM);

    if (ret == ESP_ERR_TIMEOUT)
    {
      ESP_LOGE(TAG, "I2C Timeout");
    }
    else if (ret == ESP_OK)
    {
      printf("*******************\n");
      printf("TASK[%d]  MASTER READ SENSOR( BMP280 )\n", task_idx);
      printf("*******************\n");
      printf("The temperature is %.2f *C \n ", temperature);
      printf("The pressure is %.2f hPa\n",Pressure);
      printf("The humidity is %.2f %% \n",Humidity);
     
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
