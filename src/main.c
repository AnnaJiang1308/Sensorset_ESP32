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
#include "esp_log.h"
#include "sdkconfig.h"
#include <stdio.h>

static const char *TAG = "i2c-example";



SemaphoreHandle_t print_mux = NULL;

/**
 * @brief Reads a 16 bit unsigned int via I2C
 */
uint16_t i2c_Read16(i2c_port_t i2c_num,i2c_cmd_handle_t cmd){
  uint16_t value_16;
  uint8_t value_8_front=0;
  uint8_t value_8_reverse=0;
  i2c_master_read_byte(cmd, &value_8_reverse, ACK_VAL);
  i2c_master_read_byte(cmd, &value_8_front, ACK_VAL);
  value_16=(value_8_front<<8|value_8_reverse);
  
  return value_16;

}

/**
 * @brief Reads a 16 bit signed int via I2C
 */
int16_t i2c_ReadS16(i2c_port_t i2c_num,i2c_cmd_handle_t* cmd){
  return i2c_Read16(i2c_num, cmd);
}


int32_t i2c_Read24(i2c_port_t i2c_num,i2c_cmd_handle_t* cmd){
  uint32_t value_32;
  uint8_t value_8_1=0;
  uint8_t value_8_2=0;
  uint8_t value_8_3=0;
  i2c_master_read_byte(*cmd, &value_8_1, ACK_VAL);
  i2c_master_read_byte(*cmd, &value_8_2, ACK_VAL);
  i2c_master_read_byte(*cmd, &value_8_3, ACK_VAL);
  value_32=(value_8_1<<16|value_8_2<<8|value_8_3);
  
  return value_32;
}



static esp_err_t i2c_Read_Registor(i2c_port_t i2c_num,uint8_t ADD, uint32_t *data){
 int ret;
 
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, BMP280_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, ADD, ACK_CHECK_EN);
  
  *data=i2c_Read24(i2c_num,cmd);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}


/**
 * @brief Returns the pressure from the sensor
 */
float readTemperature(i2c_port_t i2c_num) {
  int32_t var1, var2,t_fine;
  //TODO:Read adc_T via i2c ADD:[eumu]250

  uint32_t adc_T;
  i2c_Read_Registor(i2c_num,250, &adc_T);
  printf("adc_T=%d",adc_T);
  var1 = ((((adc_T >> 3) - ((int32_t)_bme280_calib.dig_T1 << 1))) *
          ((int32_t)_bme280_calib.dig_T2)) >>
         11;

  var2 = (((((adc_T >> 4) - ((int32_t)_bme280_calib.dig_T1)) *
            ((adc_T >> 4) - ((int32_t)_bme280_calib.dig_T1))) >>
           12) *
          ((int32_t)_bme280_calib.dig_T3)) >>
         14;

  t_fine = var1 + var2;

  float T = (t_fine * 5 + 128) >> 8;
  return T / 100;
}

//TODO:Returns the pressure from the sensor

//TODO:Returns the humidity from the sensor






/**
 * @brief read all the coefficients needed in the caculation
 */
static esp_err_t BMP280_Read_coefficients(i2c_port_t i2c_num){
  int ret;
  uint8_t data=0;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, BMP280_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    //TODO:Read all data via i2c

    
  _bme280_calib.dig_T1=i2c_Read16(i2c_num,&cmd);
  _bme280_calib.dig_T2=i2c_ReadS16(i2c_num,&cmd);
  _bme280_calib.dig_T3=i2c_ReadS16(i2c_num,&cmd);
  //FIXME: Testcode
  printf("value of temp:%d \t %d \t %d \n",_bme280_calib.dig_T1,_bme280_calib.dig_T2,_bme280_calib.dig_T3);


  
  _bme280_calib.dig_H6=i2c_master_read_byte(cmd,&data, NACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}




static esp_err_t i2c_master_sensor_BMP280(i2c_port_t i2c_num) {

  int ret;
  
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, BMP280_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, BMP280_CMD_START, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK) {
    return ret;
  }
  vTaskDelay(30 / portTICK_RATE_MS);

  //FIXME:text code
  //Qusetion:bug of Read16? 

  //ret= BMP280_Read_coefficients(i2c_num);

  uint8_t data1=0;
  uint8_t data2=0;
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, BMP280_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    //TODO:Read all data via i2c
  i2c_master_read_byte(cmd,&data1,ACK_VAL);
  i2c_master_read_byte(cmd,&data2,NACK_VAL);
  
  
  
  
  //_bme280_calib.dig_T1=i2c_Read16(i2c_num,cmd);
  //_bme280_calib.dig_T2=i2c_ReadS16(i2c_num,&cmd);
  //_bme280_calib.dig_T3=i2c_ReadS16(i2c_num,&cmd);
  //printf("value of temp:%d \t %d \t %d \n",_bme280_calib.dig_T1,_bme280_calib.dig_T2,_bme280_calib.dig_T3);


  
  //_bme280_calib.dig_H6=i2c_master_read_byte(cmd,&data, NACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  printf("data1:%d\tdata2:%d\n",data1,data2);
  return ret;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void) {
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


static void i2c_test_task(void *arg) {
  int ret;
  uint32_t task_idx = (uint32_t)arg;

  //float value_Temperature;
  //uint8_t sensor_data_h, sensor_data_l;
  int cnt = 0;

  while (1) {

    ESP_LOGI(TAG, "TASK[%d] test cnt: %d", task_idx, cnt++);
    ret=i2c_master_sensor_BMP280(I2C_MASTER_NUM);
    
    xSemaphoreTake(print_mux, portMAX_DELAY);

    
    if (ret == ESP_ERR_TIMEOUT) {
      ESP_LOGE(TAG, "I2C Timeout");
    } else if (ret == ESP_OK) {


      
      printf("*******************\n");
      printf("TASK[%d]  MASTER READ SENSOR( BMP280 )\n", task_idx);
      printf("*******************\n");

      /*

      printf("sensor val: %.02f  °C \n",value_Temperature);
      
      printf("data_h: %02x\n", sensor_data_h);
      printf("data_l: %02x\n", sensor_data_l);
      printf("sensor val: %.02f [Lux]\n",
             (sensor_data_h << 8 | sensor_data_l) / 1.2);

      */
     
    } else {
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

void app_main(void) {
  print_mux = xSemaphoreCreateMutex();
  ESP_ERROR_CHECK(i2c_master_init());
  xTaskCreate(i2c_test_task, "i2c_test_task_0", 1024 * 2, (void *)0, 10, NULL);
  
}
