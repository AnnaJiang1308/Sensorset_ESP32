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
#include "esp_log.h"
#include "sdkconfig.h"
#include <stdio.h>


#include "main.h"
#include "i2c_msg.h"
#include "i2c_device.h"



SemaphoreHandle_t print_mux = NULL;




void app_main(void)
{
  print_mux = xSemaphoreCreateMutex();
  ESP_ERROR_CHECK(i2c_master_init());
  //xTaskCreate(i2c_BMP280_ReadData, "i2c_BMP280_ReadData", 1024 * 2, (void *)0, 10, NULL);
  xTaskCreate(i2c_BH1750_ReadData, "i2c_BMP280_ReadData", 1024 * 2, (void *)0, 10, NULL);
  
  
}
