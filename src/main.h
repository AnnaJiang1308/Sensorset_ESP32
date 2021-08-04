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
#include "i2c_device.h"






#ifdef __cplusplus
}
#endif

#endif /*_MAIN_H_*/