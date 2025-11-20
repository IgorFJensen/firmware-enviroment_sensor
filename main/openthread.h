/*
    Made by Igor Jensen - UFES - LAEEC
    20/11/2025
*/

#ifndef OPENTHREAD_H
#define OPENTHREAD_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdio.h>
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif


#define ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG()              \
    {                                                      \
        .radio_mode = RADIO_MODE_NATIVE,                   \
    }

#define ESP_OPENTHREAD_DEFAULT_HOST_CONFIG()                        \
    {                                                               \
        .host_connection_mode = HOST_CONNECTION_MODE_CLI_USB,       \
        .host_usb_config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT(), \
    }

#define ESP_OPENTHREAD_DEFAULT_PORT_CONFIG()    \
    {                                           \
        .storage_partition_name = "nvs",        \
        .netif_queue_size = 10,                 \
        .task_queue_size = 10,                  \
    }

void ot_task_worker(void *pvParameters);

//init da task de leitura de sensores
esp_err_t sensor_data_task_init(void);
//task dos sensores
void sensor_data_task(void *pvParameters); // Declared as extern in case it's needed elsewhere, though usually just by FreeRTOS.



#ifdef __cplusplus
}
#endif

#endif // OPENTHREAD_WRAPPER_H
