#ifndef SYSTEM_INIT_H
#define SYSTEM_INIT_H

#include "esp_err.h"
#include "esp_openthread_types.h" // Necessário para os tipos dentro das macros

#ifdef __cplusplus
extern "C" {
#endif

// --- Definições de Configuração (Trazidas do seu código original) ---

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

// --- Protótipos ---

// Task principal do OpenThread (Worker)
void ot_task_worker(void *aContext);

#ifdef __cplusplus
}
#endif

#endif // SYSTEM_INIT_H