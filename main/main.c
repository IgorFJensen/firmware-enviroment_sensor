#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>  // Para PRIu32
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_vfs_eventfd.h"
#include "nvs_flash.h"
#include "esp_heap_caps.h"
#include "esp_openthread.h"
#include "esp_openthread_cli.h"
#include "esp_openthread_lock.h"
#include "esp_openthread_netif_glue.h"
#include "esp_openthread_types.h"
#include "driver/uart.h"
#include "openthread/thread.h"
#include "openthread/cli.h"
#include "openthread/instance.h"
#include "openthread/logging.h"
#include "openthread/tasklet.h"
#include "openthread.h"

//sensors:
#include "dps310.h"
#include "sgp40.h"
#include "veml7700.h"
#include "as7341.h"
#include "shtc1.h"

#include "driver/i2c_master.h"

static const char *TAG = "main";

#if CONFIG_OPENTHREAD_CLI_ESP_EXTENSION
#include "esp_ot_cli_extension.h"
#endif // CONFIG_OPENTHREAD_CLI_ESP_EXTENSION

i2c_master_bus_handle_t bus_handle;

// Função auxiliar para verificar a heap disponível
void print_heap_status(void) {
    ESP_LOGI(TAG, "Heap livre: %" PRIu32 " bytes", esp_get_free_heap_size());
    heap_caps_print_heap_info(MALLOC_CAP_DEFAULT);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Inicializando sistema...");
    esp_vfs_eventfd_config_t eventfd_config = {
        .max_fds = 5,
    };
    i2c_master_bus_config_t bus_config = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = GPIO_NUM_15,
            .scl_io_num = GPIO_NUM_3,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle);
    //verifica sensores inicializados
    if (err != ESP_OK) {
        printf("Erro ao criar barramento I2C\n");
        return;
    }

    if (dps310_init(bus_handle) != ESP_OK) {
        printf("Falha ao inicializar DPS310\n");
        return;
    }
    if (sgp40_init(bus_handle) != ESP_OK) {
        printf("Falha ao inicializar SGP40\n");
        return;
    }
    if (veml7700_init(bus_handle) != ESP_OK) {
        printf("Falha ao inicializar VEML7700.\n");
    }
     if (as7341_init(bus_handle) != ESP_OK) {
        printf("Falha ao inicializar AS7341.\n");
    }else {
    ESP_LOGI(TAG, "AS7341 inicializado.");
    }
    if(shtc1_init(bus_handle) != ESP_OK){
        printf("Falha ao inicializar AS7341.\n");
    }
    // Inicializa NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());    
    ESP_ERROR_CHECK(esp_vfs_eventfd_register(&eventfd_config));

    // Verifica a heap antes de criar a tarefa
    print_heap_status();

   
    // Cria a tarefa do OpenThread com pilha adequada
    BaseType_t res = xTaskCreate(
        ot_task_worker,
        "ot_task",
        10240,
        xTaskGetCurrentTaskHandle(),
        5,
        NULL
    );
    if (sensor_data_task_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize and start sensor data task!");
    }
   

    ESP_LOGI(TAG, "app_main finalizado.");
}
