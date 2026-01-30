// Made by Agenor Dias
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_vfs_eventfd.h"
#include "nvs_flash.h"
#include "esp_heap_caps.h"
#include "driver/i2c_master.h"

// Inclusão módulos
#include "system_init.h"
#include "sensor_task.h"

// Drivers de sensores (apenas para init)
#include "dps310.h"
#include "sgp40.h"
#include "veml7700.h"
#include "as7341.h"
#include "sht4x.h"

static const char *TAG = "MAIN";

i2c_master_bus_handle_t bus_handle;

void app_main(void)
{
    ESP_LOGI(TAG, "--- Inicializando Sistema ---");

    //Inicialização de Sistema Básico (NVS, Netif, EventLoop)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());
    
    esp_vfs_eventfd_config_t eventfd_config = { .max_fds = 5 };
    ESP_ERROR_CHECK(esp_vfs_eventfd_register(&eventfd_config));

    //Inicialização de Hardware (I2C)
    ESP_LOGI(TAG, "Configurando I2C...");
    i2c_master_bus_config_t bus_config = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = GPIO_NUM_4,
            .scl_io_num = GPIO_NUM_5,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
    };
    
    if (i2c_new_master_bus(&bus_config, &bus_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Erro fatal: Não foi possível criar barramento I2C");
        return;
    }

    // Inicialização dos Sensores
    ESP_LOGI(TAG, "Inicializando Sensores...");
    if (dps310_init(bus_handle) != ESP_OK) ESP_LOGE(TAG, "Falha DPS310");
    if (sgp40_init(bus_handle) != ESP_OK) ESP_LOGE(TAG, "Falha SGP40");
    if (veml7700_init(bus_handle) != ESP_OK) ESP_LOGE(TAG, "Falha VEML7700");
    if (as7341_init(bus_handle) != ESP_OK) ESP_LOGE(TAG, "Falha AS7341");
    if (sht4x_init(bus_handle) != ESP_OK) ESP_LOGE(TAG, "Falha SHT4X");

    //Criação da Task do OpenThread
    ESP_LOGI(TAG, "Iniciando Task OpenThread...");
    xTaskCreate(ot_task_worker, "ot_task", 10240, xTaskGetCurrentTaskHandle(), 5, NULL);

    //Criação da Task de Aplicação (Sensores + MQTT)
    ESP_LOGI(TAG, "Iniciando Task de Sensores...");
    if (sensor_task_start() != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao criar task de sensores");
    }

    ESP_LOGI(TAG, "Main finalizada (Tasks rodando).");
}