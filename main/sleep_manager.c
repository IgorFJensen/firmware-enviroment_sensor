#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "system_init.h"
#include "dps310.h"
#include "sgp41.h"
#include "veml7700.h"
#include "as7341.h"
#include "sht40.h"
#include "sleep_manager.h"

static const char *TAG = "SLEEP_MGR";

#define SLEEP_INTERVAL_SEC 15ULL
#define ACTIVE_TIME_MS (8*1000)

typedef struct {
    i2c_master_bus_handle_t bus_handle;
} sleep_manager_args_t;

static void sleep_manager_task(void *pv)
{
    sleep_manager_args_t *args = (sleep_manager_args_t *) pv;
    i2c_master_bus_handle_t bus_handle = args->bus_handle;
    vPortFree(args);

    vTaskDelay(pdMS_TO_TICKS(ACTIVE_TIME_MS));

    ESP_LOGI(TAG, "Preparando para deep-sleep: salvando dataset e desligando sensores (GPIO19)");
    save_openthread_dataset();
    as7341_deinit();
    vTaskDelay(pdMS_TO_TICKS(50));

    esp_err_t timer_res = esp_sleep_enable_timer_wakeup(SLEEP_INTERVAL_SEC * 1000000ULL);
    if (timer_res != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao habilitar timer wakeup: %s", esp_err_to_name(timer_res));
    }

    esp_err_t pd_res = esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    if (pd_res != ESP_OK) {
        ESP_LOGW(TAG, "Falha ao configurar RTC_PERIPH para sleep: %s", esp_err_to_name(pd_res));
    }

    ESP_LOGI(TAG, "Entrando em light-sleep por %llu segundos...", SLEEP_INTERVAL_SEC);
    esp_err_t ls_res = esp_light_sleep_start();
    if (ls_res != ESP_OK) {
        ESP_LOGW(TAG, "esp_light_sleep_start retornou %s", esp_err_to_name(ls_res));
    } else {
        ESP_LOGI(TAG, "Acordei do light-sleep");
    }

    vTaskDelete(NULL);
}

esp_err_t sleep_manager_start(i2c_master_bus_handle_t bus_handle)
{
    sleep_manager_args_t *args = pvPortMalloc(sizeof(*args));
    if (args == NULL) {
        return ESP_ERR_NO_MEM;
    }

    args->bus_handle = bus_handle;
    if (xTaskCreate(sleep_manager_task, "sleep_mgr", 4096, args, 2, NULL) != pdPASS) {
        vPortFree(args);
        return ESP_FAIL;
    }

    return ESP_OK;
}
