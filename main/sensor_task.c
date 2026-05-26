/* main/sensor_task.c */
#include "sensor_task.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <inttypes.h> // Necessário para imprimir int32_t (PRIi32) corretamente
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_openthread.h"
#include "openthread/thread.h"

// Drivers
#include "dps310.h"
#include "sgp40.h"
#include "veml7700.h"
#include "as7341.h"
#include "sht40.h"

#include "mqtt_handler.h"

static const char *TAG = "SENSOR_TASK";

#define SENSOR_TASK_STACK_SIZE 4096
#define SENSOR_TASK_PRIORITY   4

static void sensor_loop_task(void *pvParameters)
{
    char payload[512];
    
    // Espera inicial para estabilização dos sensores
    // vTaskDelay(pdMS_TO_TICKS(2000));

    // otInstance *instance = esp_openthread_get_instance();
    // if (!instance) {
    //     ESP_LOGE(TAG, "OpenThread instance não encontrada!");
    //     vTaskDelete(NULL);
    //     return;
    // }

    // ESP_LOGI(TAG, "Aguardando rede OpenThread...");
    // while (otThreadGetDeviceRole(instance) == OT_DEVICE_ROLE_DISABLED ||
    //        otThreadGetDeviceRole(instance) == OT_DEVICE_ROLE_DETACHED) {
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }
    // ESP_LOGI(TAG, "Rede Thread conectada! Role: %d", otThreadGetDeviceRole(instance));

    // Inicia MQTT apenas quando a rede estiver pronta
    //mqtt_app_start();

    // Variáveis de Leitura
    sht40_reading_t humid;    
    float temp = 0.0f, press = 0.0f;
    uint16_t voc = 0, white_raw = 0;
    int32_t vocindex = 0;
    as7341_spectral_data_t spectral_data = {0};

//     while (1) {
//         // Leitura dos sensores
//         int dps_ret = dps310_read(&temp, &press);
//         int veml_ret = veml7700_read_white(&white_raw);
//         int sht_ret = sht40_read_data(&humid);
//         int sgp_ret = sgp40_measure_compensated(humid.humidity, temp, &voc);
//         int sgpidx_ret = sgp40_get_voc_index(humid.humidity, temp, &vocindex);
//         int as_ret = as7341_read_all_channels(&spectral_data);

//         if (as_ret == ESP_OK) {
//             ESP_LOGI(TAG, "AS7341 Read OK. F1:%u NIR:%u", spectral_data.f1, spectral_data.nir);
//         }

//         // Verifica sucesso das leituras
//         if (dps_ret == ESP_OK && sgp_ret == ESP_OK && veml_ret == ESP_OK && sht_ret == ESP_OK && sgpidx_ret == ESP_OK) {
            
//             // Log das leituras (Usando PRIi32 para o int32_t)
//             ESP_LOGI(TAG, "Leitura: T=%.2f P=%.2f VOCidx=%" PRIi32 " Lux=%u", temp, press, vocindex, white_raw);

//             // Formata o JSON
//             int len = snprintf(payload, sizeof(payload),
//                     "{\"temp\":%.2f,\"press\":%.2f,\"voc\":%u,\"vocindex\":%" PRIi32 ","
//                     "\"humid\":%.2f,\"lux\":%u}",
//                     temp, press, voc, vocindex, humid.humidity, white_raw);

//             if (len > 0) {
//                 if (mqtt_is_connected()) {
//                     int msg_id = mqtt_publish_sensor_data(payload, len);
//                     ESP_LOGI(TAG, "Enviado MQTT ID: %d", msg_id);
//                 } else {
//                     ESP_LOGW(TAG, "MQTT Desconectado, pulando envio.");
//                 }
//             }

//         } else {
//             ESP_LOGE(TAG, "Erro na leitura dos sensores! (Codigos: %d %d %d %d %d)", 
//                 dps_ret, sgp_ret, veml_ret, sht_ret, sgpidx_ret);
//         }

//         vTaskDelay(pdMS_TO_TICKS(10000)); // 20 segundos
//     }
while (1) {
        // Leitura dos sensores
        int dps_ret = dps310_read(&temp, &press);
        int veml_ret = veml7700_read_white(&white_raw);
        int sht_ret = sht40_read_data(&humid);
        int sgp_ret = sgp40_measure_compensated(humid.humidity, temp, &voc);
        int sgpidx_ret = sgp40_get_voc_index(humid.humidity, temp, &vocindex);
        
        // Leitura do AS7341
        int as_ret = as7341_read_all_channels(&spectral_data);
        
        if (sht_ret == ESP_OK) 
            printf("SHT40:  T=%.2f C, H=%.2f %%\n", humid.temperature, humid.humidity);
        else 
            printf("SHT40:  ERRO (0x%X)\n", sht_ret);

        if (dps_ret == ESP_OK) 
            printf("DPS368: P=%.2f Pa, T_int=%.2f C\n", press, temp);
        else 
            printf("DPS368: ERRO (0x%X)\n", dps_ret);

        if (veml_ret == ESP_OK) 
            printf("VEML:   Lux=%u\n", white_raw);
        else 
            printf("VEML:   ERRO (0x%X)\n", veml_ret);

        // Print do AS7341 (Canais Espectrais)
        if (as_ret == ESP_OK) {
            printf("AS7341: F1-F4: %u, %u, %u, %u\n", spectral_data.f1, spectral_data.f2, spectral_data.f3, spectral_data.f4);
            printf("        F5-F8: %u, %u, %u, %u\n", spectral_data.f5, spectral_data.f6, spectral_data.f7, spectral_data.f8);
            printf("        NIR/CLR: %u / %u\n", spectral_data.nir, spectral_data.clear);
        } else {
            printf("AS7341: ERRO (0x%X)\n", as_ret);
        }

        printf("SGP40:  VOC_Idx=%" PRIi32 "\n", vocindex);
        printf("--------------------------\n");
        fflush(stdout);

        // Mantém a lógica original para envio MQTT
        if (dps_ret == ESP_OK && sgp_ret == ESP_OK && veml_ret == ESP_OK && sht_ret == ESP_OK && as_ret == ESP_OK) {
            
            // Sugestão: Adicionar os dados espectrais no JSON do seu TCC
            int len = snprintf(payload, sizeof(payload),
                    "{\"temp\":%.2f,\"press\":%.2f,\"voc\":%u,\"vocindex\":%" PRIi32 ","
                    "\"humid\":%.2f,\"lux\":%u,\"f1\":%u,\"f4\":%u,\"nir\":%u}",
                    temp, press, voc, vocindex, humid.humidity, white_raw, 
                    spectral_data.f1, spectral_data.f4, spectral_data.nir);

            if (len > 0 && mqtt_is_connected()) {
                int msg_id = mqtt_publish_sensor_data(payload, len);
                ESP_LOGI(TAG, "Enviado MQTT ID: %d", msg_id);
            }
        } else {
            ESP_LOGE(TAG, "Falha I2C Detectada! SHT:%d DPS:%d AS:%d", 
                     sht_ret, dps_ret, as_ret);
        }

        vTaskDelay(pdMS_TO_TICKS(2000)); // Delay de 2 segundos para monitoramento mais fluido
    }
 }


esp_err_t sensor_task_start(void)
{
    BaseType_t res = xTaskCreate(sensor_loop_task, "sensor_task", SENSOR_TASK_STACK_SIZE, NULL, SENSOR_TASK_PRIORITY, NULL);
    return (res == pdPASS) ? ESP_OK : ESP_FAIL;
}