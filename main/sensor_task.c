#include "sensor_task.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_openthread.h"
#include "esp_openthread_lock.h"
#include "openthread/thread.h"
#include "openthread/udp.h"

// Drivers
#include "dps310.h"
#include "sgp40.h"
#include "veml7700.h"
#include "as7341.h"
#include "shtc1.h"

#include "mqtt_handler.h"

static const char *TAG = "SENSOR_TASK";

#define SENSOR_TASK_STACK_SIZE 4096
#define SENSOR_TASK_PRIORITY   4

static void sensor_loop_task(void *pvParameters)
{
    char payload[512];
    
    // Espera inicial
    vTaskDelay(pdMS_TO_TICKS(2000));

    otInstance *instance = esp_openthread_get_instance();
    if (!instance) {
        ESP_LOGE(TAG, "OpenThread instance não encontrada!");
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "Aguardando rede OpenThread...");
    while (otThreadGetDeviceRole(instance) == OT_DEVICE_ROLE_DISABLED ||
           otThreadGetDeviceRole(instance) == OT_DEVICE_ROLE_DETACHED) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "Rede Thread conectada! Role: %d", otThreadGetDeviceRole(instance));

    mqtt_app_start();

    // Configuração UDP
    otIp6Address peerAddress;
    otIp6AddressFromString("ff03::1", &peerAddress);
    uint16_t peerPort = 1234;
    otUdpSocket socket;

    esp_openthread_lock_acquire(portMAX_DELAY);
    memset(&socket, 0, sizeof(socket));
    otUdpOpen(instance, &socket, NULL, NULL); // Ignorando erro aqui pois é void em alguns SDKs ou tratado internamente
    esp_openthread_lock_release();

    shtc1_reading_t humid;
    float temp, press;
    uint16_t voc, white_raw;
    int32_t vocindex;
    as7341_spectral_data_t spectral_data = {0};

    while (1) {
        int dps_ret = dps310_read(&temp, &press);
        int veml_ret = veml7700_read_white(&white_raw);
        int sht_ret = shtc1_read_data(&humid);
        int sgp_ret = sgp40_measure_compensated(humid.humidity, temp, &voc);
        int sgpidx_ret = sgp40_get_voc_index(humid.humidity, temp, &vocindex);
        int as_ret = as7341_read_all_channels(&spectral_data);

        if (as_ret == ESP_OK) {
            ESP_LOGI(TAG, "AS7341 Read OK. F1:%d NIR:%d", spectral_data.f1, spectral_data.nir);
        }

        // Verificamos sgpidx_ret agora para evitar warning de unused variable
        if (dps_ret == ESP_OK && sgp_ret == ESP_OK && veml_ret == ESP_OK && sht_ret == ESP_OK && sgpidx_ret == ESP_OK) {
            
            ESP_LOGI(TAG, "Leitura: T=%.2f P=%.2f VOC=%d Lux=%d", temp, press, voc, white_raw);

            int len = snprintf(payload, sizeof(payload),
                    "{\"temp\":%.2f,\"press\":%.2f,\"voc\":%u,\"vocindex\":%lu,"
                    "\"humid\":%.2f,\"lux\":%u}",
                    temp, press, voc, vocindex, humid.humidity, white_raw);

            if (len > 0) {
                if (mqtt_is_connected()) {
                    int msg_id = mqtt_publish_sensor_data(payload, len);
                    ESP_LOGI(TAG, "Enviado MQTT ID: %d", msg_id);
                } else {
                    ESP_LOGW(TAG, "MQTT Desconectado, pulando envio.");
                }
            }

            esp_openthread_lock_acquire(portMAX_DELAY);
            otMessage *message = otUdpNewMessage(instance, NULL);
            if (message) {
                otMessageInfo messageInfo;
                memset(&messageInfo, 0, sizeof(messageInfo));
                messageInfo.mPeerAddr = peerAddress;
                messageInfo.mPeerPort = peerPort;
                
                otMessageAppend(message, payload, len);
                otUdpSend(instance, &socket, message, &messageInfo);
                ESP_LOGI(TAG, "Enviado UDP Multicast");
            }
            esp_openthread_lock_release();

        } else {
            ESP_LOGE(TAG, "Erro na leitura dos sensores! (Codigos: %d %d %d %d %d)", 
                dps_ret, sgp_ret, veml_ret, sht_ret, sgpidx_ret);
        }

        vTaskDelay(pdMS_TO_TICKS(20000));
    }
}

esp_err_t sensor_task_start(void)
{
    BaseType_t res = xTaskCreate(sensor_loop_task, "sensor_task", SENSOR_TASK_STACK_SIZE, NULL, SENSOR_TASK_PRIORITY, NULL);
    return (res == pdPASS) ? ESP_OK : ESP_FAIL;
}