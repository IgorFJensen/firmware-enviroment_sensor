#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
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
#include "openthread/udp.h"
#include "openthread/instance.h"
#include "openthread/logging.h"
#include "openthread/dataset.h"
#include "openthread/tasklet.h"
#include "openthread.h"
#include "mqtt_client.h" 

#include <time.h>

#include "dps310.h"
#include "sgp40.h"
#include "veml7700.h"
#include "as7341.h"
#include "shtc1.h"
#include "sensirion_gas.h"

#include "driver/i2c_master.h"

#if CONFIG_OPENTHREAD_CLI_ESP_EXTENSION
#include "esp_ot_cli_extension.h"
#endif

// --- Variáveis e Tags Globais ---
static const char *TAG = "sensor_task";
static const char *TAG_MQTT = "MQTT_HANDLER";
#define SENSOR_TASK_STACK_SIZE 4096
#define SENSOR_TASK_PRIORITY   4

static esp_mqtt_client_handle_t mqtt_client;
static bool is_mqtt_connected = false; 
// --- Fim Variáveis Globais ---

// --- Protótipos das Funções ---
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
void mqtt_app_start(void);
// --- Fim Protótipos ---


static esp_netif_t *init_openthread_netif(const esp_openthread_platform_config_t *config)
{
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_OPENTHREAD();
    esp_netif_t *netif = esp_netif_new(&cfg);
    assert(netif != NULL);
    ESP_ERROR_CHECK(esp_netif_attach(netif, esp_openthread_netif_glue_init(config)));

    return netif;
}


void ot_cli_restart_command(void *context, int argc, char *argv[])
{
    (void)context;
    (void)argc;
    (void)argv;

    otLogNotePlat("Reiniciando a placa...");
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
}

// Array de comandos CLI (movido para evitar warning)
static const otCliCommand commands[] = {
    {"restart", NULL, ot_cli_restart_command},
};

void ot_task_worker(void *aContext)
{
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };

    ESP_ERROR_CHECK(esp_openthread_init(&config));

#if CONFIG_OPENTHREAD_STATE_INDICATOR_ENABLE
    ESP_ERROR_CHECK(esp_openthread_state_indicator_init(esp_openthread_get_instance()));
#endif

#if CONFIG_OPENTHREAD_LOG_LEVEL_DYNAMIC
    (void)otLoggingSetLevel(CONFIG_LOG_DEFAULT_LEVEL);
#endif

#if CONFIG_OPENTHREAD_CLI
    esp_openthread_cli_init();
#endif
    
    otCliSetUserCommands(commands, 1, esp_openthread_get_instance());

    esp_netif_t *openthread_netif;
    openthread_netif = init_openthread_netif(&config);
    esp_netif_set_default_netif(openthread_netif);

#if CONFIG_OPENTHREAD_CLI_ESP_EXTENSION
    esp_cli_custom_command_init();
#endif

#if CONFIG_OPENTHREAD_CLI
    esp_openthread_cli_create_task();
#endif
#if CONFIG_OPENTHREAD_AUTO_START
    otOperationalDatasetTlvs dataset;
    otError error = otDatasetGetActiveTlvs(esp_openthread_get_instance(), &dataset);
    ESP_ERROR_CHECK(esp_openthread_auto_start((error == OT_ERROR_NONE) ? &dataset : NULL));
#endif

    esp_openthread_launch_mainloop();

    esp_openthread_netif_glue_deinit();
    esp_netif_destroy(openthread_netif);

    esp_vfs_eventfd_unregister();
    vTaskDelete(NULL);
}


void mqtt_app_start(void)
{
    // A porta 1883 é usada como literal
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://[fd3d:9cdd:1d34:2ff1:b2c5:134f:b87a:637f]:1883",
        .broker.address.port = 1883, 
        .credentials.username = "kelvin",
        .credentials.client_id = "esp32_thread",
        .credentials.authentication.password = "teste",
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

// FUNÇÃO CORRIGIDA PARA REMOVER O CAMPO 'esp_error_code' INEXISTENTE
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client; 
    
    switch (event->event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_CONNECTED: Cliente conectado com sucesso ao broker.");
        is_mqtt_connected = true; 
        esp_mqtt_client_subscribe(client, "/esp32c6/2/led", 0);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG_MQTT, "MQTT_EVENT_DISCONNECTED: Cliente desconectado.");
        is_mqtt_connected = false; 
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_SUBSCRIBED: Tópico subscrito com sucesso. msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_UNSUBSCRIBED: msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_PUBLISHED: Mensagem enviada com sucesso. msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_DATA: Dados recebidos!");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG_MQTT, "MQTT_EVENT_ERROR: Erro detectado no motor MQTT.");
        is_mqtt_connected = false; 
        
        // CORREÇÃO: Usamos o tipo de erro e o errno do socket para diagnóstico
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
             // Erro de transporte (socket)
             ESP_LOGE(TAG_MQTT, "TCP/TLS Error (errno): %d", event->error_handle->esp_transport_sock_errno); 
        } else {
             // Outros erros, logando o tipo de erro e o errno do socket como diagnóstico
             // Este log substitui a linha problemática (203)
             ESP_LOGE(TAG_MQTT, "Outro Erro (Tipo:%d | errno:%d)", 
                      event->error_handle->error_type, 
                      event->error_handle->esp_transport_sock_errno);
        }
        break;

    default:
        ESP_LOGI(TAG_MQTT, "Outro evento: event_id=%d", event->event_id);
        break;
    }
}


void sensor_data_task(void *pvParameters) {
    
    int dps_ret, veml_white_ret, sht_ret, sgp_ret, sgpindex_ret, as7341_ret;
    char payload[512]; 
    otError error = OT_ERROR_NONE; 

    otInstance *instance = esp_openthread_get_instance();
    if (!instance) {
        ESP_LOGE(TAG, "Failed to get OpenThread instance! Is the OT task running?");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Waiting for OpenThread network to form/join...");
    while (otThreadGetDeviceRole(instance) == OT_DEVICE_ROLE_DISABLED ||
           otThreadGetDeviceRole(instance) == OT_DEVICE_ROLE_DETACHED) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "OpenThread network connected! Device role: %d", otThreadGetDeviceRole(instance));
    
    // INICIA O CLIENTE MQTT APÓS O THREAD ESTAR CONECTADO (Executa apenas 1 vez)
    mqtt_app_start(); 

    vTaskDelay(pdMS_TO_TICKS(1000));
    clock_t start_time =0, end_time = 0;
    double cpu_time_used = 0; 

    shtc1_reading_t humid;
    float temp;
    float press;
    uint16_t voc;
    int32_t vocindex;
    uint16_t white_raw;
    as7341_spectral_data_t spectral_data = {0};
    start_time = clock();

    otIp6Address peerAddress;
    otIp6AddressFromString("ff03::1", &peerAddress);
    uint16_t peerPort = 1234;


    otUdpSocket socket;
    
    esp_openthread_lock_acquire(portMAX_DELAY);
    memset(&socket, 0, sizeof(socket));
    error = otUdpOpen(instance, &socket, NULL, NULL);
    esp_openthread_lock_release();

    if (error != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to open UDP socket: %s", otThreadErrorToString(error));
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        
        dps_ret = dps310_read(&temp, &press);
        veml_white_ret = veml7700_read_white(&white_raw);
        sht_ret = shtc1_read_data(&humid);
        sgp_ret = sgp40_measure_compensated(humid.humidity, temp, &voc);
        sgpindex_ret = sgp40_get_voc_index(humid.humidity, temp, &vocindex);
        as7341_ret = as7341_read_all_channels(&spectral_data); 
 
        if (as7341_ret == ESP_OK) {
            ESP_LOGI(TAG, "AS7341 Spectral Data:");
            ESP_LOGI(TAG, "  F1 (415nm): %" PRIu16, spectral_data.f1);
            ESP_LOGI(TAG, "  F2 (445nm): %" PRIu16, spectral_data.f2);
            ESP_LOGI(TAG, "  F3 (480nm): %" PRIu16, spectral_data.f3);
            ESP_LOGI(TAG, "  F4 (515nm): %" PRIu16, spectral_data.f4);
            ESP_LOGI(TAG, "  F5 (555nm): %" PRIu16, spectral_data.f5);
            ESP_LOGI(TAG, "  F6 (590nm): %" PRIu16, spectral_data.f6);
            ESP_LOGI(TAG, "  F7 (630nm): %" PRIu16, spectral_data.f7);
            ESP_LOGI(TAG, "  F8 (680nm): %" PRIu16, spectral_data.f8);
            ESP_LOGI(TAG, "  NIR: %" PRIu16, spectral_data.nir);
            ESP_LOGI(TAG, "  Clear: %" PRIu16, spectral_data.clear);
        } else {
            ESP_LOGE(TAG, "AS7341: Failed to read spectral data! Ret1=%s",
                     esp_err_to_name(as7341_ret));
        }

        if (dps_ret == ESP_OK && sgp_ret == ESP_OK && veml_white_ret == ESP_OK && sht_ret == ESP_OK && sgpindex_ret == ESP_OK) {
            ESP_LOGI(TAG, "Sensor Readings: Temperature=%.2f C, Pressure=%.2f hPa, VOCraw=%" PRIu16 " VOCIndex=%" PRIi32 " Lux=%" PRIu16 " Humidity=%.2f",
            temp, press, voc, vocindex, white_raw, humid.humidity);

            
            int len = snprintf(payload, sizeof(payload),
                    "{\"temp\":%.2f,\"press\":%.2f,\"voc\":%u,\"vocindex\":%lu,"
                    "\"humid\":%.2f,\"lux\":%u}",
                    temp, press, voc, vocindex, humid.humidity, white_raw);
                    
           if (len < 0 || len >= sizeof(payload)) {
                ESP_LOGE("sensor_task", "Error formatting JSON payload!");
            } else {
                // VERIFICA O ESTADO DA CONEXÃO MQTT ANTES DE PUBLICAR
                if (is_mqtt_connected) {
                    // Publica via MQTT usando 'len' (tamanho exato)
                    int msg_id = esp_mqtt_client_publish(mqtt_client, "sensors/data", payload, len, 1, 0); 
                    if (msg_id > 0) {
                        ESP_LOGI("sensor_task", "MQTT message sent, id=%d: %s", msg_id, payload);
                    } else {
                        ESP_LOGE("sensor_task", "MQTT Publish FAILED! (Retorno: %d)", msg_id);
                    }
                } else {
                    ESP_LOGW("sensor_task", "MQTT client not connected. Skipping MQTT publish.");
                }
            }
        
            // Lógica de envio UDP (OpenThread)
            esp_openthread_lock_acquire(portMAX_DELAY);

            otMessage *message = NULL;
            otMessageInfo messageInfo;
            
            memset(&messageInfo, 0, sizeof(messageInfo));
            messageInfo.mPeerAddr = peerAddress;
            messageInfo.mPeerPort = peerPort;

            message = otUdpNewMessage(instance, NULL);
            if (message != NULL) {
                error = otMessageAppend(message, payload, strlen(payload));
                if (error != OT_ERROR_NONE) {
                    ESP_LOGE(TAG, "Failed to append payload to message: %s", otThreadErrorToString(error));
                    otMessageFree(message);
                } else {
                    error = otUdpSend(instance, &socket, message, &messageInfo);
                    if (error != OT_ERROR_NONE) {
                        ESP_LOGE(TAG, "Failed to send UDP message: %s", otThreadErrorToString(error));
                        otMessageFree(message);
                    } else {
                        char ip6_addr_str[OT_IP6_ADDRESS_STRING_SIZE];
                        otIp6AddressToString(&messageInfo.mPeerAddr, ip6_addr_str, sizeof(ip6_addr_str));
                        ESP_LOGI(TAG, "UDP message sent successfully to %s:%u: %s",
                                 ip6_addr_str,
                                 messageInfo.mPeerPort, payload);
                    }
                }
            } else {
                ESP_LOGE(TAG, "Failed to create new UDP message (out of memory?).");
            }
            
            esp_openthread_lock_release();
        } else {
            ESP_LOGE(TAG, "Sensor read failed! DPS=%d, SGP=%d, VEML=%d, SHT=%d, VOCidx=%d, AS7341=%d",
                dps_ret, sgp_ret, veml_white_ret, sht_ret, sgpindex_ret, as7341_ret);
        }
        end_time = clock();
        cpu_time_used = ((double)(end_time - start_time)) / CLOCKS_PER_SEC;
        ESP_LOGI(TAG, "Tempo de execução: %f segundos\n", cpu_time_used);

        vTaskDelay(pdMS_TO_TICKS(20000));
    }
    esp_openthread_lock_acquire(portMAX_DELAY);
    otUdpClose(instance, &socket);
    esp_openthread_lock_release();
    vTaskDelete(NULL);
}

esp_err_t sensor_data_task_init(void) {
    BaseType_t res = xTaskCreate(
        sensor_data_task,
        "sensor_task",
        SENSOR_TASK_STACK_SIZE,
        NULL,
        SENSOR_TASK_PRIORITY,
        NULL
    );

    if (res != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor data task!");
        return ESP_FAIL;
    } else {
        ESP_LOGI(TAG, "Sensor data task created successfully.");
        return ESP_OK;
    }
}