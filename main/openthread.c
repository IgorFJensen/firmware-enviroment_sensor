#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h> // Make sure this is included for PRIu16
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
#include "openthread/tasklet.h"
#include "openthread.h"

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
#endif // CONFIG_OPENTHREAD_CLI_ESP_EXTENSION

// Removed 'extern i2c_master_bus_handle_t bus_handle;' as it's not used directly
// in sensor_data_task's read calls based on your sensor driver signatures.

static const char *TAG = "sensor_task";
// Define the stack size and priority for the sensor task
#define SENSOR_TASK_STACK_SIZE 4096 // Adjust based on complexity of sensor drivers and payload formatting
#define SENSOR_TASK_PRIORITY   4    // Priority for the sensor task

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
    // Unused variables, but required by the function signature
    (void)context;
    (void)argc;
    (void)argv;

    otLogNotePlat("Reiniciando a placa...");
    // Atraso curto para permitir que a mensagem seja enviada ao terminal
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
}

void ot_task_worker(void *aContext)
{
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };

    // Initialize the OpenThread stack
    ESP_ERROR_CHECK(esp_openthread_init(&config));

#if CONFIG_OPENTHREAD_STATE_INDICATOR_ENABLE
    ESP_ERROR_CHECK(esp_openthread_state_indicator_init(esp_openthread_get_instance()));
#endif

#if CONFIG_OPENTHREAD_LOG_LEVEL_DYNAMIC
    // The OpenThread log level directly matches ESP log level
    (void)otLoggingSetLevel(CONFIG_LOG_DEFAULT_LEVEL);
#endif
    // Initialize the OpenThread cli
#if CONFIG_OPENTHREAD_CLI
    esp_openthread_cli_init();
#endif
static const otCliCommand commands[] = {
        {"restart", NULL, ot_cli_restart_command},
};
otCliSetUserCommands(commands, 1, esp_openthread_get_instance());

    esp_netif_t *openthread_netif;
    // Initialize the esp_netif bindings
    openthread_netif = init_openthread_netif(&config);
    esp_netif_set_default_netif(openthread_netif);

#if CONFIG_OPENTHREAD_CLI_ESP_EXTENSION
    esp_cli_custom_command_init();
#endif // CONFIG_OPENTHREAD_CLI_ESP_EXTENSION

    // Run the main loop
#if CONFIG_OPENTHREAD_CLI
    esp_openthread_cli_create_task();
#endif
#if CONFIG_OPENTHREAD_AUTO_START
    otOperationalDatasetTlvs dataset;
    otError error = otDatasetGetActiveTlvs(esp_openthread_get_instance(), &dataset);
    ESP_ERROR_CHECK(esp_openthread_auto_start((error == OT_ERROR_NONE) ? &dataset : NULL));
#endif
otInstance *instance = esp_openthread_get_instance();
    if (instance) {
        // CORREÇÃO: Usar o tipo de dado correto 'otDeviceRole'
        otDeviceRole role = otThreadGetDeviceRole(instance);
        if (role == OT_DEVICE_ROLE_DISABLED || role == OT_DEVICE_ROLE_DETACHED) {
            otLogNotePlat("A rede não está ativa. Forçando o início...");
            otIp6SetEnabled(instance, true);
            otThreadSetEnabled(instance, true);
            otIp6SetEnabled(instance, true);
        }
    }

    esp_openthread_launch_mainloop();

    // Clean up
    esp_openthread_netif_glue_deinit();
    esp_netif_destroy(openthread_netif);

    esp_vfs_eventfd_unregister();
    vTaskDelete(NULL);
}


// Function to read sensor data and send it over OpenThread UDP
void sensor_data_task(void *pvParameters) {
    otInstance *instance = esp_openthread_get_instance();
    if (!instance) {
        ESP_LOGE(TAG, "Failed to get OpenThread instance! Is the OT task running?");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Waiting for OpenThread network to form/join...");
    while (otThreadGetDeviceRole(instance) == OT_DEVICE_ROLE_DISABLED ||
           otThreadGetDeviceRole(instance) == OT_DEVICE_ROLE_DETACHED) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Espera 1 segundo e verifica novamente
    }
    ESP_LOGI(TAG, "OpenThread network connected! Device role: %d", otThreadGetDeviceRole(instance));

    vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    clock_t start_time =0, end_time = 0;
    double cpu_time_used = 0; 

    shtc1_reading_t humid;
    float temp;
    float press;
    uint16_t voc;
    int32_t vocindex;
    uint16_t white_raw;
    as7341_spectral_data_t spectral_data = {0}; // Structure to hold AS7341 data
    start_time = clock();
    // Define the destination address and port for your UDP messages.
    // **IMPORTANT:** Replace "fd00::1" and 1234 with your actual
    // target IPv6 address and UDP port for the Thread network.
    otIp6Address peerAddress;
    otIp6AddressFromString("fd46:ebf2:2db5:1:c565:6059:4ee:e6be", &peerAddress); // Example: A specific node in your Thread network
    uint16_t peerPort = 1234; // Example: Destination UDP port

    //Opens the socket only once
    otUdpSocket socket;
        otError error = OT_ERROR_NONE;

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
        // 1. Read sensor data
        // Make sure your dps310_read and sgp40_read_raw functions accept bus_handle
        // If they use internal handles, adjust this call accordingly.
        esp_err_t dps_ret = dps310_read(&temp, &press);
        esp_err_t veml_white_ret = veml7700_read_white(&white_raw);     // Read raw ALS data
        esp_err_t sht_ret = shtc1_read_data(&humid);
        esp_err_t sgp_ret = sgp40_measure_compensated(humid.humidity, temp, &voc);
        esp_err_t sgpindex_ret = sgp40_get_voc_index(humid.humidity, temp, &vocindex);
        // AS7341 - Requires two reads to get all 8 Fx channels
        esp_err_t as7341_ret = as7341_read_all_channels(&spectral_data); 
 
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

            // 2. Format the data for transmission into a JSON string
        char payload[512]; // Increased buffer size to accommodate all spectral channels
        // int len = snprintf(payload, sizeof(payload),
        //             "{\"temp\":%.2f,\"press\":%.2f,\"voc\":%u,\"vocindex\":%lu,"
        //             "\"humid\":%.2f,\"lux\":%u,\"f1\":%u,\"f2\":%u,\"f3\":%u,"
        //             "\"f4\":%u,\"f5\":%u,\"f6\":%u,\"f7\":%u,"
        //             "\"f8\":%u,\"nir\":%u,\"clear\":%u}",
        //             temp, press, voc, vocindex, humid.humidity, white_raw,
        //             spectral_data.f1, spectral_data.f2, spectral_data.f3, spectral_data.f4,
        //             spectral_data.f5, spectral_data.f6, spectral_data.f7, spectral_data.f8,
        //             spectral_data.nir, spectral_data.clear);
        int len = snprintf(payload, sizeof(payload),
                    "{\"temp\":%.2f,\"press\":%.2f,\"voc\":%u,\"vocindex\":%lu,"
                    "\"humid\":%.2f,\"lux\":%u}",
                    temp, press, voc, vocindex, humid.humidity, white_raw);
                    
            if (len >= sizeof(payload) || len < 0) {
                ESP_LOGE(TAG, "Payload buffer too small or error formatting data! (len=%d, max=%d)", len, sizeof(payload));
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }

            // 3. Send the data using OpenThread UDP APIs
            // Acquire the OpenThread lock before interacting with the OT instance
            esp_openthread_lock_acquire(portMAX_DELAY);

            otMessage *message = NULL;
            otMessageInfo messageInfo;
            
            memset(&messageInfo, 0, sizeof(messageInfo));

            // Prepare message info with destination address and port
            messageInfo.mPeerAddr = peerAddress;
            messageInfo.mPeerPort = peerPort;

            // Create a new OpenThread message
            message = otUdpNewMessage(instance, NULL);
            if (message != NULL) {
                // Append the payload data to the message
                error = otMessageAppend(message, payload, strlen(payload));
                if (error != OT_ERROR_NONE) {
                    ESP_LOGE(TAG, "Failed to append payload to message: %s", otThreadErrorToString(error));
                    otMessageFree(message);
                } else {
                    // Send the UDP message
                    error = otUdpSend(instance, &socket, message, &messageInfo);
                    if (error != OT_ERROR_NONE) {
                        ESP_LOGE(TAG, "Failed to send UDP message: %s", otThreadErrorToString(error));
                        otMessageFree(message);
                    } else {
                        // Buffer to hold the string representation of the IPv6 address
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
            
            // Release the OpenThread lock
            esp_openthread_lock_release();
        } else {
            // Log error if sensor readings failed
            ESP_LOGE(TAG, "Failed to read one or more sensors. DPS310_ret=%d, SGP40_ret=%d", dps_ret, sgp_ret);
        }
        end_time = clock();
        cpu_time_used = ((double)(end_time - start_time)) / CLOCKS_PER_SEC;
        ESP_LOGI(TAG, "Tempo de execução: %f segundos\n", cpu_time_used);

        // 5. Wait before the next reading/sending cycle
        vTaskDelay(pdMS_TO_TICKS(4000)); // Read and send every 10 seconds
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
        NULL, // No specific parameters passed to the task function here
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