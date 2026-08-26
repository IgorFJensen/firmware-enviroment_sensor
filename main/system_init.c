#include "system_init.h"
#include "esp_openthread.h"
#include "esp_openthread_cli.h"
#include "esp_openthread_netif_glue.h"
#include "esp_openthread_types.h"
#include "openthread/cli.h"
#include "openthread/instance.h"
#include "openthread/logging.h"
#include "openthread/tasklet.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_vfs_eventfd.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_sleep.h"
#include "mqtt_handler.h"

#if CONFIG_OPENTHREAD_CLI_ESP_EXTENSION
#include "esp_ot_cli_extension.h"
#endif

static esp_netif_t *init_openthread_netif(const esp_openthread_platform_config_t *config)
{
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_OPENTHREAD();
    esp_netif_t *netif = esp_netif_new(&cfg);
    assert(netif != NULL);
    ESP_ERROR_CHECK(esp_netif_attach(netif, esp_openthread_netif_glue_init(config)));
    return netif;
}
// Assinatura corrigida: Retorna otError e argc é uint8_t
otError ot_cli_restart_command(void *context, uint8_t argc, char *argv[])
{
    (void)context; 
    (void)argc; 
    (void)argv;

    ESP_LOGI("OT_CLI", "Reiniciando sistema..."); 
    
    // Pequeno delay para garantir que o log saia antes do reset
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();

    return OT_ERROR_NONE; // Retorno obrigatório
}

// O array agora aceitará a função sem erros
static const otCliCommand commands[] = {
    {"restart", ot_cli_restart_command},
};

static void openthread_state_changed_cb(otChangedFlags flags, void *ctx)
{
    // Verifica se houve mudança no papel (role) do dispositivo na rede Thread
    if (flags & OT_CHANGED_THREAD_ROLE) {
        otDeviceRole role = otThreadGetDeviceRole(esp_openthread_get_instance());
        ESP_LOGI("OT_STATE", "Mudança de estado Thread. Novo papel: %d", role);
        
        // Se conectou à rede com sucesso
        if (role == OT_DEVICE_ROLE_CHILD || role == OT_DEVICE_ROLE_ROUTER || role == OT_DEVICE_ROLE_LEADER) {
            ESP_LOGI("OT_STATE", "Anexado à rede Thread! Iniciando MQTT...");
            
            // Inicia o MQTT apenas se ainda não estiver conectado
            if (!mqtt_is_connected()) {
                mqtt_app_start();
            }
        }
    }
}

void ot_task_worker(void *aContext)
{
    // Agora o compilador reconhece essas macros porque estão no system_init.h
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };

    ESP_ERROR_CHECK(esp_openthread_init(&config));

#if CONFIG_OPENTHREAD_LOG_LEVEL_DYNAMIC
    (void)otLoggingSetLevel(CONFIG_LOG_DEFAULT_LEVEL);
#endif

#if CONFIG_OPENTHREAD_CLI
    esp_openthread_cli_init();
#endif
    
    otCliSetUserCommands(commands, 1, esp_openthread_get_instance());

    esp_netif_t *openthread_netif = init_openthread_netif(&config);
    esp_netif_set_default_netif(openthread_netif);

#if CONFIG_OPENTHREAD_CLI_ESP_EXTENSION
    esp_cli_custom_command_init();
#endif

#if CONFIG_OPENTHREAD_CLI
    esp_openthread_cli_create_task();
#endif
    otOperationalDatasetTlvs dataset;
    bool dataset_ready = false;

    // 1. First attempt: Try to load the dataset saved in NVS (persisted before deep-sleep)
    if (load_openthread_dataset(&dataset) == ESP_OK) {
        ESP_LOGI("OT_INIT", "Dataset loaded from NVS, restoring Thread network...");
        dataset_ready = true;
    } 
    // 2. Second attempt: If not in NVS, try to fetch it from the internal stack
    else {
        otError error = otDatasetGetActiveTlvs(esp_openthread_get_instance(), &dataset);
        if (error == OT_ERROR_NONE) {
            ESP_LOGI("OT_INIT", "Active dataset found in stack, restoring Thread network...");
            dataset_ready = true;
        } 
        // 3. FALLBACK: If everything else fails, inject your hardcoded factory TLV
        else {
            ESP_LOGW("OT_INIT", "No dataset in NVS or stack. Injecting default factory TLV...");
            
            const uint8_t factory_tlv[] = {
                0x05, 0x10, 0xbe, 0x3f, 0xcf, 0x43, 0xe8, 0x40, 0x2b, 0x39, 0xf1, 0xf7, 0x3d, 0x8a, 0xd6, 0x9b,
                0x84, 0x36, 0x03, 0x10, 0x41, 0x4d, 0x5a, 0x4e, 0x2d, 0x54, 0x68, 0x72, 0x65, 0x61, 0x6d, 0x2d,
                0x32, 0x36, 0x36, 0x30, 0x02, 0x08, 0x6b, 0xf9, 0x25, 0xaa, 0xf4, 0x62, 0x37, 0x10, 0x01, 0x02,
                0x26, 0x60, 0x04, 0x10, 0x5d, 0xfd, 0x5a, 0x31, 0xc3, 0x04, 0xee, 0xc4, 0xe6, 0x03, 0x41, 0xdb,
                0x8b, 0xd0, 0x14, 0x04, 0x35, 0x06, 0x00, 0x04, 0x00, 0x00, 0x08, 0x00, 0x00, 0x03, 0x00, 0x00,
                0x14, 0x0c, 0x04, 0x02, 0xa0, 0xff, 0xf8, 0x0e, 0x08, 0x00, 0x00, 0x00, 0x00, 0x69, 0xf1, 0x1b,
                0xce
            };

            dataset.mLength = sizeof(factory_tlv);
            memcpy(dataset.mTlvs, factory_tlv, sizeof(factory_tlv));

            // Apply the default dataset to the OpenThread instance
            if (otDatasetSetActiveTlvs(esp_openthread_get_instance(), &dataset) == OT_ERROR_NONE) {
                ESP_LOGI("OT_INIT", "Default factory TLV set successfully!");
                dataset_ready = true;
                
                // Optional: Save immediately to NVS so the next boot/deep-sleep wake is faster
                save_openthread_dataset(); 
            } else {
                ESP_LOGE("OT_INIT", "Critical failure while setting factory TLV.");
            }
        }
    }

    // If a valid dataset was acquired from any of the sources, start the network
    if (dataset_ready) {
        ESP_ERROR_CHECK(esp_openthread_auto_start(&dataset));
        ESP_LOGI("OT_INIT", "Thread interface automatically enabled and started.");
    }

    otSetStateChangedCallback(esp_openthread_get_instance(), openthread_state_changed_cb, NULL);
    esp_openthread_launch_mainloop();

    esp_openthread_netif_glue_deinit();
    esp_netif_destroy(openthread_netif);
    esp_vfs_eventfd_unregister();
    vTaskDelete(NULL);
}

// Persist the operational dataset TLVs into NVS so they can be restored after deep-sleep
esp_err_t save_openthread_dataset(void)
{
    otOperationalDatasetTlvs dataset;
    otError err = otDatasetGetActiveTlvs(esp_openthread_get_instance(), &dataset);
    if (err != OT_ERROR_NONE) {
        ESP_LOGW("OT_PERSIST", "Nenhum dataset ativo para salvar (%d)", err);
        return ESP_FAIL;
    }

    nvs_handle_t h;
    esp_err_t res = nvs_open("ot_ds", NVS_READWRITE, &h);
    if (res != ESP_OK) return res;

    // Salva o comprimento e os bytes TLV
    uint32_t len = dataset.mLength;
    res = nvs_set_u32(h, "len", len);
    if (res != ESP_OK) { nvs_close(h); return res; }

    res = nvs_set_blob(h, "tlvs", dataset.mTlvs, len);
    if (res != ESP_OK) { nvs_close(h); return res; }

    res = nvs_commit(h);
    nvs_close(h);
    if (res == ESP_OK) ESP_LOGI("OT_PERSIST", "Dataset salvo (%u bytes)", (unsigned)len);
    return res;
}

// Carrega dataset da NVS para o buffer fornecido
esp_err_t load_openthread_dataset(otOperationalDatasetTlvs *dataset)
{
    if (!dataset) return ESP_ERR_INVALID_ARG;
    nvs_handle_t h;
    esp_err_t res = nvs_open("ot_ds", NVS_READONLY, &h);
    if (res != ESP_OK) return res;

    uint32_t len = 0;
    res = nvs_get_u32(h, "len", &len);
    if (res != ESP_OK) { nvs_close(h); return res; }

    size_t required = len;
    if (required == 0 || required > sizeof(dataset->mTlvs)) {
        nvs_close(h);
        return ESP_ERR_INVALID_SIZE;
    }

    res = nvs_get_blob(h, "tlvs", dataset->mTlvs, &required);
    if (res == ESP_OK) {
        dataset->mLength = (uint8_t)required;
    }
    nvs_close(h);
    if (res == ESP_OK) ESP_LOGI("OT_PERSIST", "Dataset carregado (%u bytes)", (unsigned)dataset->mLength);
    return res;
}