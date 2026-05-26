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

#if CONFIG_OPENTHREAD_AUTO_START
    otOperationalDatasetTlvs dataset;
    // Primeiro tenta carregar dataset salvo na NVS (persistido antes do deep-sleep)
    if (load_openthread_dataset(&dataset) == ESP_OK) {
        ESP_LOGI("OT_INIT", "Dataset carregado da NVS, restaurando rede Thread...");
        ESP_ERROR_CHECK(esp_openthread_auto_start(&dataset));
    } else {
        // Caso não exista dataset na NVS, tenta obter do próprio stack
        otError error = otDatasetGetActiveTlvs(esp_openthread_get_instance(), &dataset);
        if (error == OT_ERROR_NONE) {
            ESP_LOGI("OT_INIT", "Dataset ativo encontrado no stack, restaurando rede Thread...");
            ESP_ERROR_CHECK(esp_openthread_auto_start(&dataset));
        } else {
            ESP_LOGW("OT_INIT", "Nenhum dataset disponível. O dispositivo aguardará configuração via CLI ou Joiner.");
        }
    }
#endif

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