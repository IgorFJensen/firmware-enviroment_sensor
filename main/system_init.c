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
    otError error = otDatasetGetActiveTlvs(esp_openthread_get_instance(), &dataset);
    
    // CORREÇÃO: Só tenta iniciar automaticamente se leu o dataset com sucesso
    if (error == OT_ERROR_NONE) {
        ESP_LOGI("OT_INIT", "Dataset encontrado, restaurando rede Thread...");
        ESP_ERROR_CHECK(esp_openthread_auto_start(&dataset));
    } else {
        // Se não tem dataset, não faz nada e deixa o dispositivo ligado esperando comandos CLI ou Joiner
        ESP_LOGW("OT_INIT", "Nenhum dataset salvo na NVS. O dispositivo aguardará configuração via CLI ou Joiner.");
    }
#endif

    esp_openthread_launch_mainloop();

    esp_openthread_netif_glue_deinit();
    esp_netif_destroy(openthread_netif);
    esp_vfs_eventfd_unregister();
    vTaskDelete(NULL);
}