#include "mqtt_handler.h"
#include "esp_log.h"
#include "esp_netif.h" 
static const char *TAG_MQTT = "MQTT_HANDLER";

static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool is_mqtt_connected = false;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    
    switch (event->event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG_MQTT, "Conectado ao Broker MQTT");
        is_mqtt_connected = true;
        // Exemplo de subscribe, se necessário
        // esp_mqtt_client_subscribe(event->client, "/esp32c6/comandos", 0);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG_MQTT, "Desconectado do Broker MQTT");
        is_mqtt_connected = false;
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG_MQTT, "Erro no MQTT");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
             ESP_LOGE(TAG_MQTT, "Erro TCP/TLS (errno): %d", event->error_handle->esp_transport_sock_errno);
        }
        is_mqtt_connected = false;
        break;

    default:
        break;
    }
}
/*
void mqtt_app_start(void)
{
    if (mqtt_client != NULL) return;
    
    // Configuração OBRIGATÓRIA para IPv6 Literal sem getaddrinfo()
    esp_mqtt_client_config_t mqtt_cfg = {
        // 1. REMOVA/COMENTE o campo .uri
        // .broker.address.uri = "mqtt://kelvin:teste@[fd3d:9cdd:1d34:2ff1:b2c5:134f:b87a:637f]:1884", 
        
        // 2. USE hostname e port separados, e configure as credenciais
        .broker.address.hostname = "[fd3d:9cdd:1d34:2ff1:b2c5:134f:b87a:637f]", // Endereço IPv6 puro
        .broker.address.port = 1884,                                          // Porta
        .broker.address.transport = MQTT_TRANSPORT_OVER_TCP,
        .credentials.username = "kelvin",
        .credentials.authentication.password = "teste",
        .credentials.client_id = "esp32_c6_sensor",

        // ... o restante da sua configuração permanece igual
        .broker.verification.skip_cert_common_name_check = true,
        .network.reconnect_timeout_ms = 10000,
        .network.disable_auto_reconnect = false,
    };
    
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG_MQTT, "FALHA: O ESP-IDF nao aceitou a URI.");
        return;
    }

    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}
*/
void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
 //       .broker.address.path = 
        .broker.address.uri = "mqtt://[fd3d:9cdd:1d34:2ff1:b2c5:134f:b87a:637f]:1883",//"[SERVER_PORT]", // IPv6 entre colchetes
        .broker.address.port = 1883,                  // Utiliza a porta definida na macro
        .credentials.username = "kelvin",
        .credentials.client_id = "esp32_thread",
        .credentials.authentication.password = "teste",
 //       .broker.address.transport = MQTT_TRANSPORT_OVER_SSL, // Configura o transporte
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

bool mqtt_is_connected(void)
{
    return is_mqtt_connected;
}

int mqtt_publish_sensor_data(const char *payload, int len)
{
    if (!is_mqtt_connected || mqtt_client == NULL) return -1;
    return esp_mqtt_client_publish(mqtt_client, "sensors/data", payload, len, 1, 0);
}