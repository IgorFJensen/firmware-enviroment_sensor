#include "mqtt_handler.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "mqtt_client.h"

static const char *TAG = "MQTT_HANDLER";

#define MQTT_BROKER_URI  "mqtt://[fd6b:f925:aaf4:3710:2b7a:c448:24b3:235c]:1883"
#define MQTT_USERNAME    "kelvin"
#define MQTT_PASSWORD    "teste"
#define MQTT_CLIENT_ID   "esp32_thread"
#define MQTT_STATE_TOPIC "sensors/data"
#define MQTT_ALERT_TOPIC "sensors/alert"

#define MQTT_CONNECTED_BIT     BIT0
#define MQTT_PUBLISHED_BIT     BIT1
#define MQTT_DISCONNECTED_BIT  BIT2

static esp_mqtt_client_handle_t s_mqtt_client = NULL;
static EventGroupHandle_t s_mqtt_events = NULL;
/* Uma publicacao por vez: os bits de confirmacao sao compartilhados. */
static SemaphoreHandle_t s_publish_mutex = NULL;
static volatile bool s_mqtt_connected = false;
static bool s_mqtt_started = false;

static void mqtt_event_handler(
    void *handler_args,
    esp_event_base_t base,
    int32_t event_id,
    void *event_data)
{
    (void)handler_args;
    (void)base;
    (void)event_id;

    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    if (event == NULL) {
        return;
    }

    switch (event->event_id) {
    case MQTT_EVENT_CONNECTED:
        s_mqtt_connected = true;
        if (s_mqtt_events != NULL) {
            xEventGroupClearBits(s_mqtt_events, MQTT_DISCONNECTED_BIT);
            xEventGroupSetBits(s_mqtt_events, MQTT_CONNECTED_BIT);
        }
        ESP_LOGI(TAG, "Conectado ao Broker MQTT");
        break;

    case MQTT_EVENT_DISCONNECTED:
        s_mqtt_connected = false;
        if (s_mqtt_events != NULL) {
            xEventGroupClearBits(s_mqtt_events, MQTT_CONNECTED_BIT);
            xEventGroupSetBits(s_mqtt_events, MQTT_DISCONNECTED_BIT);
        }
        ESP_LOGW(TAG, "Desconectado do Broker MQTT");
        break;

    case MQTT_EVENT_PUBLISHED:
        if (s_mqtt_events != NULL) {
            xEventGroupSetBits(s_mqtt_events, MQTT_PUBLISHED_BIT);
        }
        ESP_LOGI(TAG, "Publicacao confirmada pelo broker: msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_ERROR:
        s_mqtt_connected = false;
        if (s_mqtt_events != NULL) {
            xEventGroupClearBits(s_mqtt_events, MQTT_CONNECTED_BIT);
            xEventGroupSetBits(s_mqtt_events, MQTT_DISCONNECTED_BIT);
        }
        ESP_LOGE(TAG, "Erro no MQTT");
        if (event->error_handle != NULL &&
            event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGE(TAG, "Erro TCP/TLS (errno): %d",
                     event->error_handle->esp_transport_sock_errno);
        }
        break;

    default:
        break;
    }
}

void mqtt_app_start(void)
{
    if (s_mqtt_started) {
        ESP_LOGD(TAG, "Cliente MQTT ja foi iniciado");
        return;
    }

    if (s_mqtt_events == NULL) {
        s_mqtt_events = xEventGroupCreate();
        if (s_mqtt_events == NULL) {
            ESP_LOGE(TAG, "Sem memoria para eventos MQTT");
            return;
        }
    }

    if (s_publish_mutex == NULL) {
        s_publish_mutex = xSemaphoreCreateMutex();
        if (s_publish_mutex == NULL) {
            ESP_LOGE(TAG, "Sem memoria para mutex de publicacao MQTT");
            return;
        }
    }

    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        .credentials.username = MQTT_USERNAME,
        .credentials.client_id = MQTT_CLIENT_ID,
        .credentials.authentication.password = MQTT_PASSWORD,
        .session.keepalive = 60,
    };

    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (s_mqtt_client == NULL) {
        ESP_LOGE(TAG, "Falha criando cliente MQTT");
        return;
    }

    esp_err_t err = esp_mqtt_client_register_event(
        s_mqtt_client,
        ESP_EVENT_ANY_ID,
        mqtt_event_handler,
        NULL);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha registrando eventos MQTT: %s", esp_err_to_name(err));
        esp_mqtt_client_destroy(s_mqtt_client);
        s_mqtt_client = NULL;
        return;
    }

    err = esp_mqtt_client_start(s_mqtt_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha iniciando MQTT: %s", esp_err_to_name(err));
        esp_mqtt_client_destroy(s_mqtt_client);
        s_mqtt_client = NULL;
        return;
    }

    s_mqtt_started = true;
    ESP_LOGI(TAG, "Cliente iniciado: %s", MQTT_BROKER_URI);
}

bool mqtt_is_connected(void)
{
    return s_mqtt_connected;
}

static esp_err_t publish_and_wait(
    const char *topic,
    const char *payload,
    size_t len,
    uint32_t timeout_ms,
    int *out_msg_id)
{
    xEventGroupClearBits(
        s_mqtt_events,
        MQTT_PUBLISHED_BIT | MQTT_DISCONNECTED_BIT);

    const int msg_id = esp_mqtt_client_publish(
        s_mqtt_client,
        topic,
        payload,
        (int)len,
        1,
        1);

    if (out_msg_id != NULL) {
        *out_msg_id = msg_id;
    }

    if (msg_id < 0) {
        ESP_LOGE(TAG, "Falha enfileirando publicacao em %s", topic);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG,
             "Aguardando confirmacao: topico=%s msg_id=%d bytes=%u",
             topic,
             msg_id,
             (unsigned)len);

    const EventBits_t bits = xEventGroupWaitBits(
        s_mqtt_events,
        MQTT_PUBLISHED_BIT | MQTT_DISCONNECTED_BIT,
        pdTRUE,
        pdFALSE,
        pdMS_TO_TICKS(timeout_ms));

    if ((bits & MQTT_PUBLISHED_BIT) != 0U) {
        return ESP_OK;
    }

    if ((bits & MQTT_DISCONNECTED_BIT) != 0U) {
        ESP_LOGW(TAG, "MQTT desconectou durante a publicacao");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGW(TAG,
             "Timeout aguardando confirmacao MQTT: msg_id=%d timeout=%lu ms",
             msg_id,
             (unsigned long)timeout_ms);
    return ESP_ERR_TIMEOUT;
}

static esp_err_t publish_serialized(
    const char *topic,
    const char *payload,
    size_t len,
    uint32_t timeout_ms,
    int *out_msg_id)
{
    if (payload == NULL || len == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_mqtt_connected || s_mqtt_client == NULL ||
        s_mqtt_events == NULL || s_publish_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Espera, no maximo, uma publicacao em andamento terminar. */
    if (xSemaphoreTake(s_publish_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        ESP_LOGW(TAG, "Outra publicacao ocupou o MQTT por mais de %lu ms",
                 (unsigned long)timeout_ms);
        return ESP_ERR_TIMEOUT;
    }

    const esp_err_t err =
        publish_and_wait(topic, payload, len, timeout_ms, out_msg_id);

    xSemaphoreGive(s_publish_mutex);
    return err;
}

esp_err_t mqtt_publish_sensor_data(
    const char *payload,
    size_t len,
    uint32_t timeout_ms,
    int *out_msg_id)
{
    return publish_serialized(
        MQTT_STATE_TOPIC, payload, len, timeout_ms, out_msg_id);
}

esp_err_t mqtt_publish_alert(
    const char *payload,
    size_t len,
    uint32_t timeout_ms,
    int *out_msg_id)
{
    return publish_serialized(
        MQTT_ALERT_TOPIC, payload, len, timeout_ms, out_msg_id);
}
