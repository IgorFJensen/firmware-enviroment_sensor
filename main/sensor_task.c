/* main/sensor_task.c */
#include "sensor_task.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_openthread.h"
#include "openthread/thread.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

// Drivers
#include "dps310.h"
#include "sgp40.h"
#include "veml7700.h"
#include "as7341.h"
#include "sht40.h"

#include "mqtt_handler.h"
#include "stats_utils.h"
#include "esp_sleep.h"

static const char *TAG = "SENSOR_TASK";

#define SENSOR_TASK_STACK_SIZE 4096
#define SENSOR_TASK_PRIORITY   4
#define SENSOR_PWR_PIN         19  

static void sensor_loop_task(void *pvParameters)
{
    i2c_master_bus_handle_t bus_handle = (i2c_master_bus_handle_t)pvParameters;
    
    // Buffer aumentado para comportar todos os dados espectrais no JSON
    char payload[768];

    gpio_reset_pin(SENSOR_PWR_PIN);
    gpio_set_direction(SENSOR_PWR_PIN, GPIO_MODE_OUTPUT);

    const int SAMPLE_INTERVAL_MS = 500;      
    const int BURST_DURATION_MS = 10000;     
    const int BURST_SAMPLES = BURST_DURATION_MS / SAMPLE_INTERVAL_MS; 
    const int BASE_SLEEP_MS = 10000;         
    const int BASE_REPEAT = 15;              
    const int CIRCULAR_SIZE = 10;            

    const float INVALID_F = -9999.0f;        
    const float VAR_TEMP_THRESHOLD = 4.0f;   
    const float VAR_HUMID_THRESHOLD = 100.0f; 

    sht40_reading_t humid;
    float temp = 0.0f, press = 0.0f;
    uint16_t voc = 0, white_raw = 0;
    int32_t vocindex = 0;
    as7341_spectral_data_t spectral_data = {0};

    typedef struct {
        float temp;
        float humid;
        float press;
        float lux;
        float f1, f2, f3, f4, f5, f6, f7, f8, clear, nir;
        uint32_t ts;
    } longa_t;

    static longa_t longa_buffer[10];
    static int buffer_head = 0;
    static int buffer_count = 0;

    while (1) {
        float longa_temp_sum = 0.0f, longa_hum_sum = 0.0f, longa_press_sum = 0.0f;
        float longa_lux_sum = 0.0f;
        
        // Acumuladores de todos os canais espectrais
        float longa_f1_sum = 0.0f, longa_f2_sum = 0.0f, longa_f3_sum = 0.0f, longa_f4_sum = 0.0f;
        float longa_f5_sum = 0.0f, longa_f6_sum = 0.0f, longa_f7_sum = 0.0f, longa_f8_sum = 0.0f;
        float longa_clear_sum = 0.0f, longa_nir_sum = 0.0f;

        printf("\n\n");
        printf("╔════════════════════════════════════════════════════════════╗\n");
        printf("║       INICIANDO NOVO CICLO LONGO (LONGA - 5 MINUTOS)       ║\n");
        printf("╚════════════════════════════════════════════════════════════╝\n");

        for (int base_i = 0; base_i < BASE_REPEAT; ++base_i) {
            
            printf("\n> Ligando VCC (GPIO %d) e inicializando I2C...\n", SENSOR_PWR_PIN);
            gpio_set_level(SENSOR_PWR_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(1000)); 

            dps310_init(bus_handle);
            sgp40_init(bus_handle);
            veml7700_init(bus_handle);
            as7341_init(bus_handle);
            sht40_init(bus_handle);

            // DUMMY READ: Leitura de descarte
            vTaskDelay(pdMS_TO_TICKS(100));
            dps310_read(&temp, &press);
            sht40_read_data(&humid);
            veml7700_read_white(&white_raw);
            vTaskDelay(pdMS_TO_TICKS(100));

            printf("\n--- BURST CURTO %02d/%02d (10s de coleta) ---\n", base_i+1, BASE_REPEAT);
            printf(" Amo | T(°C) | U(%%)  | P(Pa)    | Lux | VOC | F1-F4           | F5-F8           | CLR / NIR \n");
            printf("-----+-------+-------+----------+-----+-----+-----------------+-----------------+-----------\n");
            
            float burst_temps[BURST_SAMPLES], burst_humids[BURST_SAMPLES], burst_press[BURST_SAMPLES];
            uint16_t burst_lux[BURST_SAMPLES];
            
            // Arrays para todos os canais do AS7341
            float burst_f1[BURST_SAMPLES], burst_f2[BURST_SAMPLES], burst_f3[BURST_SAMPLES], burst_f4[BURST_SAMPLES];
            float burst_f5[BURST_SAMPLES], burst_f6[BURST_SAMPLES], burst_f7[BURST_SAMPLES], burst_f8[BURST_SAMPLES];
            float burst_clear[BURST_SAMPLES], burst_nir[BURST_SAMPLES];

            for (int s = 0; s < BURST_SAMPLES; ++s) {
                int dps_ret = dps310_read(&temp, &press);
                int veml_ret = veml7700_read_white(&white_raw);
                int sht_ret = sht40_read_data(&humid);
                int as_ret = as7341_read_all_channels(&spectral_data);
                
                sgp40_measure_compensated(humid.humidity, temp, &voc);
                sgp40_get_voc_index(humid.humidity, temp, &vocindex);

                burst_temps[s] = (dps_ret == ESP_OK) ? temp : INVALID_F;
                burst_humids[s] = (sht_ret == ESP_OK) ? humid.humidity : INVALID_F;
                burst_press[s] = (dps_ret == ESP_OK) ? press : INVALID_F;
                burst_lux[s] = (veml_ret == ESP_OK) ? white_raw : 0;
                
                // Salvando todos os canais
                burst_f1[s] = (as_ret == ESP_OK) ? (float)spectral_data.f1 : 0.0f;
                burst_f2[s] = (as_ret == ESP_OK) ? (float)spectral_data.f2 : 0.0f;
                burst_f3[s] = (as_ret == ESP_OK) ? (float)spectral_data.f3 : 0.0f;
                burst_f4[s] = (as_ret == ESP_OK) ? (float)spectral_data.f4 : 0.0f;
                burst_f5[s] = (as_ret == ESP_OK) ? (float)spectral_data.f5 : 0.0f;
                burst_f6[s] = (as_ret == ESP_OK) ? (float)spectral_data.f6 : 0.0f;
                burst_f7[s] = (as_ret == ESP_OK) ? (float)spectral_data.f7 : 0.0f;
                burst_f8[s] = (as_ret == ESP_OK) ? (float)spectral_data.f8 : 0.0f;
                burst_clear[s] = (as_ret == ESP_OK) ? (float)spectral_data.clear : 0.0f;
                burst_nir[s] = (as_ret == ESP_OK) ? (float)spectral_data.nir : 0.0f;

                printf(" %02d  | %5.2f | %5.2f | %8.2f | %3u | %3" PRIi32 " | %.0f %.0f %.0f %.0f | %.0f %.0f %.0f %.0f | %.0f / %.0f\n", 
                       s+1, burst_temps[s], burst_humids[s], burst_press[s], burst_lux[s], vocindex,
                       burst_f1[s], burst_f2[s], burst_f3[s], burst_f4[s], 
                       burst_f5[s], burst_f6[s], burst_f7[s], burst_f8[s], 
                       burst_clear[s], burst_nir[s]);

                vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
            }

            float burst_temp_mean = stats_mean_f(burst_temps, BURST_SAMPLES, INVALID_F);
            float burst_temp_var  = stats_variance_f(burst_temps, BURST_SAMPLES, INVALID_F);
            float burst_temp_std  = sqrtf(burst_temp_var);

            float burst_hum_mean  = stats_mean_f(burst_humids, BURST_SAMPLES, INVALID_F);
            float burst_hum_var   = stats_variance_f(burst_humids, BURST_SAMPLES, INVALID_F);
            float burst_hum_std   = sqrtf(burst_hum_var);

            float burst_press_mean = stats_mean_f(burst_press, BURST_SAMPLES, INVALID_F);
            float burst_lux_mean = stats_mean_u16(burst_lux, BURST_SAMPLES, 0);
            
            // Médias dos canais espectrais
            float burst_f1_mean = stats_mean_f(burst_f1, BURST_SAMPLES, 0.0f);
            float burst_f2_mean = stats_mean_f(burst_f2, BURST_SAMPLES, 0.0f);
            float burst_f3_mean = stats_mean_f(burst_f3, BURST_SAMPLES, 0.0f);
            float burst_f4_mean = stats_mean_f(burst_f4, BURST_SAMPLES, 0.0f);
            float burst_f5_mean = stats_mean_f(burst_f5, BURST_SAMPLES, 0.0f);
            float burst_f6_mean = stats_mean_f(burst_f6, BURST_SAMPLES, 0.0f);
            float burst_f7_mean = stats_mean_f(burst_f7, BURST_SAMPLES, 0.0f);
            float burst_f8_mean = stats_mean_f(burst_f8, BURST_SAMPLES, 0.0f);
            float burst_clear_mean = stats_mean_f(burst_clear, BURST_SAMPLES, 0.0f);
            float burst_nir_mean = stats_mean_f(burst_nir, BURST_SAMPLES, 0.0f);

            printf("-----+-------+-------+----------+-----+-----+-----------------+-----------------+-----------\n");
            printf("[MED]| %5.2f | %5.2f | %8.2f | %3.0f |  -  | %.0f %.0f %.0f %.0f | %.0f %.0f %.0f %.0f | %.0f / %.0f\n", 
                    burst_temp_mean, burst_hum_mean, burst_press_mean, burst_lux_mean,
                    burst_f1_mean, burst_f2_mean, burst_f3_mean, burst_f4_mean,
                    burst_f5_mean, burst_f6_mean, burst_f7_mean, burst_f8_mean,
                    burst_clear_mean, burst_nir_mean);

            printf("\n>>> ESTATISTICAS DO BURST %d <<<\n", base_i+1);
            printf("    Temp -> Media: %5.2f °C | Variancia: %5.4f | Desvio Padrao: +-%5.4f °C\n", burst_temp_mean, burst_temp_var, burst_temp_std);
            printf("    Umid -> Media: %5.2f %%  | Variancia: %5.4f | Desvio Padrao: +-%5.4f %%\n", burst_hum_mean, burst_hum_var, burst_hum_std);

            longa_temp_sum += burst_temp_mean;
            longa_hum_sum += burst_hum_mean;
            longa_press_sum += burst_press_mean;
            longa_lux_sum += burst_lux_mean;
            longa_f1_sum += burst_f1_mean;
            longa_f2_sum += burst_f2_mean;
            longa_f3_sum += burst_f3_mean;
            longa_f4_sum += burst_f4_mean;
            longa_f5_sum += burst_f5_mean;
            longa_f6_sum += burst_f6_mean;
            longa_f7_sum += burst_f7_mean;
            longa_f8_sum += burst_f8_mean;
            longa_clear_sum += burst_clear_mean;
            longa_nir_sum += burst_nir_mean;

            printf("\n> Desligando VCC e dormindo por %d segundos...\n", BASE_SLEEP_MS / 1000);
            as7341_deinit();
            gpio_set_level(SENSOR_PWR_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(50));

            esp_err_t t_res = esp_sleep_enable_timer_wakeup(((uint64_t)BASE_SLEEP_MS) * 1000ULL);
            esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

            if (t_res == ESP_OK) {
                esp_light_sleep_start();
            } else {
                vTaskDelay(pdMS_TO_TICKS(BASE_SLEEP_MS));
            }
        } 

        // --- CÁLCULOS DA LONGA ---
        longa_t new_longa;
        new_longa.temp = longa_temp_sum / (float)BASE_REPEAT;
        new_longa.humid = longa_hum_sum / (float)BASE_REPEAT;
        new_longa.press = longa_press_sum / (float)BASE_REPEAT;
        new_longa.lux = longa_lux_sum / (float)BASE_REPEAT;
        new_longa.f1 = longa_f1_sum / (float)BASE_REPEAT;
        new_longa.f2 = longa_f2_sum / (float)BASE_REPEAT;
        new_longa.f3 = longa_f3_sum / (float)BASE_REPEAT;
        new_longa.f4 = longa_f4_sum / (float)BASE_REPEAT;
        new_longa.f5 = longa_f5_sum / (float)BASE_REPEAT;
        new_longa.f6 = longa_f6_sum / (float)BASE_REPEAT;
        new_longa.f7 = longa_f7_sum / (float)BASE_REPEAT;
        new_longa.f8 = longa_f8_sum / (float)BASE_REPEAT;
        new_longa.clear = longa_clear_sum / (float)BASE_REPEAT;
        new_longa.nir = longa_nir_sum / (float)BASE_REPEAT;
        new_longa.ts = (uint32_t)time(NULL);

        longa_buffer[buffer_head] = new_longa;
        buffer_head = (buffer_head + 1) % CIRCULAR_SIZE;
        if (buffer_count < CIRCULAR_SIZE) buffer_count++;

        float temp_buf[CIRCULAR_SIZE];
        float hum_buf[CIRCULAR_SIZE];
        for (int i = 0; i < buffer_count; ++i) {
            temp_buf[i] = longa_buffer[i].temp;
            hum_buf[i] = longa_buffer[i].humid;
        }

        float var_temp_buf = stats_variance_f(temp_buf, buffer_count, INVALID_F);
        float std_temp_buf = sqrtf(var_temp_buf);
        float var_hum_buf = stats_variance_f(hum_buf, buffer_count, INVALID_F);
        float std_hum_buf = sqrtf(var_hum_buf);

        bool temp_alert = (var_temp_buf > VAR_TEMP_THRESHOLD);
        bool humid_alert = (var_hum_buf > VAR_HUMID_THRESHOLD);

        printf("\n\n");
        printf("╔════════════════════════════════════════════════════════════╗\n");
        printf("║                 RESUMO DO CICLO LONGO (TS=%" PRIu32 ")           ║\n", new_longa.ts);
        printf("╠════════════════════════════════════════════════════════════╣\n");
        printf("║ MEDIAS DO AMBIENTE:                                        ║\n");
        printf("║ Temp: %5.2f °C | Umid: %5.2f %% | Press: %8.2f Pa     ║\n", new_longa.temp, new_longa.humid, new_longa.press);
        printf("║ Lux:  %5.1f                                            ║\n", new_longa.lux);
        printf("║ ESPECTRO AS7341:                                           ║\n");
        printf("║ F1-F4: %5.1f | %5.1f | %5.1f | %5.1f                 ║\n", new_longa.f1, new_longa.f2, new_longa.f3, new_longa.f4);
        printf("║ F5-F8: %5.1f | %5.1f | %5.1f | %5.1f                 ║\n", new_longa.f5, new_longa.f6, new_longa.f7, new_longa.f8);
        printf("║ CLR/NIR: %5.1f / %5.1f                                 ║\n", new_longa.clear, new_longa.nir);
        printf("╠════════════════════════════════════════════════════════════╣\n");
        printf("║ HISTORICO GERAL (ULTIMOS %02d CICLOS)                       ║\n", buffer_count);
        printf("║ Variancia Temp: %5.4f | Desvio Padrao: +-%5.4f °C      ║\n", var_temp_buf, std_temp_buf);
        printf("║ Variancia Umid: %5.4f | Desvio Padrao: +-%5.4f %%       ║\n", var_hum_buf, std_hum_buf);
        printf("║ Status Temp:    %-15s                            ║\n", temp_alert ? "INSTAVEL" : "ESTAVEL");
        printf("║ Status Umid:    %-15s                            ║\n", humid_alert ? "INSTAVEL" : "ESTAVEL");
        printf("╚════════════════════════════════════════════════════════════╝\n");

        int len = snprintf(payload, sizeof(payload),
            "{\"longa\":{\"temp\":%.2f,\"humid\":%.2f,\"press\":%.2f,\"lux\":%.1f,"
            "\"f1\":%.1f,\"f2\":%.1f,\"f3\":%.1f,\"f4\":%.1f,\"f5\":%.1f,\"f6\":%.1f,\"f7\":%.1f,\"f8\":%.1f,\"clr\":%.1f,\"nir\":%.1f,"
            "\"ts\":%" PRIu32 "},\"alerts\":{\"temp\":%s,\"humid\":%s},\"buf_count\":%d,\"var\":{\"temp\":%.3f,\"hum\":%.3f}}",
            new_longa.temp, new_longa.humid, new_longa.press, new_longa.lux,
            new_longa.f1, new_longa.f2, new_longa.f3, new_longa.f4,
            new_longa.f5, new_longa.f6, new_longa.f7, new_longa.f8,
            new_longa.clear, new_longa.nir, new_longa.ts,
            temp_alert?"true":"false", humid_alert?"true":"false",
            buffer_count, var_temp_buf, var_hum_buf);

        if (len > 0) {
            printf("\n[MQTT PAYLOAD] -> %s\n\n", payload);

            if (mqtt_is_connected()) {
                int msg_id = mqtt_publish_sensor_data(payload, len);
                ESP_LOGI(TAG, "MQTT publicado ID=%d", msg_id);
            } else {
                ESP_LOGW(TAG, "MQTT desconectado — aguardando proximo ciclo.");
            }
        }
    }
}

esp_err_t sensor_task_start(i2c_master_bus_handle_t bus_handle)
{
    BaseType_t res = xTaskCreate(sensor_loop_task, "sensor_task", SENSOR_TASK_STACK_SIZE, bus_handle, SENSOR_TASK_PRIORITY, NULL);
    return (res == pdPASS) ? ESP_OK : ESP_FAIL;
}