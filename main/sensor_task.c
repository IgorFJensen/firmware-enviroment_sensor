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
#include "sgp41.h"
#include "veml7700.h"
#include "as7341.h"
#include "sht40.h"
#include "mic_ics43434.h"

#include "mqtt_handler.h"
#include "stats_utils.h"
#include "esp_sleep.h"

static const char *TAG = "SENSOR_TASK";

#define SENSOR_TASK_STACK_SIZE 10240
#define SENSOR_TASK_PRIORITY   4
#define SENSOR_PWR_PIN         19  

#define AUDIO_SAMPLES_PER_READ 1024
static int32_t audio_buffer[AUDIO_SAMPLES_PER_READ];

// Definitions for circular buffers sizes
#define CIRCULAR_SIZE 3
#define BURST_CIRCULAR_SIZE 5

static void sensor_loop_task(void *pvParameters)
{
    i2c_master_bus_handle_t bus_handle = (i2c_master_bus_handle_t)pvParameters;
    setvbuf(stdout, NULL, _IONBF, 0);

    // Muted variables for MQTT payload logic
    #if 0
    char payload[800];
    bool temp_alert = false;
    bool humid_alert = false;
    #endif

    // --- INITIALIZE GPIO 19 TO BE ALWAYS ON ---
    gpio_reset_pin(SENSOR_PWR_PIN);
    gpio_set_direction(SENSOR_PWR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(SENSOR_PWR_PIN, 1);
    
    // Enable sleep override so the GPIO pin maintains its state high during Light Sleep
    #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    gpio_sleep_sel_dis(SENSOR_PWR_PIN);
    #endif

    const int SAMPLE_INTERVAL_MS = 500;      
    const int BURST_DURATION_MS = 10000;     
    const int BURST_SAMPLES = BURST_DURATION_MS / SAMPLE_INTERVAL_MS; 
    const int BASE_SLEEP_MS = 10000;         
    const int BASE_REPEAT = 5;              

    const float INVALID_F = -9999.0f;        
    const float VAR_TEMP_THRESHOLD = 4.0f;   
    const float VAR_HUMID_THRESHOLD = 10.0f; 

    sht40_reading_t humid;
    float temp = 0.0f, press = 0.0f;
    uint16_t white_raw = 0;
    int32_t vocindex = 0;
    as7341_spectral_data_t spectral_data = {0};

    // Long cycle structure
    typedef struct {
        float temp;
        float humid;
        float press;
        float lux;
        float mic_db;
        float voc;
        float nox;
        float f1, f2, f3, f4, f5, f6, f7, f8, clear, nir;
        uint32_t ts;
    } longa_t;

    // Burst history structure
    typedef struct {
        float temp;
        float humid;
        float press;
        float lux;
        float mic_db;
        float voc;
        float nox;
        float f1, f2, f3, f4, f5, f6, f7, f8, clear, nir;
        uint32_t timestamp;
    } burst_hist_t;

    static longa_t longa_buffer[10];
    static int buffer_head = 0;
    static int buffer_count = 0;

    static burst_hist_t burst_buffer[BURST_CIRCULAR_SIZE];
    static int burst_buffer_head = 0;
    static int burst_buffer_count = 0;

    printf("\n> Powering VCC (GPIO %d) permanently and initializing drivers...\n", SENSOR_PWR_PIN);
    vTaskDelay(pdMS_TO_TICKS(100)); 

    dps310_init(bus_handle);
    sgp41_init(bus_handle);
    veml7700_init(bus_handle);
    as7341_init(bus_handle);
    sht40_init(bus_handle);

    // DUMMY READ: Discard initial readings
    vTaskDelay(pdMS_TO_TICKS(100));
    dps310_read(&temp, &press);
    sht40_read_data(&humid);
    veml7700_read_white(&white_raw);
    vTaskDelay(pdMS_TO_TICKS(100));

    while (1) {
        float longa_temp_sum = 0.0f, longa_hum_sum = 0.0f, longa_press_sum = 0.0f;
        float longa_lux_sum = 0.0f, longa_mic_db_sum = 0.0f;
        float longa_voc_sum = 0.0f, longa_nox_sum = 0.0f;

        // Spectral channels accumulators
        float longa_f1_sum = 0.0f, longa_f2_sum = 0.0f, longa_f3_sum = 0.0f, longa_f4_sum = 0.0f;
        float longa_f5_sum = 0.0f, longa_f6_sum = 0.0f, longa_f7_sum = 0.0f, longa_f8_sum = 0.0f;
        float longa_clear_sum = 0.0f, longa_nir_sum = 0.0f;

        for (int base_i = 0; base_i < BASE_REPEAT; ++base_i) {

            float burst_temps[BURST_SAMPLES], burst_humids[BURST_SAMPLES], burst_press[BURST_SAMPLES], burst_mic[BURST_SAMPLES];
            float burst_voc[BURST_SAMPLES], burst_nox[BURST_SAMPLES];
            uint16_t burst_lux[BURST_SAMPLES];
            printf("\n--- SHORT BURST %02d/%02d (10s collection) ---\n", base_i+1, BASE_REPEAT);
            printf(" Amo | T(°C) | U(%%) | P(Pa)    | Lux | Mic(dB) | VOC | NOx | F1-F4           | F5-F8           | CLR / NIR \n");
            printf("-----+-------+-------+----------+-----+---------+-----+-----+-----------------+-----------------+-----------\n");
            
            // Arrays for all AS7341 channels
            float burst_f1[BURST_SAMPLES], burst_f2[BURST_SAMPLES], burst_f3[BURST_SAMPLES], burst_f4[BURST_SAMPLES];
            float burst_f5[BURST_SAMPLES], burst_f6[BURST_SAMPLES], burst_f7[BURST_SAMPLES], burst_f8[BURST_SAMPLES];
            float burst_clear[BURST_SAMPLES], burst_nir[BURST_SAMPLES];

            for (int s = 0; s < BURST_SAMPLES; ++s) {
                // MIC Acquisition
                size_t bytes_read = 0;
                esp_err_t mic_err = mic_ics43434_read(audio_buffer, sizeof(audio_buffer), &bytes_read);
                float current_db = 0.0f;
                if (mic_err == ESP_OK && bytes_read > 0) {
                    int samples_read = bytes_read / sizeof(int32_t);
                    double sum_squares = 0.0;
                    
                    for (int i = 0; i < samples_read; i++) {
                        int32_t sample = audio_buffer[i] >> 8; 
                        sum_squares += ((double)sample * (double)sample);
                    }
                    
                    double rms = sqrt(sum_squares / samples_read);
                    if (rms > 0) {
                        current_db = 20.0 * log10(rms);
                    }
                } else {
                    ESP_LOGW(TAG, "Failed to read from ICS-43434 or empty buffer");
                }

                int dps_ret = dps310_read(&temp, &press);
                int veml_ret = veml7700_read_white(&white_raw);
                int sht_ret = sht40_read_data(&humid);
                int as_ret = as7341_read_all_channels(&spectral_data);
                
                int32_t local_voc = 0;
                int32_t local_nox = 0;
                esp_err_t sgp_err = sgp41_get_indices(humid.humidity, temp, &local_voc, &local_nox);

                if (sgp_err != ESP_OK) {
                    local_voc = 100;
                    local_nox = 1;
                }

                burst_temps[s] = (dps_ret == ESP_OK) ? temp : INVALID_F;
                burst_humids[s] = (sht_ret == ESP_OK) ? humid.humidity : INVALID_F;
                burst_press[s] = (dps_ret == ESP_OK) ? press : INVALID_F;
                burst_lux[s] = (veml_ret == ESP_OK) ? white_raw : 0;
                burst_mic[s] = current_db;
                burst_voc[s] = (float)local_voc;
                burst_nox[s] = (float)local_nox;

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
               
                printf(" %02d  | %5.2f | %5.2f | %8.2f | %3u | %7.2f | %3" PRId32 " | %3" PRId32 " | %.0f %.0f %.0f %.0f | %.0f %.0f %.0f %.0f | %.0f / %.0f\n", 
                       s+1, burst_temps[s], burst_humids[s], burst_press[s], burst_lux[s], burst_mic[s],
                       local_voc, local_nox,
                       burst_f1[s], burst_f2[s], burst_f3[s], burst_f4[s], 
                       burst_f5[s], burst_f6[s], burst_f7[s], burst_f8[s], 
                       burst_clear[s], burst_nir[s]);

                vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS-64));
            }

            float burst_temp_mean = stats_mean_f(burst_temps, BURST_SAMPLES, INVALID_F);
            float burst_temp_var  = stats_variance_f(burst_temps, BURST_SAMPLES, INVALID_F);
            float burst_temp_std  = sqrtf(burst_temp_var);

            float burst_hum_mean  = stats_mean_f(burst_humids, BURST_SAMPLES, INVALID_F);
            float burst_hum_var   = stats_variance_f(burst_humids, BURST_SAMPLES, INVALID_F);
            float burst_hum_std   = sqrtf(burst_hum_var);

            float burst_press_mean = stats_mean_f(burst_press, BURST_SAMPLES, INVALID_F);
            float burst_lux_mean = stats_mean_u16(burst_lux, BURST_SAMPLES, 0);
            float burst_mic_mean = stats_mean_f(burst_mic, BURST_SAMPLES, 0.0f);
            float burst_voc_mean = stats_mean_f(burst_voc, BURST_SAMPLES, 0.0f);
            float burst_nox_mean = stats_mean_f(burst_nox, BURST_SAMPLES, 0.0f);
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

            printf("-----+-------+-------+----------+-----+---------+-----+-----+-----------------+-----------------+-----------\n");
            printf("[MED]| %5.2f | %5.2f | %8.2f | %3.0f | %7.2f | %3.0f | %3.0f | %.0f %.0f %.0f %.0f | %.0f %.0f %.0f %.0f | %.0f / %.0f\n", 
                    burst_temp_mean, burst_hum_mean, burst_press_mean, burst_lux_mean, burst_mic_mean,
                    burst_voc_mean, burst_nox_mean,
                    burst_f1_mean, burst_f2_mean, burst_f3_mean, burst_f4_mean,
                    burst_f5_mean, burst_f6_mean, burst_f7_mean, burst_f8_mean,
                    burst_clear_mean, burst_nir_mean);

            printf("\n>>> ESTATISTICAS DO BURST %d <<<\n", base_i+1);
            printf("    Temp -> Media: %5.2f °C | Variancia: %5.4f | Desvio Padrao: +-%5.4f °C\n", burst_temp_mean, burst_temp_var, burst_temp_std);
            printf("    Umid -> Media: %5.2f %%  | Variancia: %5.4f | Desvio Padrao: +-%5.4f %%\n", burst_hum_mean, burst_hum_var, burst_hum_std);
            printf("    VOC  -> Media: %5.1f    | NOx -> Media: %5.1f\n", burst_voc_mean, burst_nox_mean);

            // Save current burst means into the burst circular buffer
            burst_hist_t new_burst;
            new_burst.temp = burst_temp_mean;
            new_burst.humid = burst_hum_mean;
            new_burst.press = burst_press_mean;
            new_burst.lux = burst_lux_mean;
            new_burst.mic_db = burst_mic_mean;
            new_burst.voc = burst_voc_mean;
            new_burst.nox = burst_nox_mean;
            new_burst.f1 = burst_f1_mean;
            new_burst.f2 = burst_f2_mean;
            new_burst.f3 = burst_f3_mean;
            new_burst.f4 = burst_f4_mean;
            new_burst.f5 = burst_f5_mean;
            new_burst.f6 = burst_f6_mean;
            new_burst.f7 = burst_f7_mean;
            new_burst.f8 = burst_f8_mean;
            new_burst.clear = burst_clear_mean;
            new_burst.nir = burst_nir_mean;
            new_burst.timestamp = (uint32_t)time(NULL);

            burst_buffer[burst_buffer_head] = new_burst;
            burst_buffer_head = (burst_buffer_head + 1) % BURST_CIRCULAR_SIZE;
            if (burst_buffer_count < BURST_CIRCULAR_SIZE) burst_buffer_count++;

            // Print burst history from oldest to newest
            printf("\n--- BURST HISTORY (LAST %02d BURSTS) ---\n", burst_buffer_count);
            printf(" idx | Temp(°C) | Umid(%%) | Press(Pa) | Lux | Mic(dB) | VOC | NOx \n");
            printf("-----+----------+---------+-----------+-----+---------+-----+-----\n");
            for (int i = 0; i < burst_buffer_count; ++i) {
                int idx = (burst_buffer_head - burst_buffer_count + i + BURST_CIRCULAR_SIZE) % BURST_CIRCULAR_SIZE;
                printf("  %d  |  %5.2f   |  %5.2f  | %9.1f | %3.0f |  %6.2f | %3.0f | %3.0f\n",
                       i + 1, burst_buffer[idx].temp, burst_buffer[idx].humid, burst_buffer[idx].press,
                       burst_buffer[idx].lux, burst_buffer[idx].mic_db, burst_buffer[idx].voc, burst_buffer[idx].nox);
            }
            printf("--------------------------------------------------------------------\n");

            longa_temp_sum += burst_temp_mean;
            longa_hum_sum += burst_hum_mean;
            longa_press_sum += burst_press_mean;
            longa_lux_sum += burst_lux_mean;
            longa_mic_db_sum += burst_mic_mean;
            longa_voc_sum += burst_voc_mean;
            longa_nox_sum += burst_nox_mean;
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

            printf("\n> Dormindo por %d segundos...\n", BASE_SLEEP_MS / 1000);
            vTaskDelay(pdMS_TO_TICKS(50));

            esp_err_t t_res = esp_sleep_enable_timer_wakeup(((uint64_t)BASE_SLEEP_MS) * 1000ULL);
            esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

            if (t_res == ESP_OK) {
                esp_light_sleep_start();
            } else {
                vTaskDelay(pdMS_TO_TICKS(BASE_SLEEP_MS));
            }
            
            // Re-initialize drivers without resetting GPIO19 to keep device logic active
            if (dps310_init(bus_handle) != ESP_OK) ESP_LOGE(TAG, "Falha DPS310 (reinit)");
            if (sgp41_init(bus_handle) != ESP_OK) ESP_LOGE(TAG, "Falha SGP41 (reinit)");
            if (veml7700_init(bus_handle) != ESP_OK) ESP_LOGE(TAG, "Falha VEML7700 (reinit)");
            if (as7341_init(bus_handle) != ESP_OK) ESP_LOGE(TAG, "Falha AS7341 (reinit)");
            if (sht40_init(bus_handle) != ESP_OK) ESP_LOGE(TAG, "Falha SHT40 (reinit)");

            vTaskDelay(pdMS_TO_TICKS(50)); 
        } 

        // --- LONG CYCLE CALCULATIONS ---
        longa_t new_longa;
        new_longa.temp = longa_temp_sum / (float)BASE_REPEAT;
        new_longa.humid = longa_hum_sum / (float)BASE_REPEAT;
        new_longa.press = longa_press_sum / (float)BASE_REPEAT;
        new_longa.lux = longa_lux_sum / (float)BASE_REPEAT;
        new_longa.mic_db = longa_mic_db_sum / (float)BASE_REPEAT;
        new_longa.voc = longa_voc_sum / (float)BASE_REPEAT;
        new_longa.nox = longa_nox_sum / (float)BASE_REPEAT;
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

        if(1){
            printf("\n\n");
            printf(" RESUMO DO CICLO LONGO (TS=%" PRIu32 ") \n", new_longa.ts);
            printf("MEDIAS DO AMBIENTE: \n");
            printf("Temp: %5.2f °C | Umid: %5.2f %% | Press: %8.2f Pa \n", new_longa.temp, new_longa.humid, new_longa.press);
            printf(" Lux:  %5.1f    | Mic:  %5.2f dB | VOC:  %5.1f    | NOx:  %5.1f\n", new_longa.lux, new_longa.mic_db, new_longa.voc, new_longa.nox);
            fflush(stdout);
            vTaskDelay(pdMS_TO_TICKS(30));
            printf(" HISTORICO GERAL (ULTIMOS %02d CICLOS) \n", buffer_count);
            printf(" Variancia Temp: %5.4f | Desvio Padrao: +-%5.4f °C \n", var_temp_buf, std_temp_buf);
        }
    }
}

esp_err_t sensor_task_start(i2c_master_bus_handle_t bus_handle)
{
    BaseType_t res = xTaskCreate(sensor_loop_task, "sensor_task", SENSOR_TASK_STACK_SIZE, bus_handle, SENSOR_TASK_PRIORITY, NULL);
    return (res == pdPASS) ? ESP_OK : ESP_FAIL;
}