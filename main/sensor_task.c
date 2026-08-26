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
#include "audio_wakenet.h"

#include "mqtt_handler.h"
#include "stats_utils.h"
#include "esp_sleep.h"

static const char *TAG = "SENSOR_TASK";

#define SENSOR_TASK_STACK_SIZE 10240
#define SENSOR_TASK_PRIORITY   4
#define SENSOR_PWR_PIN         19  
#define MQTT_PAYLOAD_MAX_LEN 512

#define CIRCULAR_SIZE 3
#define BURST_CIRCULAR_SIZE 5
const float INVALID_F = -9999.0f;

// --- DATA STRUCTURES ---

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

// --- GLOBAL STATIC BUFFERS ---

static longa_t longa_buffer[10];
static int buffer_head = 0;
static int buffer_count = 0;

static burst_hist_t burst_buffer[BURST_CIRCULAR_SIZE];
static int burst_buffer_head = 0;
static int burst_buffer_count = 0;

// --- FORWARD DECLARATIONS ---

static void initialize_hardware(i2c_master_bus_handle_t bus_handle);
static void execute_hardware_reinit(i2c_master_bus_handle_t bus_handle);
static float read_microphone_db(void);
static void execute_short_burst(i2c_master_bus_handle_t bus_handle, int burst_index, int total_repeats, burst_hist_t *out_burst_mean);
static void push_burst_to_history(const burst_hist_t *new_burst);
static void print_burst_history(void);
static void process_long_cycle(float temp_sum, float hum_sum, float press_sum, float lux_sum, float mic_sum, float voc_sum, float nox_sum, float *spectral_sums);
static void execute_light_sleep(void);

// --- MAIN TASK LOOP ---

static void sensor_loop_task(void *pvParameters)
{
    i2c_master_bus_handle_t bus_handle = (i2c_master_bus_handle_t)pvParameters;
    setvbuf(stdout, NULL, _IONBF, 0);

    initialize_hardware(bus_handle);

    while (1) {
        float longa_temp_sum = 0.0f, longa_hum_sum = 0.0f, longa_press_sum = 0.0f;
        float longa_lux_sum = 0.0f, longa_mic_db_sum = 0.0f;
        float longa_voc_sum = 0.0f, longa_nox_sum = 0.0f;
        float longa_spectral_sums[10] = {0.0f}; // f1-f8, clear, nir

        const int BASE_REPEAT = 5;              

        for (int base_i = 0; base_i < BASE_REPEAT; ++base_i) {
            burst_hist_t current_burst_mean;
            
            // 1. Acquire and process the short 10s burst frame
            execute_short_burst(bus_handle, base_i + 1, BASE_REPEAT, &current_burst_mean);

            // 2. Commit burst mean data to historical circular buffer and render logs
            push_burst_to_history(&current_burst_mean);
            print_burst_history();

            // 3. Accumulate metrics for the upcoming long cycle processing stage
            longa_temp_sum   += current_burst_mean.temp;
            longa_hum_sum    += current_burst_mean.humid;
            longa_press_sum  += current_burst_mean.press;
            longa_lux_sum    += current_burst_mean.lux;
            longa_mic_db_sum += current_burst_mean.mic_db;
            longa_voc_sum    += current_burst_mean.voc;
            longa_nox_sum    += current_burst_mean.nox;
            
            longa_spectral_sums[0] += current_burst_mean.f1;
            longa_spectral_sums[1] += current_burst_mean.f2;
            longa_spectral_sums[2] += current_burst_mean.f3;
            longa_spectral_sums[3] += current_burst_mean.f4;
            longa_spectral_sums[4] += current_burst_mean.f5;
            longa_spectral_sums[5] += current_burst_mean.f6;
            longa_spectral_sums[6] += current_burst_mean.f7;
            longa_spectral_sums[7] += current_burst_mean.f8;
            longa_spectral_sums[8] += current_burst_mean.clear;
            longa_spectral_sums[9] += current_burst_mean.nir;

            // 4. Suspend operations using Light Sleep mode
           // execute_light_sleep();
           // execute_hardware_reinit(bus_handle);
        } 

        // --- END OF MAIN REPEAT LOOP: PROCESS LONG CYCLE ---
        process_long_cycle(longa_temp_sum, longa_hum_sum, longa_press_sum, longa_lux_sum, 
                           longa_mic_db_sum, longa_voc_sum, longa_nox_sum, longa_spectral_sums);
    }
}

// --- MODULAR SUBSYSTEM IMPLEMENTATIONS ---
static void initialize_hardware(i2c_master_bus_handle_t bus_handle)
{
    gpio_reset_pin(SENSOR_PWR_PIN);
    gpio_set_direction(SENSOR_PWR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(SENSOR_PWR_PIN, 1);

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    gpio_sleep_sel_dis(SENSOR_PWR_PIN);
#endif

    printf("\n> Powering VCC (GPIO %d) permanently and initializing drivers...\n",
           SENSOR_PWR_PIN);

    vTaskDelay(pdMS_TO_TICKS(100));

    if (dps310_init(bus_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Falha DPS310");
    }

    if (sgp41_init(bus_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Falha SGP41");
    }

    if (veml7700_init(bus_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Falha VEML7700");
    }

    if (as7341_init(bus_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Falha AS7341");
    }

    if (sht40_init(bus_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Falha SHT40");
    }

    // Leituras descartadas para estabilizar os pipelines
    float t = 0.0f;
    float p = 0.0f;
    uint16_t als_raw = 0;
    sht40_reading_t h = {0};

    vTaskDelay(pdMS_TO_TICKS(100));

    dps310_read(&t, &p);
    sht40_read_data(&h);
    veml7700_read_als(&als_raw);

    vTaskDelay(pdMS_TO_TICKS(100));
}

static void execute_hardware_reinit(i2c_master_bus_handle_t bus_handle)
{
    if (dps310_init(bus_handle) != ESP_OK) ESP_LOGE(TAG, "Falha DPS310 (reinit)");
    if (sgp41_init(bus_handle) != ESP_OK) ESP_LOGE(TAG, "Falha SGP41 (reinit)");
    if (veml7700_init(bus_handle) != ESP_OK) ESP_LOGE(TAG, "Falha VEML7700 (reinit)");
    if (as7341_init(bus_handle) != ESP_OK) ESP_LOGE(TAG, "Falha AS7341 (reinit)");
    if (sht40_init(bus_handle) != ESP_OK) ESP_LOGE(TAG, "Falha SHT40 (reinit)");
    vTaskDelay(pdMS_TO_TICKS(50)); 
}

static float read_microphone_db(void)
{
    // O I2S agora e consumido exclusivamente pela task do WakeNet.
    // Reaproveitamos o nivel calculado a partir do mesmo bloco de audio.
    return audio_wakenet_get_db();
}

static void execute_short_burst(i2c_master_bus_handle_t bus_handle, int burst_index, int total_repeats, burst_hist_t *out_burst_mean)
{
    const int SAMPLE_INTERVAL_MS = 500;      
    const int BURST_DURATION_MS = 10000;     
    const int BURST_SAMPLES = BURST_DURATION_MS / SAMPLE_INTERVAL_MS; 

    float burst_temps[BURST_SAMPLES], burst_humids[BURST_SAMPLES], burst_press[BURST_SAMPLES], burst_mic[BURST_SAMPLES];
    float burst_voc[BURST_SAMPLES], burst_nox[BURST_SAMPLES];
    float burst_lux[BURST_SAMPLES];
    
    float burst_f1[BURST_SAMPLES], burst_f2[BURST_SAMPLES], burst_f3[BURST_SAMPLES], burst_f4[BURST_SAMPLES];
    float burst_f5[BURST_SAMPLES], burst_f6[BURST_SAMPLES], burst_f7[BURST_SAMPLES], burst_f8[BURST_SAMPLES];
    float burst_clear[BURST_SAMPLES], burst_nir[BURST_SAMPLES];

    float temp = 0.0f, press = 0.0f;
    uint16_t als_raw = 0;
    sht40_reading_t humid;
    as7341_spectral_data_t spectral_data = {0};

    printf("\n--- SHORT BURST %02d/%02d (10s collection) ---\n", burst_index, total_repeats);
    printf(" Amo | T(°C) | U(%%) | P(Pa)    | Lux     | Mic(dB) | VOC | NOx | F1-F4           | F5-F8           | CLR / NIR \n");
    printf("-----+-------+-------+----------+---------+---------+-----+-----+-----------------+-----------------+-----------\n");

    for (int s = 0; s < BURST_SAMPLES; ++s) {
        float current_db = read_microphone_db();
        int dps_ret = dps310_read(&temp, &press);
        int veml_ret = veml7700_read_als(&als_raw);
        int sht_ret = sht40_read_data(&humid);
        int as_ret = as7341_read_all_channels(&spectral_data);
        
        int32_t local_voc = 0, local_nox = 0;
        esp_err_t sgp_err = sgp41_get_indices(humid.humidity, temp, &local_voc, &local_nox);
        if (sgp_err != ESP_OK) {
            local_voc = 100;
            local_nox = 1;
        }

        burst_temps[s]  = (dps_ret == ESP_OK) ? temp : INVALID_F;
        burst_humids[s] = (sht_ret == ESP_OK) ? humid.humidity : INVALID_F;
        burst_press[s]  = (dps_ret == ESP_OK) ? press : INVALID_F;
        burst_lux[s]    = (veml_ret == ESP_OK) ? veml7700_raw_to_lux(als_raw) : INVALID_F;
        burst_mic[s]    = current_db;
        burst_voc[s]    = (float)local_voc;
        burst_nox[s]    = (float)local_nox;

        burst_f1[s] = (as_ret == ESP_OK) ? (float)spectral_data.f1 : 0.0f;
        burst_f2[s] = (as_ret == ESP_OK) ? (float)spectral_data.f2 : 0.0f;
        burst_f3[s] = (as_ret == ESP_OK) ? (float)spectral_data.f3 : 0.0f;
        burst_f4[s] = (as_ret == ESP_OK) ? (float)spectral_data.f4 : 0.0f;
        burst_f5[s] = (as_ret == ESP_OK) ? (float)spectral_data.f5 : 0.0f;
        burst_f6[s] = (as_ret == ESP_OK) ? (float)spectral_data.f6 : 0.0f;
        burst_f7[s] = (as_ret == ESP_OK) ? (float)spectral_data.f7 : 0.0f;
        burst_f8[s] = (as_ret == ESP_OK) ? (float)spectral_data.f8 : 0.0f;
        burst_clear[s] = (as_ret == ESP_OK) ? (float)spectral_data.clear : 0.0f;
        burst_nir[s]   = (as_ret == ESP_OK) ? (float)spectral_data.nir : 0.0f;
       
        printf(" %02d  | %5.2f | %5.2f | %8.2f | %7.2f | %7.2f | %3" PRId32 " | %3" PRId32 " | %.0f %.0f %.0f %.0f | %.0f %.0f %.0f %.0f | %.0f / %.0f\n", 
               s+1, burst_temps[s], burst_humids[s], burst_press[s], burst_lux[s], burst_mic[s], local_voc, local_nox,
               burst_f1[s], burst_f2[s], burst_f3[s], burst_f4[s], burst_f5[s], burst_f6[s], burst_f7[s], burst_f8[s], burst_clear[s], burst_nir[s]);
        
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS-64));
    }

    out_burst_mean->temp   = stats_mean_f(burst_temps, BURST_SAMPLES, INVALID_F);
    out_burst_mean->humid  = stats_mean_f(burst_humids, BURST_SAMPLES, INVALID_F);
    out_burst_mean->press  = stats_mean_f(burst_press, BURST_SAMPLES, INVALID_F);
    out_burst_mean->lux    = stats_mean_f(burst_lux, BURST_SAMPLES, INVALID_F);
    out_burst_mean->mic_db = stats_mean_f(burst_mic, BURST_SAMPLES, 0.0f);
    out_burst_mean->voc    = stats_mean_f(burst_voc, BURST_SAMPLES, 0.0f);
    out_burst_mean->nox    = stats_mean_f(burst_nox, BURST_SAMPLES, 0.0f);
    out_burst_mean->f1     = stats_mean_f(burst_f1, BURST_SAMPLES, 0.0f);
    out_burst_mean->f2     = stats_mean_f(burst_f2, BURST_SAMPLES, 0.0f);
    out_burst_mean->f3     = stats_mean_f(burst_f3, BURST_SAMPLES, 0.0f);
    out_burst_mean->f4     = stats_mean_f(burst_f4, BURST_SAMPLES, 0.0f);
    out_burst_mean->f5     = stats_mean_f(burst_f5, BURST_SAMPLES, 0.0f);
    out_burst_mean->f6     = stats_mean_f(burst_f6, BURST_SAMPLES, 0.0f);
    out_burst_mean->f7     = stats_mean_f(burst_f7, BURST_SAMPLES, 0.0f);
    out_burst_mean->f8     = stats_mean_f(burst_f8, BURST_SAMPLES, 0.0f);
    out_burst_mean->clear  = stats_mean_f(burst_clear, BURST_SAMPLES, 0.0f);
    out_burst_mean->nir    = stats_mean_f(burst_nir, BURST_SAMPLES, 0.0f);
    out_burst_mean->timestamp = (uint32_t)time(NULL);

    float burst_temp_var = stats_variance_f(burst_temps, BURST_SAMPLES, INVALID_F);
    float burst_hum_var  = stats_variance_f(burst_humids, BURST_SAMPLES, INVALID_F);

    printf("-----+-------+-------+----------+---------+---------+-----+-----+-----------------+-----------------+-----------\n");
    printf("[MED]| %5.2f | %5.2f | %8.2f | %7.2f | %7.2f | %3.0f | %3.0f | %.0f %.0f %.0f %.0f | %.0f %.0f %.0f %.0f | %.0f / %.0f\n", 
            out_burst_mean->temp, out_burst_mean->humid, out_burst_mean->press, out_burst_mean->lux, out_burst_mean->mic_db, out_burst_mean->voc, out_burst_mean->nox,
            out_burst_mean->f1, out_burst_mean->f2, out_burst_mean->f3, out_burst_mean->f4, out_burst_mean->f5, out_burst_mean->f6, out_burst_mean->f7, out_burst_mean->f8, out_burst_mean->clear, out_burst_mean->nir);

    printf("\n>>> ESTATISTICAS DO BURST %d <<<\n", burst_index);
    printf("    Temp -> Media: %5.2f °C | Variancia: %5.4f | Desvio Padrao: +-%5.4f °C\n", out_burst_mean->temp, burst_temp_var, sqrtf(burst_temp_var));
    printf("    Umid -> Media: %5.2f %%  | Variancia: %5.4f | Desvio Padrao: +-%5.4f %%\n", out_burst_mean->humid, burst_hum_var, sqrtf(burst_hum_var));
    printf("    VOC  -> Media: %5.1f    | NOx -> Media: %5.1f\n", out_burst_mean->voc, out_burst_mean->nox);
}

static void push_burst_to_history(const burst_hist_t *new_burst)
{
    burst_buffer[burst_buffer_head] = *new_burst;
    burst_buffer_head = (burst_buffer_head + 1) % BURST_CIRCULAR_SIZE;
    if (burst_buffer_count < BURST_CIRCULAR_SIZE) burst_buffer_count++;
}

static void print_burst_history(void)
{
    float hist_temps[BURST_CIRCULAR_SIZE];
    float hist_humids[BURST_CIRCULAR_SIZE];
    float hist_voc[BURST_CIRCULAR_SIZE];
    float hist_nox[BURST_CIRCULAR_SIZE];

    printf("\n--- BURST HISTORY (LAST %02d BURSTS) ---\n", burst_buffer_count);
    printf(" idx | Temp(°C) | Umid(%%) | Press(Pa) | Lux | Mic(dB) | VOC | NOx \n");
    printf("-----+----------+---------+-----------+-----+---------+-----+-----\n");
    
    // Pegamos a última amostra inserida para avaliar as flags instantâneas de estado ambiente
    int last_inserted_idx = (burst_buffer_head - 1 + BURST_CIRCULAR_SIZE) % BURST_CIRCULAR_SIZE;
    float current_temp  = burst_buffer[last_inserted_idx].temp;
    float current_humid = burst_buffer[last_inserted_idx].humid;
    float current_lux   = burst_buffer[last_inserted_idx].lux;
    float current_voc   = burst_buffer[last_inserted_idx].voc;

    for (int i = 0; i < burst_buffer_count; ++i) {
        int idx = (burst_buffer_head - burst_buffer_count + i + BURST_CIRCULAR_SIZE) % BURST_CIRCULAR_SIZE;
        
        printf("  %d  |  %5.2f   |  %5.2f  | %9.1f | %3.0f |  %6.2f | %3.0f | %3.0f\n",
               i + 1, burst_buffer[idx].temp, burst_buffer[idx].humid, burst_buffer[idx].press,
               burst_buffer[idx].lux, burst_buffer[idx].mic_db, burst_buffer[idx].voc, burst_buffer[idx].nox);

        hist_temps[i]  = burst_buffer[idx].temp;
        hist_humids[i] = burst_buffer[idx].humid;
        hist_voc[i]    = burst_buffer[idx].voc;
        hist_nox[i]    = burst_buffer[idx].nox;
    }
    printf("--------------------------------------------------------------------\n");

    if (burst_buffer_count > 1) {
        float var_temp = stats_variance_f(hist_temps, burst_buffer_count, INVALID_F);
        float var_hum  = stats_variance_f(hist_humids, burst_buffer_count, INVALID_F);
        float var_voc  = stats_variance_f(hist_voc, burst_buffer_count, INVALID_F);
        float var_nox  = stats_variance_f(hist_nox, burst_buffer_count, INVALID_F);

        // --- EVALUATE ENVIRONMENTAL FLAGS & SYSTEM THRESHOLDS ---
        bool flag_temp_variance = (var_temp > 5.0f);
        bool flag_temp_range    = (current_temp < 20.0f || current_temp > 29.0f);
        bool flag_humid_elderly = (current_humid < 45.0f || current_humid > 85.0f);
        bool flag_voc_alert     = (current_voc > 120.0f);
        const char* lux_state   = (current_lux < 10.0f) ? "DARK (OFF)" : "LIT (ON)";

        printf(">>> SYSTEM MONITORING FLAGS & CRITICAL ALERTS <<<\n");
        printf("    [TEMP VAR] : %s (Var: %5.4f °C)\n", flag_temp_variance ? " CRITICAL SPIKE" : "OK STABLE", var_temp);
        printf("    [TEMP COMF]: %s (Current: %5.2f °C | Ideal: 20-29°C)\n", flag_temp_range ? " OUT OF RANGE" : "OK COMFORT", current_temp);
        printf("    [ELDER HUM]: %s (Current: %5.2f %%  | Ideal: 45-85%%)\n", flag_humid_elderly ? " ATTENTION (RISK)" : "OK HEALTHY", current_humid);
        printf("    [LUX STATE]: %s (Current: %.0f Lux)\n", lux_state, current_lux);
        printf("    [AIR QUAL] : %s (Current VOC Index: %.0f | Max Safe: 120)\n", flag_voc_alert ? " POLLUTED / BAD" : "OK CLEAN AIR", current_voc);
        printf("    Var Hist   | Var Umid: %5.4f | Var VOC: %5.2f | Var NOx: %5.2f\n", var_hum, var_voc, var_nox);
        printf("--------------------------------------------------------------------\n");
    }
}

static void execute_light_sleep(void)
{
    const int BASE_SLEEP_MS = 10000;         
    printf("\n> Dormindo por %d segundos...\n", BASE_SLEEP_MS / 1000);
    vTaskDelay(pdMS_TO_TICKS(50));

    esp_err_t t_res = esp_sleep_enable_timer_wakeup(((uint64_t)BASE_SLEEP_MS) * 1000ULL);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

    if (t_res == ESP_OK) {
        esp_light_sleep_start();
    } else {
        vTaskDelay(pdMS_TO_TICKS(BASE_SLEEP_MS));
    }
}

static void process_long_cycle(float temp_sum, float hum_sum, float press_sum, float lux_sum, float mic_sum, float voc_sum, float nox_sum, float *spectral_sums)
{
    const int BASE_REPEAT = 5;              
    longa_t new_longa;

    new_longa.temp   = temp_sum / (float)BASE_REPEAT;
    new_longa.humid  = hum_sum / (float)BASE_REPEAT;
    new_longa.press  = press_sum / (float)BASE_REPEAT;
    new_longa.lux    = lux_sum / (float)BASE_REPEAT;
    new_longa.mic_db = mic_sum / (float)BASE_REPEAT;
    new_longa.voc    = voc_sum / (float)BASE_REPEAT;
    new_longa.nox    = nox_sum / (float)BASE_REPEAT;
    
    new_longa.f1    = spectral_sums[0] / (float)BASE_REPEAT;
    new_longa.f2    = spectral_sums[1] / (float)BASE_REPEAT;
    new_longa.f3    = spectral_sums[2] / (float)BASE_REPEAT;
    new_longa.f4    = spectral_sums[3] / (float)BASE_REPEAT;
    new_longa.f5    = spectral_sums[4] / (float)BASE_REPEAT;
    new_longa.f6    = spectral_sums[5] / (float)BASE_REPEAT;
    new_longa.f7    = spectral_sums[6] / (float)BASE_REPEAT;
    new_longa.f8    = spectral_sums[7] / (float)BASE_REPEAT;
    new_longa.clear = spectral_sums[8] / (float)BASE_REPEAT;
    new_longa.nir   = spectral_sums[9] / (float)BASE_REPEAT;
    new_longa.ts    = (uint32_t)time(NULL);

    longa_buffer[buffer_head] = new_longa;
    buffer_head = (buffer_head + 1) % CIRCULAR_SIZE;
    if (buffer_count < CIRCULAR_SIZE) buffer_count++;

    float temp_buf[CIRCULAR_SIZE];
    for (int i = 0; i < buffer_count; ++i) {
        temp_buf[i] = longa_buffer[i].temp;
    }
    float var_temp_buf = stats_variance_f(temp_buf, buffer_count, INVALID_F);

    printf("\n\n");
    printf(" RESUMO DO CICLO LONGO (TS=%" PRIu32 ") \n", new_longa.ts);
    printf("MEDIAS DO AMBIENTE: \n");
    printf("Temp: %5.2f °C | Umid: %5.2f %% | Press: %8.2f Pa \n", new_longa.temp, new_longa.humid, new_longa.press);
    printf(" Lux:  %5.1f    | Mic:  %5.2f dB | VOC:  %5.1f    | NOx:  %5.1f\n", new_longa.lux, new_longa.mic_db, new_longa.voc, new_longa.nox);
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(30));
    printf(" HISTORICO GERAL (ULTIMOS %02d CICLOS) \n", buffer_count);
    printf(" Variancia Temp: %5.4f | Desvio Padrao: +-%5.4f °C \n", var_temp_buf, sqrtf(var_temp_buf));
}

esp_err_t sensor_task_start(i2c_master_bus_handle_t bus_handle)
{
    BaseType_t res = xTaskCreate(sensor_loop_task, "sensor_task", SENSOR_TASK_STACK_SIZE, bus_handle, SENSOR_TASK_PRIORITY, NULL);
    return (res == pdPASS) ? ESP_OK : ESP_FAIL;
}