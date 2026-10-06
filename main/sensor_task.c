/* main/sensor_task.c
 * Made by Igor Jensen - UFES - LAEEC
 *
 * Revisao out/2026:
 *   - temperatura principal agora vem do SHT40 (+-0.2 C) em vez do DPS368
 *     (+-0.5 C); o DPS368 continua sendo lido e aparece como "T_dps"
 *   - correcao de offset de temperatura (TEMP_OFFSET_C) com recalculo da UR
 *   - limites unificados em env_limits.h: terminal e MQTT usam as MESMAS flags
 *   - leituras com falha nao entram mais como 0 nas medias
 *   - flags de VOC/NOx so valem depois do aquecimento do algoritmo Sensirion
 *   - Gas Index atualizado a cada 1.0 s de fato (antes oscilava entre 1 e 1.5 s)
 *   - saida do terminal reorganizada
 */
#include "sensor_task.h"
#include <stdio.h>
#include <stdbool.h>
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
#include "audio_ml.h"

#include "mqtt_handler.h"
#include "risk_alert.h"
#include "stats_utils.h"
#include "env_limits.h"
#include "esp_sleep.h"
#include "esp_timer.h"

static const char *TAG = "SENSOR_TASK";

#define SENSOR_TASK_STACK_SIZE 10240
#define SENSOR_TASK_PRIORITY   4
#define SENSOR_PWR_PIN         19
#define MQTT_PAYLOAD_MAX_LEN       1280
#define MQTT_PUBLISH_TIMEOUT_MS    5000
#define AUDIO_PAUSE_TIMEOUT_MS     1500
#define MQTT_PRE_TX_QUIET_MS       100
#define MQTT_POST_TX_GUARD_MS      250

#define BASE_REPEAT                5        /* bursts por ciclo longo        */
#define SAMPLE_INTERVAL_MS         500
#define BURST_DURATION_MS          10000
#define BURST_SAMPLES              (BURST_DURATION_MS / SAMPLE_INTERVAL_MS)

/* Gas Index: periodo nominal 1 s. Com amostragem a cada 500 ms, um limiar
 * de exatamente 1.000 s fazia o update cair as vezes para 1.5 s (jitter). */
#define SGP_UPDATE_MIN_US          950000LL

/* AS7341: ATIME=29, ASTEP=999 -> fundo de escala (29+1)*(999+1) = 30000 */
#define AS7341_FULL_SCALE          30000.0f

#define CIRCULAR_SIZE 3
#define BURST_CIRCULAR_SIZE 5
static const float INVALID_F = -9999.0f;

/* ----------------------------- terminal ----------------------------- */
#if TERM_USE_COLOR
  #define C_RST  "\033[0m"
  #define C_DIM  "\033[2m"
  #define C_BLD  "\033[1m"
  #define C_RED  "\033[31m"
  #define C_GRN  "\033[32m"
  #define C_YEL  "\033[33m"
  #define C_CYN  "\033[36m"
#else
  #define C_RST  ""
  #define C_DIM  ""
  #define C_BLD  ""
  #define C_RED  ""
  #define C_GRN  ""
  #define C_YEL  ""
  #define C_CYN  ""
#endif
#define LINE_EQ "=============================================================================="
#define LINE_DS "------------------------------------------------------------------------------"

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
    float temp;            /* SHT40 corrigido (TEMP_OFFSET_C)            */
    float humid;           /* SHT40 recalculada p/ a temperatura corrigida */
    float temp_raw;        /* SHT40 sem correcao                          */
    float humid_raw;
    float temp_dps;        /* temperatura interna do DPS368                */
    float press;
    float lux;
    float mic_db;
    float voc;
    float nox;
    float temp_variance;
    float humidity_variance;
    float f1, f2, f3, f4, f5, f6, f7, f8, clear, nir;
    uint8_t n_sht, n_dps, n_veml, n_as;   /* amostras validas no burst */
    bool as_saturated;
    bool veml_saturated;
    uint32_t timestamp;
} burst_hist_t;

/* Flags calculadas UMA vez e usadas no terminal e no MQTT */
typedef struct {
    bool temp_valid, humid_valid, press_valid, lux_valid;
    bool measurement_valid;     /* todos os 4 acima (compatibilidade MQTT) */
    bool gas_ready;
    bool audio_valid;
    audio_ml_risk_status_t audio;
    float temp_stddev_hist;     /* desvio padrao entre os ultimos bursts */
    bool temp_low, temp_high, temp_out_of_range;
    bool temp_variation;
    bool humid_low, humid_high, humidity_out_of_range;
    bool dark;
    bool voc_elevated;
    bool nox_elevated;
    bool audio_risk;
    bool critical;
} env_flags_t;

// --- GLOBAL STATIC BUFFERS ---

static longa_t longa_buffer[CIRCULAR_SIZE];
static int buffer_head = 0;
static int buffer_count = 0;

static burst_hist_t burst_buffer[BURST_CIRCULAR_SIZE];
static int burst_buffer_head = 0;
static int burst_buffer_count = 0;

// SGP41 follows the 1 s cadence expected by the Sensirion Gas Index Algorithm.
static bool s_sgp41_ready = false;
static bool s_sgp41_conditioned = false;
static int32_t s_last_voc_index = 100;
static int32_t s_last_nox_index = 1;
static int64_t s_last_sgp_update_us = 0;
static int64_t s_first_sgp_index_us = 0;
static uint32_t s_sgp_ok_count = 0;
static uint32_t s_mqtt_cycle_number = 0;

// --- FORWARD DECLARATIONS ---

static void initialize_hardware(i2c_master_bus_handle_t bus_handle);
static void execute_hardware_reinit(i2c_master_bus_handle_t bus_handle) __attribute__((unused));
static float read_microphone_db(void);
static void execute_short_burst(int burst_index, int total_repeats, burst_hist_t *out_burst_mean);
static void push_burst_to_history(const burst_hist_t *new_burst);
static void evaluate_flags(const burst_hist_t *mean, env_flags_t *f);
static void print_burst_report(const burst_hist_t *mean, const env_flags_t *f, int burst_index, int total_bursts);
static void publish_burst_mean(const burst_hist_t *mean, const env_flags_t *f, int burst_index, int total_bursts);
static void process_long_cycle(const float *sums);
static void execute_light_sleep(void) __attribute__((unused));

// --- HELPERS ---

static float mean_or_invalid(const float *data, size_t n)
{
    return (stats_count_valid_f(data, n, INVALID_F) > 0)
               ? stats_mean_f(data, n, INVALID_F)
               : INVALID_F;
}

static inline bool is_valid(float v) { return v != INVALID_F; }

/* Pressao de vapor de saturacao (Magnus, hPa) */
static float magnus_es(float t_c)
{
    return 6.112f * expf((17.62f * t_c) / (243.12f + t_c));
}

/*
 * Aplica o offset de temperatura e recalcula a UR mantendo a mesma
 * quantidade de vapor (pressao de vapor constante).
 */
static void apply_temp_calibration(float t_raw, float rh_raw, float *t_out, float *rh_out)
{
    const float t_corr = t_raw - TEMP_OFFSET_C;
    float rh_corr = rh_raw;
    if (TEMP_OFFSET_C != 0.0f) {
        rh_corr = rh_raw * magnus_es(t_raw) / magnus_es(t_corr);
        if (rh_corr > 100.0f) rh_corr = 100.0f;
        if (rh_corr < 0.0f) rh_corr = 0.0f;
    }
    *t_out = t_corr;
    *rh_out = rh_corr;
}

static const char *fmt_val(char *buf, size_t sz, float v, const char *fmt)
{
    if (!is_valid(v)) {
        snprintf(buf, sz, "  ---");
    } else {
        snprintf(buf, sz, fmt, v);
    }
    return buf;
}

static const char *tag_ok(bool bad, bool valid, const char *bad_txt)
{
    static char out[4][48];
    static int k = 0;
    k = (k + 1) & 3;
    if (!valid) {
        snprintf(out[k], sizeof(out[k]), C_DIM "[ sem dado ]" C_RST);
    } else if (bad) {
        snprintf(out[k], sizeof(out[k]), C_RED C_BLD "[ %s ]" C_RST, bad_txt);
    } else {
        snprintf(out[k], sizeof(out[k]), C_GRN "[ OK ]" C_RST);
    }
    return out[k];
}

// --- MAIN TASK LOOP ---

static void sensor_loop_task(void *pvParameters)
{
    i2c_master_bus_handle_t bus_handle = (i2c_master_bus_handle_t)pvParameters;
    setvbuf(stdout, NULL, _IONBF, 0);

    initialize_hardware(bus_handle);

    while (1) {
        /* temp, humid, press, lux, mic, voc, nox, f1..f8, clear, nir */
        float sums[17] = {0};
        int   counts[17] = {0};

        for (int base_i = 0; base_i < BASE_REPEAT; ++base_i) {
            burst_hist_t m;
            env_flags_t flags;

            // 1. 10 s de amostragem
            execute_short_burst(base_i + 1, BASE_REPEAT, &m);

            // 2. Historico + flags (mesmas para terminal e MQTT)
            push_burst_to_history(&m);
            evaluate_flags(&m, &flags);
            print_burst_report(&m, &flags, base_i + 1, BASE_REPEAT);

            /*
             * Todas as 20 leituras deste burst ja terminaram. A task de
             * sensores fica bloqueada aqui ate o QoS 1 ser confirmado, de
             * forma que nenhuma nova transacao I2C comece durante o envio.
             */
            publish_burst_mean(&m, &flags, base_i + 1, BASE_REPEAT);

            // 3. Acumula para o ciclo longo (ignorando bursts sem dado)
            const float v[17] = {m.temp, m.humid, m.press, m.lux, m.mic_db, m.voc, m.nox,
                                 m.f1, m.f2, m.f3, m.f4, m.f5, m.f6, m.f7, m.f8, m.clear, m.nir};
            for (int i = 0; i < 17; ++i) {
                if (is_valid(v[i])) { sums[i] += v[i]; counts[i]++; }
            }

            // 4. Light sleep (desativado)
            // execute_light_sleep();
            // execute_hardware_reinit(bus_handle);
        }

        for (int i = 0; i < 17; ++i) {
            sums[i] = (counts[i] > 0) ? sums[i] / (float)counts[i] : INVALID_F;
        }
        process_long_cycle(sums);
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

    printf("\n" C_CYN LINE_EQ "\n"
           "  EnvSens - inicializando sensores (VCC no GPIO %d)\n"
           "  Norma: %s | T %.0f-%.0f C | UR %.0f-%.0f %% | offset T %.2f C\n"
           LINE_EQ C_RST "\n",
           SENSOR_PWR_PIN, ENV_NORM_NAME,
           FLAG_TEMP_MIN_C, FLAG_TEMP_MAX_C, FLAG_HUMID_MIN_PCT, FLAG_HUMID_MAX_PCT,
           TEMP_OFFSET_C);

    vTaskDelay(pdMS_TO_TICKS(100));

    const bool dps_ok = (dps310_init(bus_handle) == ESP_OK);
    s_sgp41_ready = (sgp41_init(bus_handle) == ESP_OK);
    const bool veml_ok = (veml7700_init(bus_handle) == ESP_OK);
    const bool as_ok = (as7341_init(bus_handle) == ESP_OK);
    const bool sht_ok = (sht40_init(bus_handle) == ESP_OK);

    printf("  Sensores: SHT40 %s | DPS368 %s | SGP41 %s | VEML7700 %s | AS7341 %s\n",
           sht_ok ? C_GRN "OK" C_RST : C_RED "FALHA" C_RST,
           dps_ok ? C_GRN "OK" C_RST : C_RED "FALHA" C_RST,
           s_sgp41_ready ? C_GRN "OK" C_RST : C_RED "FALHA" C_RST,
           veml_ok ? C_GRN "OK" C_RST : C_RED "FALHA" C_RST,
           as_ok ? C_GRN "OK" C_RST : C_RED "FALHA" C_RST);

    float t_dps = 25.0f;
    float p = 0.0f;
    uint16_t als_raw = 0;
    sht40_reading_t h = {
        .temperature = 25.0f,
        .humidity = 50.0f
    };

    vTaskDelay(pdMS_TO_TICKS(100));
    dps310_read(&t_dps, &p);
    const bool sht_first_ok = (sht40_read_data(&h) == ESP_OK);
    veml7700_read_als(&als_raw);

    /*
     * O NOx do SGP41 precisa de aproximadamente 10 s de conditioning.
     * Como GPIO19 permanece ligado, fazemos essa etapa uma unica vez.
     * Compensacao usa T/UR do SHT40 (recomendacao Sensirion).
     */
    if (s_sgp41_ready && !s_sgp41_conditioned) {
        printf("  SGP41: conditioning de 10 s ");

        bool conditioning_ok = true;
        for (int i = 0; i < 10; ++i) {
            uint16_t voc_raw = 0;
            risk_alert_sensor_gate_enter();
            esp_err_t err = sgp41_execute_conditioning(
                sht_first_ok ? h.humidity : 50.0f,
                sht_first_ok ? h.temperature : 25.0f,
                &voc_raw);
            risk_alert_sensor_gate_exit();

            if (err != ESP_OK) {
                conditioning_ok = false;
                printf(C_RED "x" C_RST);
            } else {
                printf(".");
            }

            // execute_conditioning consome aproximadamente 50 ms.
            vTaskDelay(pdMS_TO_TICKS(950));
        }

        s_sgp41_conditioned = conditioning_ok;
        printf(" %s\n", conditioning_ok ? C_GRN "OK" C_RST
                                        : C_YEL "com falhas (segue com retry)" C_RST);
    }

    s_last_sgp_update_us = 0;
    s_first_sgp_index_us = 0;
    s_sgp_ok_count = 0;
    printf(C_CYN LINE_EQ C_RST "\n");
    vTaskDelay(pdMS_TO_TICKS(100));
}

static void execute_hardware_reinit(i2c_master_bus_handle_t bus_handle)
{
    if (dps310_init(bus_handle) != ESP_OK) ESP_LOGE(TAG, "Falha DPS310 (reinit)");

    if (sgp41_init(bus_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Falha SGP41 (reinit)");
        s_sgp41_ready = false;
    } else {
        s_sgp41_ready = true;
    }

    if (veml7700_init(bus_handle) != ESP_OK) ESP_LOGE(TAG, "Falha VEML7700 (reinit)");
    if (as7341_init(bus_handle) != ESP_OK) ESP_LOGE(TAG, "Falha AS7341 (reinit)");
    if (sht40_init(bus_handle) != ESP_OK) ESP_LOGE(TAG, "Falha SHT40 (reinit)");

    s_last_sgp_update_us = 0;
    vTaskDelay(pdMS_TO_TICKS(50));
}

static float read_microphone_db(void)
{
    // O I2S e consumido exclusivamente pela task de captura do pipeline TinyML.
    return audio_ml_get_db();
}

static bool gas_index_ready(void)
{
    if (!s_sgp41_ready || s_first_sgp_index_us == 0) return false;
    /* Conditioning com falha nao bloqueia para sempre: depois de 10
     * leituras boas consideramos o sensor operando. */
    if (!s_sgp41_conditioned && s_sgp_ok_count < 10) return false;
    return (esp_timer_get_time() - s_first_sgp_index_us) >= (int64_t)GAS_INDEX_WARMUP_S * 1000000LL;
}

static void execute_short_burst(int burst_index, int total_repeats, burst_hist_t *out)
{
    float b_temp[BURST_SAMPLES], b_hum[BURST_SAMPLES];
    float b_temp_raw[BURST_SAMPLES], b_hum_raw[BURST_SAMPLES], b_temp_dps[BURST_SAMPLES];
    float b_press[BURST_SAMPLES], b_mic[BURST_SAMPLES], b_lux[BURST_SAMPLES];
    float b_voc[BURST_SAMPLES], b_nox[BURST_SAMPLES];
    float b_f[10][BURST_SAMPLES];   /* f1..f8, clear, nir */

    bool as_sat = false, veml_sat = false;

    printf("\n" C_BLD "BURST %d/%d" C_RST C_DIM "  (20 amostras, 10 s)" C_RST "\n",
           burst_index, total_repeats);
#if TERM_PRINT_EACH_SAMPLE
    printf(C_DIM "  #  | T(C)  | UR(%%) | T_dps | P(hPa)  |  Lux    | dB   | VOC | NOx | CLR    NIR%s" C_RST "\n",
           TERM_PRINT_SPECTRAL_SAMPLE ? "  | F1..F8" : "");
#endif

    TickType_t next_sample_tick = xTaskGetTickCount();

    for (int s = 0; s < BURST_SAMPLES; ++s) {
        float t_dps = 0.0f, press = 0.0f;
        uint16_t als_raw = 0;
        sht40_reading_t sht = {0};
        as7341_spectral_data_t sp = {0};

        /*
         * Portao dos sensores: se a IA detectar risco, a task de alerta fecha
         * o portao e esta leitura so comeca depois que o alerta for enviado.
         */
        risk_alert_sensor_gate_enter();

        const float current_db = read_microphone_db();

        const esp_err_t dps_ret = dps310_read(&t_dps, &press);
        const esp_err_t veml_ret = veml7700_read_als(&als_raw);
        const esp_err_t sht_ret = sht40_read_data(&sht);
        const esp_err_t as_ret = as7341_read_all_channels(&sp);

        /*
         * Os demais sensores continuam em 2 Hz.
         * O SGP41/Gas Index e atualizado a cada ~1 s.
         */
        const int64_t now_us = esp_timer_get_time();
        if (s_sgp41_ready &&
            (s_last_sgp_update_us == 0 ||
             (now_us - s_last_sgp_update_us) >= SGP_UPDATE_MIN_US)) {

            /* Compensacao com os valores LOCAIS do SHT40 (sem offset):
             * e o ar em volta do SGP41 que importa. */
            const float comp_h = (sht_ret == ESP_OK) ? sht.humidity : 50.0f;
            const float comp_t = (sht_ret == ESP_OK) ? sht.temperature
                               : (dps_ret == ESP_OK) ? t_dps : 25.0f;

            int32_t voc = s_last_voc_index;
            int32_t nox = s_last_nox_index;

            const esp_err_t sgp_err = sgp41_get_indices(comp_h, comp_t, &voc, &nox);

            if (sgp_err == ESP_OK) {
                s_last_voc_index = voc;
                s_last_nox_index = nox;
                s_last_sgp_update_us = now_us;
                if (s_first_sgp_index_us == 0) s_first_sgp_index_us = now_us;
                s_sgp_ok_count++;
            } else {
                ESP_LOGW(TAG, "SGP41 falhou; mantendo VOC=%" PRId32 " NOx=%" PRId32,
                         s_last_voc_index, s_last_nox_index);
            }
        }

        risk_alert_sensor_gate_exit();

        if (sht_ret == ESP_OK) {
            b_temp_raw[s] = sht.temperature;
            b_hum_raw[s]  = sht.humidity;
            apply_temp_calibration(sht.temperature, sht.humidity, &b_temp[s], &b_hum[s]);
        } else {
            b_temp_raw[s] = b_hum_raw[s] = b_temp[s] = b_hum[s] = INVALID_F;
        }
        b_temp_dps[s] = (dps_ret == ESP_OK) ? t_dps : INVALID_F;
        b_press[s]    = (dps_ret == ESP_OK) ? press : INVALID_F;
        b_lux[s]      = (veml_ret == ESP_OK) ? veml7700_raw_to_lux(als_raw) : INVALID_F;
        if (veml_ret == ESP_OK && als_raw >= 65535) veml_sat = true;
        b_mic[s]      = current_db;
        b_voc[s]      = (float)s_last_voc_index;
        b_nox[s]      = (float)s_last_nox_index;

        const uint16_t spv[10] = {sp.f1, sp.f2, sp.f3, sp.f4, sp.f5, sp.f6, sp.f7, sp.f8, sp.clear, sp.nir};
        for (int c = 0; c < 10; ++c) {
            b_f[c][s] = (as_ret == ESP_OK) ? (float)spv[c] : INVALID_F;
            if (as_ret == ESP_OK && (float)spv[c] >= AS7341_FULL_SCALE) as_sat = true;
        }

#if TERM_PRINT_EACH_SAMPLE
        char a[12], b[12], c[12], d[12], e[12];
        printf("  %02d | %5s | %5s | %5s | %7s | %7s | %4.1f | %3" PRId32 " | %3" PRId32 " | %5.0f %5.0f",
               s + 1,
               fmt_val(a, sizeof a, b_temp[s], "%5.2f"),
               fmt_val(b, sizeof b, b_hum[s], "%5.1f"),
               fmt_val(c, sizeof c, b_temp_dps[s], "%5.2f"),
               fmt_val(d, sizeof d, is_valid(b_press[s]) ? b_press[s] / 100.0f : INVALID_F, "%7.2f"),
               fmt_val(e, sizeof e, b_lux[s], "%7.1f"),
               current_db, s_last_voc_index, s_last_nox_index,
               is_valid(b_f[8][s]) ? b_f[8][s] : 0.0f,
               is_valid(b_f[9][s]) ? b_f[9][s] : 0.0f);
#if TERM_PRINT_SPECTRAL_SAMPLE
        printf("  | %.0f %.0f %.0f %.0f %.0f %.0f %.0f %.0f",
               b_f[0][s], b_f[1][s], b_f[2][s], b_f[3][s], b_f[4][s], b_f[5][s], b_f[6][s], b_f[7][s]);
#endif
        printf("\n");
#endif

        vTaskDelayUntil(&next_sample_tick, pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
    }

    out->temp      = mean_or_invalid(b_temp, BURST_SAMPLES);
    out->humid     = mean_or_invalid(b_hum, BURST_SAMPLES);
    out->temp_raw  = mean_or_invalid(b_temp_raw, BURST_SAMPLES);
    out->humid_raw = mean_or_invalid(b_hum_raw, BURST_SAMPLES);
    out->temp_dps  = mean_or_invalid(b_temp_dps, BURST_SAMPLES);
    out->press     = mean_or_invalid(b_press, BURST_SAMPLES);
    out->lux       = mean_or_invalid(b_lux, BURST_SAMPLES);
    out->mic_db    = stats_mean_f(b_mic, BURST_SAMPLES, INVALID_F);
    out->voc       = stats_mean_f(b_voc, BURST_SAMPLES, INVALID_F);
    out->nox       = stats_mean_f(b_nox, BURST_SAMPLES, INVALID_F);
    out->f1    = mean_or_invalid(b_f[0], BURST_SAMPLES);
    out->f2    = mean_or_invalid(b_f[1], BURST_SAMPLES);
    out->f3    = mean_or_invalid(b_f[2], BURST_SAMPLES);
    out->f4    = mean_or_invalid(b_f[3], BURST_SAMPLES);
    out->f5    = mean_or_invalid(b_f[4], BURST_SAMPLES);
    out->f6    = mean_or_invalid(b_f[5], BURST_SAMPLES);
    out->f7    = mean_or_invalid(b_f[6], BURST_SAMPLES);
    out->f8    = mean_or_invalid(b_f[7], BURST_SAMPLES);
    out->clear = mean_or_invalid(b_f[8], BURST_SAMPLES);
    out->nir   = mean_or_invalid(b_f[9], BURST_SAMPLES);
    out->n_sht  = (uint8_t)stats_count_valid_f(b_temp, BURST_SAMPLES, INVALID_F);
    out->n_dps  = (uint8_t)stats_count_valid_f(b_press, BURST_SAMPLES, INVALID_F);
    out->n_veml = (uint8_t)stats_count_valid_f(b_lux, BURST_SAMPLES, INVALID_F);
    out->n_as   = (uint8_t)stats_count_valid_f(b_f[8], BURST_SAMPLES, INVALID_F);
    out->as_saturated   = as_sat;
    out->veml_saturated = veml_sat;
    out->timestamp = (uint32_t)time(NULL);
    out->temp_variance     = stats_variance_f(b_temp, BURST_SAMPLES, INVALID_F);
    out->humidity_variance = stats_variance_f(b_hum, BURST_SAMPLES, INVALID_F);
}

static void push_burst_to_history(const burst_hist_t *new_burst)
{
    burst_buffer[burst_buffer_head] = *new_burst;
    burst_buffer_head = (burst_buffer_head + 1) % BURST_CIRCULAR_SIZE;
    if (burst_buffer_count < BURST_CIRCULAR_SIZE) burst_buffer_count++;
}

static void evaluate_flags(const burst_hist_t *m, env_flags_t *f)
{
    memset(f, 0, sizeof(*f));

    f->temp_valid  = is_valid(m->temp);
    f->humid_valid = is_valid(m->humid);
    f->press_valid = is_valid(m->press);
    f->lux_valid   = is_valid(m->lux);
    f->measurement_valid = f->temp_valid && f->humid_valid && f->press_valid && f->lux_valid;
    f->gas_ready   = gas_index_ready();
    f->audio_valid = (audio_ml_get_risk_status(&f->audio) == ESP_OK);

    /* Variacao entre bursts (historico) */
    float hist[BURST_CIRCULAR_SIZE];
    int n = 0;
    for (int i = 0; i < burst_buffer_count; ++i) {
        if (is_valid(burst_buffer[i].temp)) hist[n++] = burst_buffer[i].temp;
    }
    f->temp_stddev_hist = (n > 1) ? sqrtf(stats_variance_f(hist, n, INVALID_F)) : 0.0f;

    /* Cada flag depende so do proprio sensor (antes, falha no VEML
     * desligava as flags de temperatura e umidade). */
    f->temp_low   = f->temp_valid && m->temp < FLAG_TEMP_MIN_C;
    f->temp_high  = f->temp_valid && m->temp > FLAG_TEMP_MAX_C;
    f->temp_out_of_range = f->temp_low || f->temp_high;
    f->temp_variation    = (n > 1) && f->temp_stddev_hist > FLAG_TEMP_STDDEV_LIMIT_C;

    f->humid_low  = f->humid_valid && m->humid < FLAG_HUMID_MIN_PCT;
    f->humid_high = f->humid_valid && m->humid > FLAG_HUMID_MAX_PCT;
    f->humidity_out_of_range = f->humid_low || f->humid_high;

    f->dark = f->lux_valid && m->lux < FLAG_DARK_LUX;

    /* Antes do aquecimento o indice vale 0/100 fixo e gerava falso alarme */
    f->voc_elevated = f->gas_ready && m->voc >= FLAG_VOC_ELEVATED_INDEX;
    f->nox_elevated = f->gas_ready && m->nox >= FLAG_NOX_ELEVATED_INDEX;

    f->audio_risk = f->audio_valid && f->audio.risk_active;

    f->critical = f->temp_out_of_range || f->temp_variation ||
                  f->humidity_out_of_range || f->voc_elevated ||
                  f->nox_elevated || f->audio_risk;
}

static void print_burst_report(const burst_hist_t *m, const env_flags_t *f, int burst_index, int total_bursts)
{
    char v1[16], v2[16], v3[16];

    printf(C_CYN LINE_EQ C_RST "\n");
    printf(C_BLD " RESUMO BURST %d/%d" C_RST "   ciclo MQTT %" PRIu32 "   uptime %" PRIu32 " s   " C_DIM "(%s)" C_RST "\n",
           burst_index, total_bursts, s_mqtt_cycle_number + 1,
           (uint32_t)(esp_timer_get_time() / 1000000LL), ENV_NORM_NAME);
    printf(C_CYN LINE_DS C_RST "\n");

    printf("  Temperatura  %s C   sd %.2f  " C_DIM "(bruto %s | DPS %s)" C_RST "  faixa %.0f-%.0f  %s\n",
           fmt_val(v1, sizeof v1, m->temp, "%6.2f"),
           is_valid(m->temp) ? sqrtf(m->temp_variance) : 0.0f,
           fmt_val(v2, sizeof v2, m->temp_raw, "%.2f"),
           fmt_val(v3, sizeof v3, m->temp_dps, "%.2f"),
           FLAG_TEMP_MIN_C, FLAG_TEMP_MAX_C,
           tag_ok(f->temp_out_of_range, f->temp_valid, f->temp_low ? "FRIO" : "QUENTE"));

    printf("  Umidade      %s %%   sd %.2f  " C_DIM "(bruto %s)" C_RST "              faixa %.0f-%.0f  %s\n",
           fmt_val(v1, sizeof v1, m->humid, "%6.1f"),
           is_valid(m->humid) ? sqrtf(m->humidity_variance) : 0.0f,
           fmt_val(v2, sizeof v2, m->humid_raw, "%.1f"),
           FLAG_HUMID_MIN_PCT, FLAG_HUMID_MAX_PCT,
           tag_ok(f->humidity_out_of_range, f->humid_valid, f->humid_low ? "SECO" : "UMIDO"));

    printf("  Variacao T   %6.2f C (sd ultimos %d bursts)               limite %.1f  %s\n",
           f->temp_stddev_hist, burst_buffer_count, FLAG_TEMP_STDDEV_LIMIT_C,
           tag_ok(f->temp_variation, burst_buffer_count > 1, "PICO"));

    printf("  Pressao      %s hPa\n",
           fmt_val(v1, sizeof v1, is_valid(m->press) ? m->press / 100.0f : INVALID_F, "%7.2f"));

    printf("  Luz          %s lx%s                                      %s\n",
           fmt_val(v1, sizeof v1, m->lux, "%7.1f"),
           m->veml_saturated ? C_YEL " SATURADO" C_RST : "",
           !f->lux_valid ? C_DIM "[ sem dado ]" C_RST
                         : (f->dark ? C_YEL "[ ESCURO ]" C_RST : C_GRN "[ ACESO ]" C_RST));

    printf("  Som          %6.1f dB\n", m->mic_db);

    if (f->gas_ready) {
        printf("  VOC Index    %6.0f      (100 = media 24 h)           alerta >= %.0f  %s\n",
               m->voc, FLAG_VOC_ELEVATED_INDEX, tag_ok(f->voc_elevated, true, "ELEVADO"));
        printf("  NOx Index    %6.0f      (1 = linha de base)          alerta >= %.0f  %s\n",
               m->nox, FLAG_NOX_ELEVATED_INDEX, tag_ok(f->nox_elevated, true, "ELEVADO"));
    } else {
        printf("  VOC/NOx      %6.0f / %.0f " C_YEL "(aquecendo/aprendendo, flags desativadas)" C_RST "\n",
               m->voc, m->nox);
    }

    printf("  Espectro     F1 %.0f  F2 %.0f  F3 %.0f  F4 %.0f  F5 %.0f  F6 %.0f  F7 %.0f  F8 %.0f\n"
           "               CLR %.0f  NIR %.0f%s\n",
           m->f1, m->f2, m->f3, m->f4, m->f5, m->f6, m->f7, m->f8, m->clear, m->nir,
           m->as_saturated ? C_YEL "   SATURADO (reduza ganho do AS7341)" C_RST : "");

    if (f->audio_valid) {
        printf("  IA audio     p=%.3f  media=%.3f  votos %u/%u                     %s\n",
               f->audio.probability, f->audio.history_average,
               f->audio.positive_votes, f->audio.history_count,
               f->audio.risk_active ? C_RED C_BLD "[ RISCO ]" C_RST : C_GRN "[ NORMAL ]" C_RST);
    } else {
        printf("  IA audio     " C_DIM "sem status" C_RST "\n");
    }

    printf(C_DIM "  Leituras ok: SHT40 %u/%d  DPS368 %u/%d  VEML %u/%d  AS7341 %u/%d  SGP41 %s" C_RST "\n",
           m->n_sht, BURST_SAMPLES, m->n_dps, BURST_SAMPLES, m->n_veml, BURST_SAMPLES,
           m->n_as, BURST_SAMPLES,
           !s_sgp41_ready ? "falha" : (f->gas_ready ? "pronto" : "aquecendo"));

    /* Historico compacto */
    printf(C_CYN LINE_DS C_RST "\n");
    printf(C_DIM "  Historico   T(C)    UR(%%)   P(hPa)    Lux     dB    VOC  NOx" C_RST "\n");
    for (int i = 0; i < burst_buffer_count; ++i) {
        const int idx = (burst_buffer_head - burst_buffer_count + i + BURST_CIRCULAR_SIZE) % BURST_CIRCULAR_SIZE;
        const burst_hist_t *h = &burst_buffer[idx];
        char a[12], b[12], c[12], d[12];
        printf("  %s%d%s        %6s  %6s  %7s  %6s  %5.1f  %4.0f %4.0f\n",
               (i == burst_buffer_count - 1) ? C_BLD : "", i + 1, (i == burst_buffer_count - 1) ? " <" C_RST : "  ",
               fmt_val(a, sizeof a, h->temp, "%6.2f"),
               fmt_val(b, sizeof b, h->humid, "%6.1f"),
               fmt_val(c, sizeof c, is_valid(h->press) ? h->press / 100.0f : INVALID_F, "%7.1f"),
               fmt_val(d, sizeof d, h->lux, "%6.0f"),
               h->mic_db, h->voc, h->nox);
    }

    /* Status geral - exatamente as mesmas flags enviadas no MQTT */
    printf(C_CYN LINE_DS C_RST "\n");
    if (!f->critical) {
        printf("  STATUS: " C_GRN C_BLD "OK" C_RST "%s\n",
               f->measurement_valid ? "" : C_YEL "  (algum sensor sem dado)" C_RST);
    } else {
        printf("  STATUS: " C_RED C_BLD "CRITICO" C_RST " ->");
        if (f->temp_low)       printf(" TEMP_BAIXA");
        if (f->temp_high)      printf(" TEMP_ALTA");
        if (f->temp_variation) printf(" VARIACAO_TEMP");
        if (f->humid_low)      printf(" UMIDADE_BAIXA");
        if (f->humid_high)     printf(" UMIDADE_ALTA");
        if (f->voc_elevated)   printf(" VOC");
        if (f->nox_elevated)   printf(" NOX");
        if (f->audio_risk)     printf(" AUDIO_RISCO");
        printf("\n");
    }
    printf(C_CYN LINE_EQ C_RST "\n");
}

static void publish_burst_mean(
    const burst_hist_t *m,
    const env_flags_t *f,
    int burst_index,
    int total_bursts)
{
    if (m == NULL || f == NULL) {
        return;
    }

    ++s_mqtt_cycle_number;

    /* Valores invalidos vao como null no JSON */
    char t_s[16], h_s[16], p_s[16], l_s[16], td_s[16], tr_s[16];
#define JNUM(buf, v, fmt) (is_valid(v) ? (snprintf(buf, sizeof buf, fmt, (double)(v)), buf) : "null")

    char payload[MQTT_PAYLOAD_MAX_LEN];
    const int len = snprintf(
        payload,
        sizeof(payload),
        "{"
        "\"device\":\"esp32c6_environment\","
        "\"cycle\":%" PRIu32 ","
        "\"burst\":%d,"
        "\"total_bursts\":%d,"
        "\"samples\":%d,"
        "\"uptime_ms\":%" PRIu64 ","
        "\"temperature_c\":%s,"
        "\"humidity_pct\":%s,"
        "\"temperature_raw_c\":%s,"
        "\"temperature_dps_c\":%s,"
        "\"temp_offset_c\":%.2f,"
        "\"pressure_pa\":%s,"
        "\"lux\":%s,"
        "\"mic_db\":%.2f,"
        "\"voc_index\":%.1f,"
        "\"nox_index\":%.1f,"
        "\"f1\":%.0f,\"f2\":%.0f,\"f3\":%.0f,\"f4\":%.0f,"
        "\"f5\":%.0f,\"f6\":%.0f,\"f7\":%.0f,\"f8\":%.0f,"
        "\"clear\":%.0f,\"nir\":%.0f,"
        "\"audio_probability\":%.3f,"
        "\"audio_average\":%.3f,"
        "\"audio_votes\":%u,"
        "\"audio_history\":%u,"
        "\"audio_risk\":%s,"
        "\"norm\":\"%s\","
        "\"flags\":{"
        "\"measurement_valid\":%s,"
        "\"audio_status_valid\":%s,"
        "\"gas_sensor_ready\":%s,"
        "\"temperature_out_of_range\":%s,"
        "\"temperature_variation\":%s,"
        "\"humidity_out_of_range\":%s,"
        "\"dark\":%s,"
        "\"voc_elevated\":%s,"
        "\"nox_elevated\":%s,"
        "\"audio_risk\":%s,"
        "\"critical\":%s"
        "}"
        "}",
        s_mqtt_cycle_number,
        burst_index,
        total_bursts,
        BURST_SAMPLES,
        (uint64_t)(esp_timer_get_time() / 1000LL),
        JNUM(t_s, m->temp, "%.2f"),
        JNUM(h_s, m->humid, "%.2f"),
        JNUM(tr_s, m->temp_raw, "%.2f"),
        JNUM(td_s, m->temp_dps, "%.2f"),
        TEMP_OFFSET_C,
        JNUM(p_s, m->press, "%.2f"),
        JNUM(l_s, m->lux, "%.2f"),
        m->mic_db,
        m->voc,
        m->nox,
        is_valid(m->f1) ? m->f1 : 0.0f, is_valid(m->f2) ? m->f2 : 0.0f,
        is_valid(m->f3) ? m->f3 : 0.0f, is_valid(m->f4) ? m->f4 : 0.0f,
        is_valid(m->f5) ? m->f5 : 0.0f, is_valid(m->f6) ? m->f6 : 0.0f,
        is_valid(m->f7) ? m->f7 : 0.0f, is_valid(m->f8) ? m->f8 : 0.0f,
        is_valid(m->clear) ? m->clear : 0.0f,
        is_valid(m->nir) ? m->nir : 0.0f,
        f->audio_valid ? f->audio.probability : 0.0f,
        f->audio_valid ? f->audio.history_average : 0.0f,
        f->audio_valid ? f->audio.positive_votes : 0U,
        f->audio_valid ? f->audio.history_count : 0U,
        f->audio_risk ? "true" : "false",
        ENV_NORM_NAME,
        f->measurement_valid ? "true" : "false",
        f->audio_valid ? "true" : "false",
        f->gas_ready ? "true" : "false",
        f->temp_out_of_range ? "true" : "false",
        f->temp_variation ? "true" : "false",
        f->humidity_out_of_range ? "true" : "false",
        f->dark ? "true" : "false",
        f->voc_elevated ? "true" : "false",
        f->nox_elevated ? "true" : "false",
        f->audio_risk ? "true" : "false",
        f->critical ? "true" : "false");
#undef JNUM

    if (len < 0 || (size_t)len >= sizeof(payload)) {
        ESP_LOGE(TAG, "Payload MQTT excedeu o buffer de %u bytes", (unsigned)sizeof(payload));
        return;
    }

    if (!mqtt_is_connected()) {
        printf(C_YEL "  MQTT: desconectado, ciclo %" PRIu32 " nao enviado" C_RST "\n",
               s_mqtt_cycle_number);
        return;
    }

    ESP_LOGD(TAG, "Pausando sensores e audio para envio; ciclo=%" PRIu32, s_mqtt_cycle_number);

    const esp_err_t pause_err = audio_ml_pause(AUDIO_PAUSE_TIMEOUT_MS);
    if (pause_err != ESP_OK) {
        printf(C_YEL "  MQTT: envio cancelado (audio nao pausou: %s)" C_RST "\n",
               esp_err_to_name(pause_err));
        return;
    }

    /* Janela silenciosa depois da ultima transacao de sensor. */
    vTaskDelay(pdMS_TO_TICKS(MQTT_PRE_TX_QUIET_MS));

    const int64_t t0 = esp_timer_get_time();
    int msg_id = -1;
    const esp_err_t err = mqtt_publish_sensor_data(payload, (size_t)len, MQTT_PUBLISH_TIMEOUT_MS, &msg_id);
    const int64_t dt_ms = (esp_timer_get_time() - t0) / 1000LL;

    if (err == ESP_OK) {
        printf(C_GRN "  MQTT: ciclo %" PRIu32 " enviado e confirmado" C_RST C_DIM
               " (msg_id=%d, %" PRId64 " ms, %d bytes)" C_RST "\n",
               s_mqtt_cycle_number, msg_id, dt_ms, len);
    } else {
        printf(C_RED "  MQTT: falha no ciclo %" PRIu32 ": %s" C_RST "\n",
               s_mqtt_cycle_number, esp_err_to_name(err));
    }

    /* Margem para o radio encerrar os ultimos pacotes antes das leituras. */
    vTaskDelay(pdMS_TO_TICKS(MQTT_POST_TX_GUARD_MS));
    audio_ml_resume();
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

static void process_long_cycle(const float *avg)
{
    longa_t L = {
        .temp = avg[0], .humid = avg[1], .press = avg[2], .lux = avg[3],
        .mic_db = avg[4], .voc = avg[5], .nox = avg[6],
        .f1 = avg[7], .f2 = avg[8], .f3 = avg[9], .f4 = avg[10],
        .f5 = avg[11], .f6 = avg[12], .f7 = avg[13], .f8 = avg[14],
        .clear = avg[15], .nir = avg[16],
        .ts = (uint32_t)time(NULL),
    };

    longa_buffer[buffer_head] = L;
    buffer_head = (buffer_head + 1) % CIRCULAR_SIZE;
    if (buffer_count < CIRCULAR_SIZE) buffer_count++;

    float temp_buf[CIRCULAR_SIZE];
    for (int i = 0; i < buffer_count; ++i) {
        temp_buf[i] = longa_buffer[i].temp;
    }
    const float var_temp_buf = stats_variance_f(temp_buf, buffer_count, INVALID_F);

    char a[16], b[16], c[16], d[16];
    printf("\n" C_CYN LINE_EQ "\n" C_RST C_BLD " CICLO LONGO" C_RST "  media de %d bursts (~%d s)\n",
           BASE_REPEAT, BASE_REPEAT * BURST_DURATION_MS / 1000);
    printf("  T %s C | UR %s %% | P %s hPa | Lux %s | %5.1f dB | VOC %.0f | NOx %.0f\n",
           fmt_val(a, sizeof a, L.temp, "%.2f"),
           fmt_val(b, sizeof b, L.humid, "%.1f"),
           fmt_val(c, sizeof c, is_valid(L.press) ? L.press / 100.0f : INVALID_F, "%.2f"),
           fmt_val(d, sizeof d, L.lux, "%.1f"),
           is_valid(L.mic_db) ? L.mic_db : 0.0f,
           is_valid(L.voc) ? L.voc : 0.0f,
           is_valid(L.nox) ? L.nox : 0.0f);
    printf("  Ultimos %d ciclos longos: sd T = %.3f C\n", buffer_count, sqrtf(var_temp_buf));
    printf(C_CYN LINE_EQ C_RST "\n");
}

esp_err_t sensor_task_start(i2c_master_bus_handle_t bus_handle)
{
    BaseType_t res = xTaskCreate(sensor_loop_task, "sensor_task", SENSOR_TASK_STACK_SIZE, bus_handle, SENSOR_TASK_PRIORITY, NULL);
    return (res == pdPASS) ? ESP_OK : ESP_FAIL;
}
