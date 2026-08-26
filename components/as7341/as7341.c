/*
    Made by Igor Jensen - UFES - LAEEC
    20/11/2025
*/

#include "as7341.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// // Tags para logs
// static const char *TAG = "AS7341";

#define AS7341_REG_ENABLE       0x80
#define AS7341_REG_ATIME        0x81
#define AS7341_REG_ASTEP_L      0xCA
#define AS7341_REG_ASTEP_H      0xCB
#define AS7341_REG_CFG1         0xAA
#define AS7341_REG_CFG0         0xA9
#define AS7341_REG_CFG8         0xB1
#define AS7341_REG_CFG12        0xB5
#define AS7341_REG_STATUS       0x93
#define AS7341_REG_CH0_DATA_L   0x95
#define AS7341_REG_CH0_DATA_H   0x96
#define AS7341_REG_CH1_DATA_L   0x97
#define AS7341_REG_CH1_DATA_H   0x98
#define AS7341_REG_CH2_DATA_L   0x99
#define AS7341_REG_CH2_DATA_H   0x9A
#define AS7341_REG_CH3_DATA_L   0x9B
#define AS7341_REG_CH3_DATA_H   0x9C
#define AS7341_REG_CH4_DATA_L   0x9D
#define AS7341_REG_CH4_DATA_H   0x9E
#define AS7341_REG_CH5_DATA_L   0x9F
#define AS7341_REG_CH5_DATA_H   0xA0
#define AS7341_REG_SMUX_CFG_0   0x00
#define AS7341_REG_CONTROL      0xFA


#include "as7341.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "AS7341";

/* Diagnostic dump forward declarations (placed early to ensure visibility) */
static void as7341_dump_regs(void);
static void as7341_dump_smux(void);

/* =========================================================
 * REGISTRADORES
 * ========================================================= */

#define AS7341_REG_ENABLE       0x80
#define AS7341_REG_ATIME        0x81
#define AS7341_REG_CFG0         0xA9
#define AS7341_REG_CFG1         0xAA
#define AS7341_REG_STATUS2      0xA3

#define AS7341_REG_CH0_DATA_L   0x95
#define AS7341_REG_CH1_DATA_L   0x97
#define AS7341_REG_CH2_DATA_L   0x99
#define AS7341_REG_CH3_DATA_L   0x9B
#define AS7341_REG_CH4_DATA_L   0x9D
#define AS7341_REG_CH5_DATA_L   0x9F

#define AS7341_REG_ASTEP_L      0xCA
#define AS7341_REG_ASTEP_H      0xCB

#define AS7341_REG_ID           0x92

#define AS7341_REG_CONTROL      0xFA

/* =========================================================
 * BITS
 * ========================================================= */

#define AS7341_ENABLE_PON       (1 << 0)
#define AS7341_ENABLE_SP_EN     (1 << 1)
#define AS7341_ENABLE_SMUXEN    (1 << 4)

#define AS7341_CFG0_BANK        (1 << 6)

#define AS7341_STATUS2_AVALID   (1 << 6)

/* CONTROL bits */
#define AS7341_CONTROL_SMUX_CMD  (1 << 3)
#define AS7341_CONTROL_ADC_INIT  (1 << 0)

/* =========================================================
 * SMUX CONFIG OFICIAL
 * ========================================================= */

static const uint8_t smux_f1_f4[20] = {
    0x30, 0x01, 0x00, 0x00,
    0x00, 0x42, 0x00, 0x00,
    0x50, 0x00, 0x00, 0x00,
    0x20, 0x04, 0x00, 0x30,
    0x01, 0x50, 0x00, 0x06
};

static const uint8_t smux_f5_f8[20] = {
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x50,
    0x00, 0x30, 0x02, 0x00,
    0x00, 0x00, 0x40, 0x02,
    0x00, 0x00, 0x10, 0x03
};

/* =========================================================
 * HANDLE
 * ========================================================= */

static i2c_master_dev_handle_t as7341_handle = NULL;

/* =========================================================
 * LOW LEVEL
 * ========================================================= */

static esp_err_t as7341_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};

    return i2c_master_transmit(
        as7341_handle,
        data,
        2,
        1000
    );
}

static esp_err_t as7341_read_reg(uint8_t reg, uint8_t *value)
{
    return i2c_master_transmit_receive(
        as7341_handle,
        &reg,
        1,
        value,
        1,
        1000
    );
}

static esp_err_t as7341_read_reg16(uint8_t reg, uint16_t *value)
{
    uint8_t buf[2];

    esp_err_t ret = i2c_master_transmit_receive(
        as7341_handle,
        &reg,
        1,
        buf,
        2,
        1000
    );

    if (ret == ESP_OK) {
        *value = ((uint16_t)buf[1] << 8) | buf[0];
    }

    return ret;
}

/* (forward declaration moved earlier) */

/* =========================================================
 * ENABLE
 * ========================================================= */

static esp_err_t as7341_set_enable(
    bool pon,
    bool spectral,
    bool smux)
{
    uint8_t val = 0;

    if (pon)
        val |= AS7341_ENABLE_PON;

    if (spectral)
        val |= AS7341_ENABLE_SP_EN;

    if (smux)
        val |= AS7341_ENABLE_SMUXEN;

    return as7341_write_reg(
        AS7341_REG_ENABLE,
        val
    );
}

/* =========================================================
 * WAIT SMUX
 * ========================================================= */

static esp_err_t as7341_wait_smux(void)
{
    uint8_t enable;

    for (int i = 0; i < 200; i++) {

        if (as7341_read_reg(
                AS7341_REG_ENABLE,
                &enable) != ESP_OK) {
            ESP_LOGE(TAG, "as7341_wait_smux: falha leitura ENABLE");
            return ESP_FAIL;
        }

        if (!(enable & AS7341_ENABLE_SMUXEN)) {
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }

    ESP_LOGE(TAG, "as7341_wait_smux: timeout, ENABLE=0x%02X", enable);
    return ESP_ERR_TIMEOUT;
}

/* =========================================================
 * WAIT DATA
 * ========================================================= */

static esp_err_t as7341_wait_data(void)
{
    uint8_t status;

    for (int i = 0; i < 200; i++) {

        if (as7341_read_reg(
                AS7341_REG_STATUS2,
                &status) != ESP_OK) {
            ESP_LOGE(TAG, "as7341_wait_data: falha leitura STATUS2");
            return ESP_FAIL;
        }

        if (status & AS7341_STATUS2_AVALID) {
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGE(TAG, "as7341_wait_data: timeout, STATUS2=0x%02X", status);
    /* Dump key registers for diagnostics (compare to datasheet) */
    as7341_dump_regs();
    as7341_dump_smux();
    return ESP_ERR_TIMEOUT;
}

// Diagnostic dump of key registers
static void as7341_dump_regs(void)
{
   }

/* Read back SMUX registers (0..19) by switching to bank 1 temporarily */
static void as7341_dump_smux(void)
{
    uint8_t orig_cfg0;
    if (as7341_read_reg(AS7341_REG_CFG0, &orig_cfg0) != ESP_OK) {
        ESP_LOGW(TAG, "as7341_dump_smux: failed read CFG0");
        return;
    }

    /* switch to bank 1 */
    if (as7341_write_reg(AS7341_REG_CFG0, orig_cfg0 | AS7341_CFG0_BANK) != ESP_OK) {
        ESP_LOGW(TAG, "as7341_dump_smux: failed set BANK=1");
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(1));

    for (int i = 0; i < 20; ++i) {
        uint8_t v = 0;
        if (as7341_read_reg((uint8_t)i, &v) == ESP_OK) {
            ESP_LOGI(TAG, "SMUX[%02d]=0x%02X", i, v);
        } else {
            ESP_LOGW(TAG, "SMUX[%02d] read fail", i);
        }
    }

    /* restore bank */
    if (as7341_write_reg(AS7341_REG_CFG0, orig_cfg0 & ~AS7341_CFG0_BANK) != ESP_OK) {
        ESP_LOGW(TAG, "as7341_dump_smux: failed restore BANK=0");
    }
    vTaskDelay(pdMS_TO_TICKS(1));
}

/* =========================================================
 * LOAD SMUX
 * ========================================================= */

static esp_err_t as7341_load_smux(
    const uint8_t *config)
{
    uint8_t cfg0;

    ESP_ERROR_CHECK(
        as7341_read_reg(
            AS7341_REG_CFG0,
            &cfg0)
    );

    /* BANK = 1 */
    ESP_ERROR_CHECK(
        as7341_write_reg(
            AS7341_REG_CFG0,
            cfg0 | AS7341_CFG0_BANK)
    );

    vTaskDelay(pdMS_TO_TICKS(1));

    for (int i = 0; i < 20; i++) {

        ESP_ERROR_CHECK(
            as7341_write_reg(
                i,
                config[i])
        );
    }

    /* BANK = 0 */
    ESP_ERROR_CHECK(
        as7341_write_reg(
            AS7341_REG_CFG0,
            cfg0 & ~AS7341_CFG0_BANK)
    );

    vTaskDelay(pdMS_TO_TICKS(1));



    return ESP_OK;
}

/* =========================================================
 * START MEASUREMENT
 * ========================================================= */

static esp_err_t as7341_start_measurement(
    const uint8_t *smux)
{
   ESP_ERROR_CHECK(as7341_set_enable(true, false, false));
    vTaskDelay(pdMS_TO_TICKS(1));

    /* Load SMUX na RAM */
    ESP_ERROR_CHECK(as7341_load_smux(smux));

    /* 1. Ativa o bit SMUXEN primeiro! */
    ESP_ERROR_CHECK(as7341_set_enable(true, false, true));

    /* 2. AGORA SIM, envia o comando de Apply SMUX */
    ESP_ERROR_CHECK(as7341_write_reg(AS7341_REG_CONTROL, AS7341_CONTROL_SMUX_CMD));

    /* Wait SMUX (vai esperar o SMUXEN voltar a 0) */
    esp_err_t ret = as7341_wait_smux();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SMUX timeout");
        return ret;
    }

    /* Start spectral measurement */
    ESP_ERROR_CHECK(as7341_set_enable(true, true, false));
    vTaskDelay(pdMS_TO_TICKS(1));

    /* Kick ADC conversion */
    ESP_ERROR_CHECK(as7341_write_reg(AS7341_REG_CONTROL, AS7341_CONTROL_ADC_INIT));



    /* Wait integration time */
    vTaskDelay(pdMS_TO_TICKS(50));

    return ESP_OK;
}

/* =========================================================
 * INIT
 * ========================================================= */
esp_err_t as7341_init(i2c_master_bus_handle_t i2c_bus) 
{
    // If handle exists, remove it first to allow clean re-registration after deep/light sleep routine
    if (as7341_handle != NULL) {
        i2c_master_bus_rm_device(as7341_handle);
        as7341_handle = NULL;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS7341_I2C_ADDR,
        .scl_speed_hz = 400000,
    };

    esp_err_t ret = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &as7341_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao adicionar dispositivo I2C: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(300));

    uint8_t id = 0;
    ret = as7341_read_reg(AS7341_REG_ID, &id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erro leitura ID");
        return ret;
    }

    if (id != 0x24) {
        ESP_LOGE(TAG, "Sensor nao encontrado (ID=0x%02X)", id);
        return ESP_FAIL;
    }

    /* Force hardware Power ON and register setup on every init invocation */
    as7341_set_enable(true, false, false);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Apply integration time and gain profiles to unbrick measurements */
    as7341_write_reg(AS7341_REG_ATIME, 29);
    as7341_write_reg(AS7341_REG_ASTEP_L, 0xE7);
    as7341_write_reg(AS7341_REG_ASTEP_H, 0x03);
    as7341_write_reg(AS7341_REG_CFG1, 0x06); // Gain 64x

    ESP_LOGI(TAG, "AS7341 inicializado e registrado com sucesso");
    return ESP_OK;
}

/* =========================================================
 * READ ALL CHANNELS
 * ========================================================= */

esp_err_t as7341_read_all_channels(
    as7341_spectral_data_t *data)
{
    esp_err_t ret;

    /* =====================================================
     * F1-F4
     * ===================================================== */

    ret = as7341_start_measurement(smux_f1_f4);

    if (ret != ESP_OK) {
        return ret;
    }

    ret = as7341_wait_data();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Timeout F1-F4");
        return ret;
    }

    if (as7341_read_reg16(AS7341_REG_CH0_DATA_L, &data->f1) != ESP_OK) { ESP_LOGE(TAG, "Erro leitura f1"); return ESP_FAIL; }
    if (as7341_read_reg16(AS7341_REG_CH1_DATA_L, &data->f2) != ESP_OK) { ESP_LOGE(TAG, "Erro leitura f2"); return ESP_FAIL; }
    if (as7341_read_reg16(AS7341_REG_CH2_DATA_L, &data->f3) != ESP_OK) { ESP_LOGE(TAG, "Erro leitura f3"); return ESP_FAIL; }
    if (as7341_read_reg16(AS7341_REG_CH3_DATA_L, &data->f4) != ESP_OK) { ESP_LOGE(TAG, "Erro leitura f4"); return ESP_FAIL; }

    /* =====================================================
     * F5-F8
     * ===================================================== */

    ret = as7341_start_measurement(smux_f5_f8);

    if (ret != ESP_OK) {
        return ret;
    }

    ret = as7341_wait_data();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Timeout F5-F8");
        return ret;
    }

    if (as7341_read_reg16(AS7341_REG_CH0_DATA_L, &data->f5) != ESP_OK) { ESP_LOGE(TAG, "Erro leitura f5"); return ESP_FAIL; }
    if (as7341_read_reg16(AS7341_REG_CH1_DATA_L, &data->f6) != ESP_OK) { ESP_LOGE(TAG, "Erro leitura f6"); return ESP_FAIL; }
    if (as7341_read_reg16(AS7341_REG_CH2_DATA_L, &data->f7) != ESP_OK) { ESP_LOGE(TAG, "Erro leitura f7"); return ESP_FAIL; }
    if (as7341_read_reg16(AS7341_REG_CH3_DATA_L, &data->f8) != ESP_OK) { ESP_LOGE(TAG, "Erro leitura f8"); return ESP_FAIL; }

    if (as7341_read_reg16(AS7341_REG_CH4_DATA_L, &data->clear) != ESP_OK) { ESP_LOGE(TAG, "Erro leitura clear"); return ESP_FAIL; }
    if (as7341_read_reg16(AS7341_REG_CH5_DATA_L, &data->nir) != ESP_OK) { ESP_LOGE(TAG, "Erro leitura nir"); return ESP_FAIL; }

    /* STOP spectral */
    as7341_set_enable(
        true,
        false,
        false
    );

    return ESP_OK;
}

/* =========================================================
 * DEINIT
 * ========================================================= */

esp_err_t as7341_deinit(void)
{
    if (as7341_handle != NULL) {

        i2c_master_bus_rm_device(
            as7341_handle
        );

        as7341_handle = NULL;
    }

    return ESP_OK;
}