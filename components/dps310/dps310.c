/*
    Made by Igor Jensen - UFES - LAEEC
    20/11/2025
*/

#include "dps310.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DPS310";   // Tag para logs
static i2c_master_dev_handle_t dps_handle; // Handle do dispositivo I2C
static dps310_calib_data_t calib;   // Coeficientes de calibração

// Endereço e registradores do sensor
#define DPS310_ADDR          0x76
#define I2C_TIMEOUT_MS       1000

#define DPS310_REG_COEF      0x10
#define DPS310_REG_PSR_CFG   0x06
#define DPS310_REG_TMP_CFG   0x07
#define DPS310_REG_MEAS_CFG  0x08
#define DPS310_REG_TMP_B2    0x03
#define DPS310_REG_PSR_B2    0x00
#define DPS310_REG_PROD_ID   0x0D

// Lê bytes de um registrador
static esp_err_t dps_read_reg(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(dps_handle, &reg, 1, data, len, I2C_TIMEOUT_MS);
}

// Escreve em um registrador
static esp_err_t dps_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    return i2c_master_transmit(dps_handle, buf, 2, I2C_TIMEOUT_MS);
}

// Lê valor bruto de 24 bits (temperatura/pressão)
static int32_t dps_read_raw24(uint8_t reg_base) {
    uint8_t buf[3];
    if (dps_read_reg(reg_base, buf, 3) != ESP_OK) return 0;

    int32_t val = (int32_t)((buf[0] << 16) | (buf[1] << 8) | buf[2]);

    if (val & 0x800000) val |= 0xFF000000; // Extensão de sinal

    return val;
}

// Lê coeficientes de calibração do sensor
static void dps_read_calibration(void) {
    uint8_t buf[18];
    if (dps_read_reg(DPS310_REG_COEF, buf, 18) != ESP_OK) {
        ESP_LOGE(TAG, "Erro ao ler calibração");
        return;
    }

    // Conversão conforme datasheet
    calib.c0 = ((buf[0] << 4) | (buf[1] >> 4)) & 0x0FFF;
    if (calib.c0 & 0x0800) calib.c0 |= 0xF000;

    calib.c1 = ((buf[1] & 0x0F) << 8) | buf[2];
    if (calib.c1 & 0x0800) calib.c1 |= 0xF000;

    calib.c00 = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | ((buf[5] & 0xF0) >> 4);
    if (calib.c00 & 0x80000) calib.c00 |= 0xFFF00000;

    calib.c10 = ((int32_t)(buf[5] & 0x0F) << 16) | ((int32_t)buf[6] << 8) | buf[7];
    if (calib.c10 & 0x80000) calib.c10 |= 0xFFF00000;

    calib.c01 = (int16_t)((buf[8] << 8) | buf[9]);
    calib.c11 = (int16_t)((buf[10] << 8) | buf[11]);
    calib.c20 = (int16_t)((buf[12] << 8) | buf[13]);
    calib.c21 = (int16_t)((buf[14] << 8) | buf[15]);
    calib.c30 = (int16_t)((buf[16] << 8) | buf[17]);
}

// Compensa temperatura usando calibração
static float dps_compensate_temp(int32_t raw) {
    return calib.c0 * 0.5f + calib.c1 * raw / 524288.0f;
}

// Compensa pressão usando calibração
static float dps_compensate_pressure(int32_t raw_p, int32_t raw_t) {
    float t = raw_t / 524288.0f;
    float p = raw_p / 1572864.0f;
    return calib.c00 +
        p * (calib.c10 + p * (calib.c20 + p * calib.c30)) +
        t * (calib.c01 + p * (calib.c11 + p * calib.c21));
}

// Inicializa o sensor DPS310
esp_err_t dps310_init(i2c_master_bus_handle_t bus) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DPS310_ADDR,
        .scl_speed_hz = 400000, // 400kHz
    };

    // Adiciona dispositivo ao barramento
    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &dps_handle);
    if (err != ESP_OK) return err;

    // Verifica ID do sensor
    uint8_t prod_id = 0;
    err = dps_read_reg(DPS310_REG_PROD_ID, &prod_id, 1);
    if (err != ESP_OK || prod_id != 0x10) {
        ESP_LOGE(TAG, "DPS310 ID inválido: 0x%02X", prod_id);
        return ESP_FAIL;
    }

    // Lê calibração interna
    dps_read_calibration();

    // Configura modo contínuo temperatura + pressão
    dps_write_reg(DPS310_REG_TMP_CFG, 0x80); // Temp oversampling
    dps_write_reg(DPS310_REG_PSR_CFG, 0x01); // Press oversampling
    dps_write_reg(DPS310_REG_MEAS_CFG, 0x07); // Modo contínuo

    return ESP_OK;
}

// Lê temperatura e pressão compensadas
esp_err_t dps310_read(float *temperature, float *pressure) {
    if (!temperature || !pressure) return ESP_ERR_INVALID_ARG;

    int32_t raw_temp = dps_read_raw24(DPS310_REG_TMP_B2); // Temp bruta
    int32_t raw_pres = dps_read_raw24(DPS310_REG_PSR_B2); // Pressão bruta

    *temperature = dps_compensate_temp(raw_temp);
    *pressure = dps_compensate_pressure(raw_pres, raw_temp);

    return ESP_OK;
}
