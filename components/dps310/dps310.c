#include "dps310.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DPS368";
static i2c_master_dev_handle_t dps_handle;
static dps310_calib_data_t calib;

#define DPS368_ADDR          0x77
#define I2C_TIMEOUT_MS       1000

// Registradores conforme Datasheet
#define REG_PSR_B2           0x00
#define REG_TMP_B2           0x03
#define REG_PSR_CFG          0x06
#define REG_TMP_CFG          0x07
#define REG_MEAS_CFG         0x08
#define REG_CFG_REG          0x09
#define REG_RESET            0x0C
#define REG_PROD_ID          0x0D
#define REG_COEF             0x10
#define REG_COEF_SRCE        0x28

// Oversampling 16x (Standard)
#define OVERSAMPLING_16X     0x04 
static const float k_factor = 253952.0f; // Fator para 16x (Tabela 9)

static esp_err_t dps_read_reg(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(dps_handle, &reg, 1, data, len, I2C_TIMEOUT_MS);
}

static esp_err_t dps_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    return i2c_master_transmit(dps_handle, buf, 2, I2C_TIMEOUT_MS);
}

static int32_t dps_read_raw24(uint8_t reg_base) {
    uint8_t buf[3];
    if (dps_read_reg(reg_base, buf, 3) != ESP_OK) return 0;
    // Dados sao 24-bit 2's complement
    int32_t val = (int32_t)((buf[0] << 16) | (buf[1] << 8) | buf[2]);
    if (val & 0x800000) val |= 0xFF000000; 
    return val;
}

static void dps_read_calibration(void) {
    uint8_t buf[18];
    // Espera os coeficientes estarem prontos (bit 7 de 0x08)
    uint8_t ready = 0;
    while (!(ready & 0x80)) {
        dps_read_reg(REG_MEAS_CFG, &ready, 1);
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (dps_read_reg(REG_COEF, buf, 18) != ESP_OK) {
        ESP_LOGE(TAG, "Erro ao ler coeficientes");
        return;
    }

    // Extracao conforme Tabela 18 do Datasheet
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
    
    ESP_LOGI(TAG, "Coeficientes de calibracao lidos com sucesso.");
}

esp_err_t dps310_init(i2c_master_bus_handle_t bus) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DPS368_ADDR,
        .scl_speed_hz = 400000,
    };

    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &dps_handle);
    if (err != ESP_OK) return err;

    dps_write_reg(REG_RESET, 0x09); // Soft Reset
    vTaskDelay(pdMS_TO_TICKS(40));  // Tempo de startup (Tabela 8)

    // Verifica ID (deve ser 0x10 para DPS368)
    uint8_t prod_id = 0;
    dps_read_reg(REG_PROD_ID, &prod_id, 1);
    if ((prod_id & 0x0F) != 0x00) { // PROD_ID sao os bits 3:0
        ESP_LOGW(TAG, "Aviso: ID esperado 0x10, recebido 0x%02X", prod_id);
    }

    // Descobrir fonte da temperatura para a calibracao (Reg 0x28)
    uint8_t temp_src = 0;
    dps_read_reg(REG_COEF_SRCE, &temp_src, 1);
    temp_src &= 0x80; // Pega apenas o bit 7 (TMP_COEF_SRCE)

    dps_read_calibration();

    // 1. Configurar Oversampling em 0x06 e 0x07 (SEM bit de shift aqui!)
    dps_write_reg(REG_PSR_CFG, OVERSAMPLING_16X); 
    dps_write_reg(REG_TMP_CFG, OVERSAMPLING_16X | temp_src); // Usa a mesma fonte da calibracao

    // 2. Configurar o SHIFT no registrador correto: CFG_REG (0x09)
    // Bit 2: P_SHIFT, Bit 3: T_SHIFT. Valor 0x0C ativa ambos.
    dps_write_reg(REG_CFG_REG, 0x0C); 

    // 3. Iniciar modo continuo (Background Mode)
    dps_write_reg(REG_MEAS_CFG, 0x07); 

    ESP_LOGI(TAG, "DPS368 inicializado (16x, Shift ativo, Src: %s)", 
             temp_src ? "Externo" : "Interno");
    return ESP_OK;
}

esp_err_t dps310_read(float *temperature, float *pressure) {
    if (!temperature || !pressure) return ESP_ERR_INVALID_ARG;

    int32_t raw_t = dps_read_raw24(REG_TMP_B2);
    int32_t raw_p = dps_read_raw24(REG_PSR_B2);

    // Escalonamento (Tabela 9)
    float sc_t = (float)raw_t / k_factor;
    float sc_p = (float)raw_p / k_factor;

    // Compensacao de Temperatura (Secao 4.9.2)
    *temperature = (float)calib.c0 * 0.5f + (float)calib.c1 * sc_t;
    
    // Compensacao de Pressao (Secao 4.9.1)
    *pressure = (float)calib.c00 +
                sc_p * ((float)calib.c10 + sc_p * ((float)calib.c20 + sc_p * (float)calib.c30)) +
                sc_t * (float)calib.c01 +
                sc_t * sc_p * ((float)calib.c11 + sc_p * (float)calib.c21);

    return ESP_OK;
}