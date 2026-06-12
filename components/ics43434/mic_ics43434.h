#ifndef MIC_ICS43434_H_
#define MIC_ICS43434_H_

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

// Definição dos pinos GPIO para o microfone ICS-43434
#define MIC_DATA_PIN  5  // SD (Serial Data)
#define MIC_BCLK_PIN  6  // SCK (Continuous Serial Clock)
#define MIC_WS_PIN    7  // WS (Word Select / LRCLK)

// Inicializa o periférico I2S e os pinos do microfone
esp_err_t mic_ics43434_init(void);

// Lê os dados brutos do microfone. 
// buffer deve ser um ponteiro para um array de int32_t.
esp_err_t mic_ics43434_read(int32_t *buffer, size_t buffer_len_bytes, size_t *bytes_read);

#endif /* MIC_ICS43434_H_ */