#ifndef WAV_WRITER_H
#define WAV_WRITER_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Estrutura para o cabeçalho de um arquivo WAV (PCM mono de 16 bits)
typedef struct {
    // RIFF Chunk
    char     chunkID[4];     // Deve ser "RIFF"
    uint32_t chunkSize;      // Tamanho total do arquivo menos 8 bytes
    char     format[4];      // Deve ser "WAVE"

    // FMT Subchunk
    char     subchunk1ID[4]; // Deve ser "fmt "
    uint32_t subchunk1Size;  // 16 para PCM
    uint16_t audioFormat;    // 1 para PCM linear
    uint16_t numChannels;    // 1 para mono, 2 para estéreo
    uint32_t sampleRate;     // Taxa de amostragem (ex: 16000)
    uint32_t byteRate;       // SampleRate * NumChannels * BitsPerSample/8
    uint16_t blockAlign;     // NumChannels * BitsPerSample/8
    uint16_t bitsPerSample;  // 16 para PCM de 16 bits

    // DATA Subchunk
    char     subchunk2ID[4]; // Deve ser "data"
    uint32_t subchunk2Size;  // Número de bytes de dados de áudio
} wav_header_t;

/**
 * @brief Escreve um arquivo WAV para o sistema de arquivos.
 *
 * Esta função pega um buffer de amostras de áudio PCM de 16 bits (mono)
 * e as escreve em um arquivo no formato WAV, incluindo o cabeçalho apropriado.
 *
 * @param[in] file_path Caminho completo para o arquivo WAV a ser criado (ex: "/spiffs/audio.wav").
 * @param[in] audio_data Ponteiro para o buffer contendo os dados de áudio PCM (int16_t).
 * @param[in] num_samples Número de amostras (int16_t) no buffer de áudio.
 * @param[in] sample_rate_hz Taxa de amostragem do áudio em Hz.
 * @return
 * - ESP_OK em caso de sucesso.
 * - ESP_FAIL em caso de falha (ex: não foi possível abrir o arquivo, erro de escrita).
 */
esp_err_t wav_writer_write_file(const char *file_path, const int16_t *audio_data, size_t num_samples, uint32_t sample_rate_hz);

#ifdef __cplusplus
}
#endif

#endif // WAV_WRITER_H