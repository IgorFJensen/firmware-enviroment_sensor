#include "wav_writer.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>

static const char *TAG_WAV = "WAV_WRITER";

esp_err_t wav_writer_write_file(const char *file_path, const int16_t *audio_data, size_t num_samples, uint32_t sample_rate_hz) {
    if (!file_path || !audio_data || num_samples == 0 || sample_rate_hz == 0) {
        ESP_LOGE(TAG_WAV, "Argumentos inválidos para wav_writer_write_file.");
        return ESP_ERR_INVALID_ARG;
    }

    FILE *f = fopen(file_path, "wb");
    if (f == NULL) {
        ESP_LOGE(TAG_WAV, "Falha ao abrir o arquivo '%s' para escrita.", file_path);
        return ESP_FAIL;
    }

    wav_header_t header;
    size_t data_size_bytes = num_samples * sizeof(int16_t);

    // Preencher o cabeçalho RIFF
    memcpy(header.chunkID, "RIFF", 4);
    header.chunkSize = 36 + data_size_bytes; // 36 bytes para o cabeçalho + tamanho dos dados
    memcpy(header.format, "WAVE", 4);

    // Preencher o subchunk FMT
    memcpy(header.subchunk1ID, "fmt ", 4);
    header.subchunk1Size = 16;
    header.audioFormat = 1; // PCM
    header.numChannels = 1; // Mono
    header.sampleRate = sample_rate_hz;
    header.bitsPerSample = 16; // 16 bits por amostra
    header.byteRate = header.sampleRate * header.numChannels * (header.bitsPerSample / 8);
    header.blockAlign = header.numChannels * (header.bitsPerSample / 8);

    // Preencher o subchunk DATA
    memcpy(header.subchunk2ID, "data", 4);
    header.subchunk2Size = data_size_bytes;

    // Escrever o cabeçalho no arquivo
    if (fwrite(&header, 1, sizeof(wav_header_t), f) != sizeof(wav_header_t)) {
        ESP_LOGE(TAG_WAV, "Falha ao escrever o cabeçalho WAV.");
        fclose(f);
        return ESP_FAIL;
    }

    // Escrever os dados de áudio no arquivo
    if (fwrite(audio_data, 1, data_size_bytes, f) != data_size_bytes) {
        ESP_LOGE(TAG_WAV, "Falha ao escrever os dados de áudio.");
        fclose(f);
        return ESP_FAIL;
    }

    fclose(f);
    ESP_LOGI(TAG_WAV, "Arquivo WAV '%s' escrito com sucesso. Tamanho: %u bytes.", file_path, header.chunkSize + 8);
    return ESP_OK;
}
