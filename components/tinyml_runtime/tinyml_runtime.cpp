#include "tinyml_runtime.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include "model_data.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"


namespace {

constexpr int kSampleRate = 16000;
constexpr int kAudioSamples = 16000;
constexpr int kFrameLength = 480;
constexpr int kFrameStep = 320;
constexpr int kFftLength = 512;
constexpr int kSpectrumBins = (kFftLength / 2) + 1;
constexpr int kFrames = 49;
constexpr int kMelBins = 32;
constexpr float kLowerHz = 80.0f;
constexpr float kUpperHz = 7600.0f;
constexpr float kLogFloor = 1.0e-6f;
constexpr float kPi = 3.14159265358979323846f;

constexpr size_t kTensorArenaBytes = 64U * 1024U;

/* Ponto operacional escolhido para o firmware, preservando especificidade. */
constexpr float kRiskOnThreshold = 0.415f;
constexpr float kRiskOffThreshold = 0.400f;
constexpr uint8_t kHistorySize = 3;
constexpr uint8_t kRiskOnVotes = 2;
constexpr uint8_t kRiskKeepVotes = 2;

const char *TAG = "TINYML_V61";

alignas(16) uint8_t s_tensor_arena[kTensorArenaBytes];
float s_hann[kFrameLength];
float s_mel_weights[kSpectrumBins][kMelBins];
float s_fft_real[kFftLength];
float s_fft_imag[kFftLength];
float s_probability_history[kHistorySize];

const tflite::Model *s_model = nullptr;
tflite::MicroInterpreter *s_interpreter = nullptr;
TfLiteTensor *s_input = nullptr;
TfLiteTensor *s_output = nullptr;
bool s_initialized = false;
uint8_t s_history_head = 0;
uint8_t s_history_count = 0;

tinyml_risk_status_t s_status = {};
portMUX_TYPE s_status_mux = portMUX_INITIALIZER_UNLOCKED;

float hertz_to_mel(float hertz)
{
    return 1127.0f * logf(1.0f + hertz / 700.0f);
}

void prepare_hann_window()
{
    /* tf.signal.hann_window usa periodic=True por padrao. */
    for (int n = 0; n < kFrameLength; ++n) {
        s_hann[n] = 0.5f - 0.5f * cosf((2.0f * kPi * n) / kFrameLength);
    }
}

void prepare_mel_filterbank()
{
    const float lower_mel = hertz_to_mel(kLowerHz);
    const float upper_mel = hertz_to_mel(kUpperHz);
    float edges[kMelBins + 2];

    for (int i = 0; i < kMelBins + 2; ++i) {
        edges[i] = lower_mel
            + (upper_mel - lower_mel) * static_cast<float>(i)
                / static_cast<float>(kMelBins + 1);
    }

    for (int k = 0; k < kSpectrumBins; ++k) {
        const float hertz = static_cast<float>(k * kSampleRate) / kFftLength;
        const float mel = hertz_to_mel(hertz);

        for (int band = 0; band < kMelBins; ++band) {
            const float lower = (mel - edges[band])
                / (edges[band + 1] - edges[band]);
            const float upper = (edges[band + 2] - mel)
                / (edges[band + 2] - edges[band + 1]);
            const float weight = fminf(lower, upper);
            s_mel_weights[k][band] = fmaxf(0.0f, weight);
        }
    }
}

void fft_512()
{
    for (unsigned i = 1, j = 0; i < kFftLength; ++i) {
        unsigned bit = kFftLength >> 1;
        for (; j & bit; bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;

        if (i < j) {
            const float tmp_real = s_fft_real[i];
            const float tmp_imag = s_fft_imag[i];
            s_fft_real[i] = s_fft_real[j];
            s_fft_imag[i] = s_fft_imag[j];
            s_fft_real[j] = tmp_real;
            s_fft_imag[j] = tmp_imag;
        }
    }

    for (unsigned length = 2; length <= kFftLength; length <<= 1) {
        const float angle = -2.0f * kPi / static_cast<float>(length);
        const float wlen_real = cosf(angle);
        const float wlen_imag = sinf(angle);

        for (unsigned start = 0; start < kFftLength; start += length) {
            float w_real = 1.0f;
            float w_imag = 0.0f;

            for (unsigned j = 0; j < length / 2; ++j) {
                const unsigned even = start + j;
                const unsigned odd = even + length / 2;
                const float odd_real = s_fft_real[odd] * w_real
                    - s_fft_imag[odd] * w_imag;
                const float odd_imag = s_fft_real[odd] * w_imag
                    + s_fft_imag[odd] * w_real;
                const float even_real = s_fft_real[even];
                const float even_imag = s_fft_imag[even];

                s_fft_real[even] = even_real + odd_real;
                s_fft_imag[even] = even_imag + odd_imag;
                s_fft_real[odd] = even_real - odd_real;
                s_fft_imag[odd] = even_imag - odd_imag;

                const float next_w_real = w_real * wlen_real - w_imag * wlen_imag;
                w_imag = w_real * wlen_imag + w_imag * wlen_real;
                w_real = next_w_real;
            }
        }
    }
}

int8_t quantize_feature(float value)
{
    const float scale = s_input->params.scale;
    const int zero_point = s_input->params.zero_point;
    int32_t quantized = static_cast<int32_t>(lrintf(value / scale)) + zero_point;

    if (quantized > INT8_MAX) {
        quantized = INT8_MAX;
    } else if (quantized < INT8_MIN) {
        quantized = INT8_MIN;
    }

    return static_cast<int8_t>(quantized);
}

void extract_logmel(const int16_t *pcm)
{
    int feature_index = 0;

    for (int frame = 0; frame < kFrames; ++frame) {
        const int offset = frame * kFrameStep;

        for (int i = 0; i < kFftLength; ++i) {
            s_fft_real[i] = (i < kFrameLength)
                ? (static_cast<float>(pcm[offset + i]) / 32768.0f) * s_hann[i]
                : 0.0f;
            s_fft_imag[i] = 0.0f;
        }

        fft_512();

        float mel_energy[kMelBins] = {};
        for (int k = 0; k < kSpectrumBins; ++k) {
            const float power = s_fft_real[k] * s_fft_real[k]
                + s_fft_imag[k] * s_fft_imag[k];

            for (int band = 0; band < kMelBins; ++band) {
                mel_energy[band] += power * s_mel_weights[k][band];
            }
        }

        for (int band = 0; band < kMelBins; ++band) {
            const float logmel = logf(mel_energy[band] + kLogFloor);
            s_input->data.int8[feature_index++] = quantize_feature(logmel);
        }
    }
}

bool tensor_shape_is(const TfLiteTensor *tensor, int d0, int d1, int d2, int d3)
{
    return tensor != nullptr
        && tensor->dims != nullptr
        && tensor->dims->size == 4
        && tensor->dims->data[0] == d0
        && tensor->dims->data[1] == d1
        && tensor->dims->data[2] == d2
        && tensor->dims->data[3] == d3;
}

void update_temporal_decision(float probability)
{
    s_probability_history[s_history_head] = probability;
    s_history_head = (s_history_head + 1U) % kHistorySize;
    if (s_history_count < kHistorySize) {
        ++s_history_count;
    }

    const bool was_active = s_status.risk_active;
    const float vote_threshold = was_active ? kRiskOffThreshold : kRiskOnThreshold;
    uint8_t votes = 0;
    float sum = 0.0f;

    for (uint8_t i = 0; i < s_history_count; ++i) {
        sum += s_probability_history[i];
        if (s_probability_history[i] >= vote_threshold) {
            ++votes;
        }
    }

    bool is_active = was_active;
    if (!was_active && s_history_count >= kRiskOnVotes && votes >= kRiskOnVotes) {
        is_active = true;
    } else if (was_active && s_history_count == kHistorySize && votes < kRiskKeepVotes) {
        is_active = false;
    }

    tinyml_risk_status_t next = {};
    next.probability = probability;
    next.history_average = sum / static_cast<float>(s_history_count);
    next.risk_active = is_active;
    next.positive_votes = votes;
    next.history_count = s_history_count;
    next.inference_count = s_status.inference_count + 1U;

    portENTER_CRITICAL(&s_status_mux);
    s_status = next;
    portEXIT_CRITICAL(&s_status_mux);

    ESP_LOGI(TAG,
             "p=%.3f media=%.3f votos=%u/%u limite=%.3f estado=%s",
             probability,
             next.history_average,
             next.positive_votes,
             next.history_count,
             vote_threshold,
             next.risk_active ? "RISCO" : "NORMAL");

    if (is_active != was_active) {
        ESP_LOGW(TAG, "%s",
                 is_active
                     ? ">>> RISCO CONFIRMADO (3 de 5 janelas) <<<"
                     : ">>> RISCO ENCERRADO (histerese) <<<");
    }
}

}  // namespace

extern "C" esp_err_t tinyml_runtime_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    s_model = tflite::GetModel(g_model);
    if (s_model == nullptr || s_model->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(TAG, "Modelo ausente ou schema TFLite incompativel");
        return ESP_ERR_INVALID_VERSION;
    }

    static tflite::MicroMutableOpResolver<5> resolver;
    if (resolver.AddConv2D() != kTfLiteOk
        || resolver.AddDepthwiseConv2D() != kTfLiteOk
        || resolver.AddMean() != kTfLiteOk
        || resolver.AddFullyConnected() != kTfLiteOk
        || resolver.AddLogistic() != kTfLiteOk) {
        ESP_LOGE(TAG, "Falha registrando operacoes TFLite");
        return ESP_FAIL;
    }

    static tflite::MicroInterpreter interpreter(
        s_model, resolver, s_tensor_arena, kTensorArenaBytes);
    s_interpreter = &interpreter;

    if (s_interpreter->AllocateTensors() != kTfLiteOk) {
        ESP_LOGE(TAG, "Arena TFLite insuficiente (%u bytes)",
                 static_cast<unsigned>(kTensorArenaBytes));
        return ESP_ERR_NO_MEM;
    }

    s_input = s_interpreter->input(0);
    s_output = s_interpreter->output(0);

    if (s_input == nullptr || s_input->type != kTfLiteInt8
        || !tensor_shape_is(s_input, 1, kFrames, kMelBins, 1)) {
        ESP_LOGE(TAG, "Entrada inesperada; esperado INT8 [1,49,32,1]");
        return ESP_ERR_INVALID_SIZE;
    }

    if (s_output == nullptr || s_output->type != kTfLiteInt8
        || s_output->dims == nullptr || s_output->dims->size != 2
        || s_output->dims->data[0] != 1 || s_output->dims->data[1] != 1) {
        ESP_LOGE(TAG, "Saida inesperada; esperado INT8 [1,1]");
        return ESP_ERR_INVALID_SIZE;
    }

    if (s_input->params.scale <= 0.0f || s_output->params.scale <= 0.0f) {
        ESP_LOGE(TAG, "Parametros de quantizacao invalidos");
        return ESP_ERR_INVALID_ARG;
    }

    prepare_hann_window();
    prepare_mel_filterbank();
    memset(s_probability_history, 0, sizeof(s_probability_history));
    memset(&s_status, 0, sizeof(s_status));
    s_history_head = 0;
    s_history_count = 0;
    s_initialized = true;

ESP_LOGI(
    TAG,
    "V6.1 pronto: modelo=%u bytes, arena usada=%u/%u, entrada=(%.9f,%d), saida=(%.9f,%d)",
    g_model_len,
    static_cast<unsigned>(s_interpreter->arena_used_bytes()),
    static_cast<unsigned>(kTensorArenaBytes),
    s_input->params.scale,
    static_cast<int>(s_input->params.zero_point),
    s_output->params.scale,
    static_cast<int>(s_output->params.zero_point)
);

ESP_LOGI(
    TAG,
    "Decisao: liga com %u/%u >= %.3f; "
    "mantem com %u/%u >= %.3f",
    kRiskOnVotes,
    kHistorySize,
    kRiskOnThreshold,
    kRiskKeepVotes,
    kHistorySize,
    kRiskOffThreshold);

return ESP_OK;
}

extern "C" esp_err_t tinyml_runtime_infer_pcm16(
    const int16_t *pcm,
    size_t sample_count)
{
    if (!s_initialized || s_interpreter == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (pcm == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (sample_count != kAudioSamples) {
        return ESP_ERR_INVALID_SIZE;
    }

    extract_logmel(pcm);

    if (s_interpreter->Invoke() != kTfLiteOk) {
        ESP_LOGE(TAG, "Invoke falhou");
        return ESP_FAIL;
    }

    float probability =
        (static_cast<int>(s_output->data.int8[0]) - s_output->params.zero_point)
        * s_output->params.scale;
    probability = fminf(1.0f, fmaxf(0.0f, probability));
    update_temporal_decision(probability);
    return ESP_OK;
}

extern "C" esp_err_t tinyml_runtime_get_status(tinyml_risk_status_t *out_status)
{
    if (out_status == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    portENTER_CRITICAL(&s_status_mux);
    *out_status = s_status;
    portEXIT_CRITICAL(&s_status_mux);
    return ESP_OK;
}

