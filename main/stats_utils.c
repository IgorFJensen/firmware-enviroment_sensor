#include "stats_utils.h"
#include <stdint.h>

// Count valid float samples (ignoring sentinel)
int stats_count_valid_f(const float *data, size_t n, float invalid_sentinel)
{
    int cnt = 0;
    for (size_t i = 0; i < n; ++i) {
        if (data[i] != invalid_sentinel) cnt++;
    }
    return cnt;
}

// Mean for float array ignoring sentinel
float stats_mean_f(const float *data, size_t n, float invalid_sentinel)
{
    float sum = 0.0f;
    int cnt = 0;
    for (size_t i = 0; i < n; ++i) {
        float v = data[i];
        if (v == invalid_sentinel) continue;
        sum += v;
        cnt++;
    }
    return (cnt > 0) ? (sum / cnt) : 0.0f;
}

// Variance for float array ignoring sentinel (population variance)
float stats_variance_f(const float *data, size_t n, float invalid_sentinel)
{
    int cnt = 0;
    float mean = 0.0f;
    // First pass: compute mean
    for (size_t i = 0; i < n; ++i) {
        float v = data[i];
        if (v == invalid_sentinel) continue;
        mean += v;
        cnt++;
    }
    if (cnt == 0) return 0.0f;
    mean /= cnt;

    // Second pass: accumulate squared deviations
    float acc = 0.0f;
    for (size_t i = 0; i < n; ++i) {
        float v = data[i];
        if (v == invalid_sentinel) continue;
        float d = v - mean;
        acc += d * d; // no math.h used
    }
    return acc / cnt;
}

// uint16 helpers
float stats_mean_u16(const uint16_t *data, size_t n, uint16_t invalid_sentinel)
{
    uint64_t sum = 0;
    size_t cnt = 0;
    for (size_t i = 0; i < n; ++i) {
        uint16_t v = data[i];
        if (v == invalid_sentinel) continue;
        sum += v;
        cnt++;
    }
    return (cnt > 0) ? ((float)sum / (float)cnt) : 0.0f;
}

float stats_variance_u16(const uint16_t *data, size_t n, uint16_t invalid_sentinel)
{
    size_t cnt = 0;
    float mean = 0.0f;
    for (size_t i = 0; i < n; ++i) {
        uint16_t v = data[i];
        if (v == invalid_sentinel) continue;
        mean += (float)v;
        cnt++;
    }
    if (cnt == 0) return 0.0f;
    mean /= cnt;

    float acc = 0.0f;
    for (size_t i = 0; i < n; ++i) {
        uint16_t v = data[i];
        if (v == invalid_sentinel) continue;
        float d = (float)v - mean;
        acc += d * d;
    }
    return acc / cnt;
}
