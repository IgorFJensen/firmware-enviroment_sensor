#ifndef STATS_UTILS_H_
#define STATS_UTILS_H_

#include <stddef.h>
#include <stdint.h>

// Count valid float samples (ignoring sentinel)
int stats_count_valid_f(const float *data, size_t n, float invalid_sentinel);

// Mean and variance for float arrays (ignoring sentinel values)
float stats_mean_f(const float *data, size_t n, float invalid_sentinel);
float stats_variance_f(const float *data, size_t n, float invalid_sentinel);

// Mean and variance helpers for uint16 arrays (treats `invalid_sentinel` as invalid)
float stats_mean_u16(const uint16_t *data, size_t n, uint16_t invalid_sentinel);
float stats_variance_u16(const uint16_t *data, size_t n, uint16_t invalid_sentinel);

#endif // STATS_UTILS_H_
