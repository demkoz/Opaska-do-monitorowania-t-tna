#ifndef MAXIM_ALGORITHM_H_
#define MAXIM_ALGORITHM_H_

#include <stdint.h>

void maxim_heart_rate_and_oxygen_saturation(
    uint32_t *ir_buf,
    int32_t buf_len,
    uint32_t *red_buf,
    int32_t *spo2,
    int8_t *spo2_valid,
    int32_t *heart_rate,
    int8_t *hr_valid);

#endif /* MAXIM_ALGORITHM_H_ */
