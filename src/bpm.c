#include "bpm.h"
#include "max30102.h"
#include "maxim_algorithm.h"
#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

#define MAXIM_BUF_LEN   100     /* 4 sekundy przy FreqS=25 w algorytmie */
#define FILTER_SIZE     5

static int bpm_buffer[FILTER_SIZE];
static int bpm_index = 0;
static bool buffer_filled = false;

static uint32_t ir_buf[MAXIM_BUF_LEN];
static uint32_t red_buf[MAXIM_BUF_LEN];
static int buf_idx = 0;


uint16_t bpm = 0;
uint8_t  spo2 = 0;

/* Ostatnie poprawne wyniki */
static uint16_t last_bpm = 0;
static uint8_t  last_spo2 = 0;

static int bpm_get_filtered(int new_bpm)
{
    bpm_buffer[bpm_index++] = new_bpm;
    if (bpm_index >= FILTER_SIZE) {
        bpm_index = 0;
        buffer_filled = true;
    }

    int count = buffer_filled ? FILTER_SIZE : bpm_index;
    int sum = 0;
    for (int i = 0; i < count; i++) {
        sum += bpm_buffer[i];
    }
    return (count > 0) ? (sum / count) : new_bpm;
}
    /* Inicjalizacja modułu BPM */
void bpm_init(const struct device *i2c_dev)
{
    MAX30102_check(i2c_dev);
    MAX30102_reset(i2c_dev);
    k_msleep(100);

    MAX30102_config(i2c_dev);
    k_msleep(100);

    buf_idx = 0;
    bpm = 0;
    spo2 = 0;
    last_bpm = 0;
    last_spo2 = 0;
    bpm_index = 0;
    buffer_filled = false;
}
/* Przetwarzanie danych BPM */
void bpm_process(const struct device *i2c_dev)
{
    uint32_t ir_raw = 0, red_raw = 0;

    if (!MAX30102_read_fifo(i2c_dev, &red_raw, &ir_raw)) {
        return;
    }

    ir_buf[buf_idx]  = ir_raw;
    red_buf[buf_idx] = red_raw;
    buf_idx++;

    if (buf_idx >= MAXIM_BUF_LEN) {
        int32_t spo2_calc = 0, hr_calc = 0;
        int8_t spo2_valid = 0, hr_valid = 0;

        maxim_heart_rate_and_oxygen_saturation(
            ir_buf, MAXIM_BUF_LEN, red_buf,
            &spo2_calc, &spo2_valid, &hr_calc, &hr_valid);

        if (hr_valid) {
            int32_t hr = hr_calc;
            if (hr < 0) { hr = 0; }
            if (hr > 250) { hr = 250; }
            last_bpm = (uint16_t)bpm_get_filtered((int)hr);
        }

        if (spo2_valid) {
            int32_t s = spo2_calc;
            if (s < 0) { s = 0; }
            if (s > 100) { s = 100; }
            last_spo2 = (uint8_t)s;
        }

        bpm = last_bpm;
        spo2 = last_spo2;

        buf_idx = 0;
    }
}
/* Resetowanie stanu modułu BPM */
void bpm_reset_state(void)
{
    /* reset filtra */
    for (int i = 0; i < FILTER_SIZE; i++) {
        bpm_buffer[i] = 0;
    }
    bpm_index = 0;
    buffer_filled = false;

    /* reset buforów próbek */
    for (int i = 0; i < MAXIM_BUF_LEN; i++) {
        ir_buf[i] = 0;
        red_buf[i] = 0;
    }
    buf_idx = 0;

    /* reset wyników */
    bpm = 0;
    spo2 = 0;
    last_bpm = 0;
    last_spo2 = 0;
}
