#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdint.h>
#include <stdbool.h>

#include "bpm.h"
#include "bpm_ble.h"
#include "wear_detect.h"

#define I2C_BPM_NODE   DT_NODELABEL(i2c0)
#define I2C_TEMP_NODE  DT_NODELABEL(i2c1)

#define BUZZER_PORT    DT_NODELABEL(gpio1)
#define BUZZER_PIN     15
#define TONE_HZ        4000

#define CALIBRATION_MS      15000
#define SEND_INTERVAL_MS     4000

#define BPM_MAX_VALID     200U
#define SPO2_MAX_VALID    100U

#define HR_ALARM_ON                 70U /* 70 bpm do testów*/
#define HR_ALARM_OFF                75U /* 75 bpm do testów*/
#define HR_ALARM_CONFIRM_SAMPLES    2U

#define ALARM_BEEP_PERIOD_MS        1000
#define ALARM_BEEP_TONE_MS          80

/* Timing */
#define MEAS_PERIOD_MS              40      /* 25 Hz */
#define WEAR_CHECK_ON_MS            1000
#define WEAR_CHECK_OFF_MS           200
#define OFF_BEEP_PERIOD_MS          1360

static const struct device *i2c_bpm;
static const struct device *i2c_temp;
static const struct device *buzzer_gpio;

/* generowanie dzwięku buzezera */
static void buzzer_tone(uint32_t freq_hz, uint32_t duration_ms)
{
    int64_t t_end = k_uptime_get() + (int64_t)duration_ms;
    uint32_t half_period_us = 1000000U / (freq_hz * 2U);

    while (k_uptime_get() < t_end) {
        gpio_pin_set(buzzer_gpio, BUZZER_PIN, 1);
        k_busy_wait(half_period_us);
        gpio_pin_set(buzzer_gpio, BUZZER_PIN, 0);
        k_busy_wait(half_period_us);
    }
    gpio_pin_set(buzzer_gpio, BUZZER_PIN, 0);
}
/* Ustawienie dźwięku dla opaski */
static void buzzer_band_off_pattern(void)
{
    buzzer_tone(TONE_HZ, 120);
    k_msleep(120);
    buzzer_tone(TONE_HZ, 120);
}
/* Ustawienie dźwięku dla alarmu opaski*/
static void buzzer_alarm_pattern(void)
{
    buzzer_tone(TONE_HZ, ALARM_BEEP_TONE_MS);
    k_msleep(ALARM_BEEP_TONE_MS);
    buzzer_tone(TONE_HZ, ALARM_BEEP_TONE_MS);
}
/* Sprawdzenie poprawności próbek */
static bool sample_valid(uint16_t hr, uint8_t s)
{
    if (hr == 0U || s == 0U) {
        return false;
    }
    if (hr > BPM_MAX_VALID) {
        return false;
    }
    if (s > SPO2_MAX_VALID) {
        return false;
    }
    return true;
}

static int64_t min_i64(int64_t a, int64_t b)
{
    return (a < b) ? a : b;
}

/* stan */
struct app_state {
    bool worn;
    bool calibrated;
    int64_t calib_end_ms;

    bool alarm_active;
    uint8_t low_confirm;
    uint8_t high_confirm;
    uint8_t last_alarm_sent;
};

static struct app_state st;
/* Ustawienie alarmu */
static void alarm_set(uint8_t on)
{
    on = on ? 1U : 0U;
    if (st.last_alarm_sent != on) {
        (void)ble_alarm_update(on);
        st.last_alarm_sent = on;
    }
}
/* Ustawienie dźwięku dla opaski */
static void on_band_put_on(int64_t now_ms)
{
    st.worn = true;
    st.calibrated = false;
    st.calib_end_ms = now_ms + (int64_t)CALIBRATION_MS;

    st.alarm_active = false;
    st.low_confirm = 0U;
    st.high_confirm = 0U;
    st.last_alarm_sent = 0U;

    alarm_set(0U);
    (void)ble_update(0U, 0U, true);
}
/* Ustawienie dźwięku dla opaski */
static void on_band_taken_off(void)
{
    st.worn = false;
    st.calibrated = false;

    st.alarm_active = false;
    st.low_confirm = 0U;
    st.high_confirm = 0U;
    st.last_alarm_sent = 0U;

    alarm_set(0U);
    (void)ble_update(0U, 0U, false);
}

int main(void)
{
    i2c_bpm  = DEVICE_DT_GET(I2C_BPM_NODE);
    i2c_temp = DEVICE_DT_GET(I2C_TEMP_NODE);
    buzzer_gpio = DEVICE_DT_GET(BUZZER_PORT);

    if (!device_is_ready(i2c_bpm) || !device_is_ready(i2c_temp)) {
        return 0;
    }
    if (!device_is_ready(buzzer_gpio)) {
        return 0;
    }
    if (gpio_pin_configure(buzzer_gpio, BUZZER_PIN, GPIO_OUTPUT_INACTIVE) != 0) {
        return 0;
    }

    bpm_init(i2c_bpm);
    (void)wear_detect_init(i2c_temp);
    ble_init();

    st.worn = false;
    st.calibrated = false;
    st.alarm_active = false;
    st.low_confirm = 0U;
    st.high_confirm = 0U;
    st.last_alarm_sent = 0U;
    st.calib_end_ms = 0;

    int64_t now = k_uptime_get();

    int64_t next_meas_ms       = now + (int64_t)MEAS_PERIOD_MS;
    int64_t next_wear_ms       = now;
    int64_t next_send_ms       = now + (int64_t)SEND_INTERVAL_MS;
    int64_t next_alarm_beep_ms = now + (int64_t)ALARM_BEEP_PERIOD_MS;
    int64_t next_off_beep_ms   = now;

    /* Wykryj czy założona */
    bool worn0 = wear_detect(i2c_temp);
    now = k_uptime_get();
    if (worn0) {
        on_band_put_on(now);
        next_send_ms = st.calib_end_ms;
        next_wear_ms = now + (int64_t)WEAR_CHECK_ON_MS;
    } else {
        on_band_taken_off();
        next_wear_ms = now + (int64_t)WEAR_CHECK_OFF_MS;
    }

    while (1) {
        now = k_uptime_get();

        /* Wear detect */
        if (now >= next_wear_ms) {
            bool w = wear_detect(i2c_temp);

            if (w && !st.worn) {
                on_band_put_on(now);
                next_send_ms = st.calib_end_ms;
                next_meas_ms = now + (int64_t)MEAS_PERIOD_MS;
                next_wear_ms = now + (int64_t)WEAR_CHECK_ON_MS;
            } else if (!w && st.worn) {
                on_band_taken_off();
                next_off_beep_ms = now;
                next_wear_ms = now + (int64_t)WEAR_CHECK_OFF_MS;
            } else {
                next_wear_ms = now + (int64_t)(st.worn ? WEAR_CHECK_ON_MS : WEAR_CHECK_OFF_MS);
            }
        }

        if (st.worn) {
            /* Kalibracja */
            if (!st.calibrated && now >= st.calib_end_ms) {
                st.calibrated = true;
                st.alarm_active = false;
                st.low_confirm = 0U;
                st.high_confirm = 0U;
                alarm_set(0U);
                next_send_ms = now;
            }

            /* nadrabiaj brakujące ticki  */
            int catchup = 0;
            while (now >= next_meas_ms && catchup < 5) {
                bpm_process(i2c_bpm);
                next_meas_ms += (int64_t)MEAS_PERIOD_MS;
                catchup++;
                now = k_uptime_get();
            }
            if (catchup >= 5) {
                /* jak było duże opóźnienie, zresync do "teraz" */
                next_meas_ms = now + (int64_t)MEAS_PERIOD_MS;
            }

            /* Logika alarmu po aktualizacji bpm/spo2 */
            if (st.calibrated && sample_valid(bpm, spo2)) {
                if (!st.alarm_active) {
                    if (bpm < HR_ALARM_ON) {
                        if (st.low_confirm < 255U) {
                            st.low_confirm++;
                        }
                        if (st.low_confirm >= HR_ALARM_CONFIRM_SAMPLES) {
                            st.alarm_active = true;
                            st.high_confirm = 0U;
                            alarm_set(1U);
                            next_alarm_beep_ms = now;
                        }
                    } else {
                        st.low_confirm = 0U;
                    }
                } else {
                    if (bpm > HR_ALARM_OFF) {
                        if (st.high_confirm < 255U) {
                            st.high_confirm++;
                        }
                        if (st.high_confirm >= HR_ALARM_CONFIRM_SAMPLES) {
                            st.alarm_active = false;
                            st.low_confirm = 0U;
                            alarm_set(0U);
                        }
                    } else {
                        st.high_confirm = 0U;
                    }
                }
            }

            /* Alarm beep */
            if (st.calibrated && st.alarm_active && now >= next_alarm_beep_ms) {
                buzzer_alarm_pattern();
                next_alarm_beep_ms += (int64_t)ALARM_BEEP_PERIOD_MS;
                while (next_alarm_beep_ms <= now) {
                    next_alarm_beep_ms += (int64_t)ALARM_BEEP_PERIOD_MS;
                }
            }

            /* BLE wysyłka */
            if (st.calibrated && now >= next_send_ms) {
                if (sample_valid(bpm, spo2)) {
                    (void)ble_update(bpm, spo2, true);
                } else {
                    (void)ble_update(0U, 0U, true);
                }

                next_send_ms += (int64_t)SEND_INTERVAL_MS;
                while (next_send_ms <= now) {
                    next_send_ms += (int64_t)SEND_INTERVAL_MS;
                }
            }
        } else {
            /* Zdjęta */
            if (now >= next_off_beep_ms) {
                buzzer_band_off_pattern();
                next_off_beep_ms = now + (int64_t)OFF_BEEP_PERIOD_MS;
            }
        }

        /* Sen do najbli ższego deadline */
        int64_t next_deadline = now + 50;
        next_deadline = min_i64(next_deadline, next_wear_ms);

        if (st.worn) {
            next_deadline = min_i64(next_deadline, next_meas_ms);
            if (st.calibrated) {
                next_deadline = min_i64(next_deadline, next_send_ms);
                if (st.alarm_active) {
                    next_deadline = min_i64(next_deadline, next_alarm_beep_ms);
                }
            }
        } else {
            next_deadline = min_i64(next_deadline, next_off_beep_ms);
        }

        int64_t sleep_ms = next_deadline - k_uptime_get();
        if (sleep_ms < 0) {
            sleep_ms = 0;
        }
        if (sleep_ms > 100) {
            sleep_ms = 100;
        }

        k_msleep((uint32_t)sleep_ms);
    }

    return 0;
}
