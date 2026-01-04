#pragma once
#include <zephyr/device.h>
#include <stdbool.h>

// Inicjalizacja czujnika temperatury
bool wear_detect_init(const struct device *i2c_temp);
// Wykrywanie czy urządzenie jest założone
bool wear_detect(const struct device *i2c_temp);
