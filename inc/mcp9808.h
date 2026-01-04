#pragma once
#include <zephyr/device.h>
#include <stdbool.h>
// Odczyt temperatury z MCP9808
bool mcp9808_read_temp(const struct device *i2c, float *out_temp_c);
