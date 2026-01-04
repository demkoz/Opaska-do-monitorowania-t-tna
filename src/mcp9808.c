#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include "mcp9808.h"

#define MCP9808_ADDR 0x18
#define MCP9808_REG_TEMP 0x05

/* Odczyt temperatury z MCP9808 */
bool mcp9808_read_temp(const struct device *i2c, float *out_temp_c)
{
    uint8_t reg = MCP9808_REG_TEMP;
    uint8_t data[2];
    int ret = i2c_write_read(i2c, MCP9808_ADDR, &reg, 1, data, 2);
    if (ret != 0) {
        return false;
    }

    int16_t raw = ((data[0] & 0x1F) << 8) | data[1];
    if (data[0] & 0x10) {
        raw -= 8192; 
    }

    *out_temp_c = raw * 0.0625f;
    return true;
}
