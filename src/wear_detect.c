#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
//#include <zephyr/sys/printk.h>
#include "wear_detect.h"

#define MCP9808_ADDR 0x18
#define MCP9808_REG_TEMP 0x05

static float last_temp_c = 0.0f;

bool wear_detect_init(const struct device *i2c_temp)
{
    if (!device_is_ready(i2c_temp)) {
        //printk("Wear detect: I2C not ready!\n");
        return false;
    }

    uint8_t reg = 0x07;
    uint8_t id[2];
    if (i2c_write_read(i2c_temp, MCP9808_ADDR, &reg, 1, id, 2) == 0) {
        if (id[0] == 0x04 && id[1] == 0x00) {
            printk("MCP9808 detected OK\n");
            return true;
        }
    }
  //  printk("MCP9808 not responding\n");
    return false;
}
/*  wykrywanie noszenia */
bool wear_detect(const struct device *i2c_temp)
{
    uint8_t reg = MCP9808_REG_TEMP;
    uint8_t data[2];
    int ret = i2c_write_read(i2c_temp, MCP9808_ADDR, &reg, 1, data, 2);

    if (ret != 0) {
        //printk("Wear detect: I2C read error %d\n", ret);
        return false;
    }

    int16_t raw = ((data[0] & 0x1F) << 8) | data[1];
    if (data[0] & 0x10) raw -= 8192; // temp < 0
    last_temp_c = raw * 0.0625f;

    int t_int = (int)last_temp_c;
    int t_frac = (int)((last_temp_c - t_int) * 100);
    if (t_frac < 0) t_frac = -t_frac;


    //printk("Temp: %d.%02d C\n", t_int, t_frac);

    return (last_temp_c > 28.0f);
}
