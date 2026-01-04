#ifndef MAX30102_H_
#define MAX30102_H_

#include <zephyr/device.h>
#include <stdint.h>
#include <stdbool.h>

#define MAX30102_ADDRESS  0x57 // I2C Address for MAX30102

#define MAX30102_INT_STATUS_1        0x00
#define MAX30102_INT_STATUS_2        0x01
#define MAX30102_INT_ENABLE_1        0x02
#define MAX30102_INT_ENABLE_2        0x03
#define MAX30102_FIFO_WRITE_PTR      0x04
#define MAX30102_OVERFLOW_COUNT      0x05
#define MAX30102_FIFO_READ_PTR       0x06
#define MAX30102_REG_FIFO_DATA       0x07
#define MAX30102_FIFO_CONFIG         0x08
#define MAX30102_MODE_CONFIG         0x09
#define MAX30102_SPO2_CONFIG         0x0A
#define MAX30102_LED1_PULSE_AMP      0x0C
#define MAX30102_LED2_PULSE_AMP      0x0D
#define MAX30102_PARTID              0xFF
#define MAX30102_RESET               0x40

/* Deklaracje funkcji dostępnych globalnie */
int  MAX30102_check(const struct device *dev_i2c);
int  MAX30102_reset(const struct device *dev_i2c);
int  MAX30102_clear(const struct device *dev_i2c);
int  MAX30102_config(const struct device *dev_i2c);
bool MAX30102_read_fifo(const struct device *dev_i2c, uint32_t *pun_red_led, uint32_t *pun_ir_led);
bool MAX30102_read_reg(const struct device *dev_i2c, uint8_t uch_addr, uint8_t *puch_data);
bool MAX30102_set_shutdown(const struct device *i2c, bool en);
void max30102_enable(const struct device *dev, bool enable);
void max30102_read_samples(const struct device *dev);
uint8_t max30102_calculate_bpm(void);

#endif 