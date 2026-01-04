#ifndef BPM_H_
#define BPM_H_

#include <zephyr/device.h>
#include <stdint.h>

/* Wyniki ostatniego poprawnego pomiaru */
extern uint16_t bpm;  
extern uint8_t  spo2; 

void bpm_init(const struct device *i2c_dev);
void bpm_process(const struct device *i2c_dev);
void bpm_reset_state(void);

#endif 
