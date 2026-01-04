#pragma once
#include <stdint.h>
#include <stdbool.h>
void ble_init(void);

int ble_update(uint16_t bpm, uint8_t spo2, bool worn);

/* 0 = brak alarmu, 1 = alarm aktywny */
int ble_alarm_update(uint8_t alarm);
