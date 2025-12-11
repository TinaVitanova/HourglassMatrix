#ifndef DOT_MATRIX
#define DOT_MATRIX

#include <string.h>
#include <stdio.h>
#include "esp_check.h"

typedef struct {
    uint8_t grains[64]; // 8x8
} MatrixSand;

#define MAX_DIGIT0 0x01
#define MAX7219_REG_SHUTDOWN 0x0C
#define MPU_INT_STATUS       0x3A

void max7219_broadcast(uint8_t addr, uint8_t data);

void max7219_write_to_displays(uint8_t addr, uint8_t data1, uint8_t data2);

void max7219_init(void);

void max7219_clear(void);

#endif