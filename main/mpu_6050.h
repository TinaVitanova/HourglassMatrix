#ifndef MPU_6050
#define MPU_6050

#include "esp_check.h"

void mpu_config_motion_interrupt(uint8_t mot_thr, uint8_t mot_dur);

esp_err_t i2c_init(void);

esp_err_t mpu_write(uint8_t reg, uint8_t val);

esp_err_t mpu_read_accel_raw(int16_t *ax, int16_t *ay, int16_t *az);

esp_err_t mpu_init(void);

#endif