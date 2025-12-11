#include "driver/i2c.h"
#include "esp_check.h"

#define I2C_PORT I2C_NUM_0
#define I2C_SDA 8
#define I2C_SCL 9
#define I2C_FREQ_HZ 400000

#define MPU_ADDR 0x68
#define MPU_PWR_MGMT_1 0x6B
#define MPU_ACCEL_XOUT_H 0x3B

#define MPU_INT_PIN_CFG      0x37
#define MPU_INT_ENABLE       0x38
#define MPU_MOT_THR          0x1F
#define MPU_MOT_DUR          0x20
#define MPU_MOT_DETECT_CTRL  0x69

esp_err_t mpu_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_write_to_device(I2C_PORT, MPU_ADDR, buf, 2, pdMS_TO_TICKS(100));
}

void mpu_config_motion_interrupt(uint8_t mot_thr, uint8_t mot_dur) {
    mpu_write(MPU_INT_PIN_CFG, 0x20);
    // Configure INT pin: active-high, push-pull, latch until INT_STATUS read
    // mpu_write(MPU_INT_PIN_CFG, 0x20);    // LATCH_INT_EN = 1, others 0

    // Set motion threshold (~2 mg/LSB)
    mpu_write(MPU_MOT_THR, mot_thr);

    // Motion duration (~1 ms/LSB)
    mpu_write(MPU_MOT_DUR, mot_dur);

    // Motion detect control: set accelerometer delay counter, recommended 0x15
    mpu_write(MPU_MOT_DETECT_CTRL, 0x15);

    // Enable Motion Interrupt (bit6 = 1)
    mpu_write(MPU_INT_ENABLE, 0x40);
}

esp_err_t i2c_init(void) {
    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &cfg));
    return i2c_driver_install(I2C_PORT, cfg.mode, 0, 0, 0);
}

esp_err_t mpu_read_accel_raw(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t reg = MPU_ACCEL_XOUT_H;
    uint8_t data[6];
    ESP_ERROR_CHECK(i2c_master_write_read_device(I2C_PORT, MPU_ADDR, &reg, 1, data, 6, pdMS_TO_TICKS(100)));
    *ax = (int16_t)((data[0] << 8) | data[1]);
    *ay = (int16_t)((data[2] << 8) | data[3]);
    *az = (int16_t)((data[4] << 8) | data[5]);
    return ESP_OK;
}

esp_err_t mpu_init(void) {
    // wake up: PWR_MGMT_1 = 0x00
    return mpu_write(MPU_PWR_MGMT_1, 0x00);
}