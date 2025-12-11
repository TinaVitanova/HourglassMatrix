#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_check.h"
#include "mpu_6050.h"
#include "dot_matrix.h"
#include "driver/i2c.h"
#include "esp_sleep.h"
#include "esp_timer.h"


RTC_DATA_ATTR static MatrixSand saved_sand1;
RTC_DATA_ATTR static MatrixSand saved_sand2;
RTC_DATA_ATTR static bool has_saved_state = false;

#define DELAY_MS 200

// GPIO0..GPIO5
#define MPU_INT_GPIO         GPIO_NUM_0
#define INACTIVITY_MS        5000

static uint64_t last_motion_us;

void configure_gpio_for_wakeup(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << MPU_INT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE, // Adjust to match your actual physical circuit
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

void enter_deep_sleep_wait_motion(void) {
    configure_gpio_for_wakeup();
    esp_deep_sleep_enable_gpio_wakeup(1ULL << MPU_INT_GPIO, ESP_GPIO_WAKEUP_GPIO_HIGH);
    esp_deep_sleep_start();
}

static void rotate90_counterclockwise(uint8_t rows[8]) {
    uint8_t out[8] = {0};

    for (int y = 0; y < 8; ++y) {
        for (int x = 0; x < 8; ++x) {
            if (rows[y] & (1u << (7 - x))) {
                int nx = y;          // new column
                int ny = 7 - x;      // new row
                out[ny] |= (1u << (7 - nx));
            }
        }
    }
    memcpy(rows, out, 8);
}

static void rotate90_clockwise(uint8_t rows[8]) {
    uint8_t out[8] = {0};

    for (int y = 0; y < 8; ++y) {
        for (int x = 0; x < 8; ++x) {
            if (rows[y] & (1u << (7 - x))) {
                int nx = 7 - y;      // new column
                int ny = x;          // new row
                out[ny] |= (1u << (7 - nx));
            }
        }
    }
    memcpy(rows, out, 8);
}

static inline uint64_t now_us(void) {
    return esp_timer_get_time();
}


static inline int idx(MatrixSand *s, int x, int y) {
    return x + 8 * y;
}

static bool ms_get(MatrixSand *s, int x, int y) {
    return s->grains[idx(s, x, y)] != 0;
}

static void ms_set(MatrixSand *s, int x, int y, bool v) {
    s->grains[idx(s, x, y)] = v ? 1 : 0;
}

static void ms_init(MatrixSand *s) {
    memset(s->grains, 0, sizeof(s->grains));// [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
}

// Count grains on left vs right of main diagonal, with optional upside-down swap
static void ms_side_count(MatrixSand *s, bool upside_down, int *left, int *right) {
    int L = 0, R = 0;
    for (int x = 0; x < 8; x++) {
        for (int y = 0; y < 8; y++) {
            if (x != y && ms_get(s, x, y)) {
                if (x > y) R++; else L++;
            }
        }
    }
    if (upside_down) { *left = R; *right = L; } else { *left = L; *right = R; }
}


inline int clamp_value(int value, int low, int high) {
    return value < low ? low : (value > high ? high : value);
}

// One physics iteration based on acceleration vector, returns true if anything moved
static bool ms_iterate(MatrixSand *s, float ax, float ay, float az) {
    // if z dominates, don't do anything
    if (fabsf(az) > fabsf(ax) && fabsf(az) > fabsf(ay)) return false;

    int ix = 0, iy = 0;
    if (fabsf(ax) > 0.01f) {
        float ratio = fabsf(ay / ax);
        if (ratio < 2.414f) ix = (ax > 0) ? 1 : -1;        // tan(67.5°)
        if (ratio > 0.414f) iy = (ay > 0) ? 1 : -1;        // tan(22.5°)
    } else {
        iy = (ay > 0) ? 1 : -1;
    }

    uint8_t new_grains[64];
    memcpy(new_grains, s->grains, sizeof(new_grains));
    bool updated = false;

    for (int x = 0; x < 8; x++) {
        for (int y = 0; y < 8; y++) {
            if (!s->grains[idx(s, x, y)]) continue;

            bool moved = false;
            int newx = x + ix;
            int newy = y + iy;

            newx = clamp_value(newx, 0, 7);
            newy = clamp_value(newy, 0, 7);

            if (x != newx || y != newy) {
                moved = true;
                // blocked?
                if (new_grains[idx(s, newx, newy)]) {
                    bool free_y = !new_grains[idx(s, x, newy)];
                    bool free_x = !new_grains[idx(s, newx, y)];
                    if (free_y && free_x) {
                        int left, right;
                        ms_side_count(s, (ax < 0) && (ay < 0), &left, &right);
                        if (left >= right) {
                            newy = y; // move in x only
                        } else {
                            newx = x; // move in y only
                        }
                    } else if (free_y) {
                        newx = x; // move in y only
                    } else if (free_x) {
                        newy = y; // move in x only
                    } else {
                        moved = false; // totally blocked
                    }
                }
            }

            if (moved) {
                new_grains[idx(s, x, y)] = 0;
                new_grains[idx(s, newx, newy)] = 1;
                updated = true;
            }
        }
    }

    if (updated) memcpy(s->grains, new_grains, sizeof(new_grains));
    return updated;
}

// Build row bytes from two MatrixSand objects and push to both drivers
static void update_matrices(MatrixSand *m1, MatrixSand *m2) {
    uint8_t rows_matrix_1[8] = {0x00}; // upper panel (device nearest)
    uint8_t rows_matrix_2[8] = {0x00}; // lower panel (farther device)
    // [0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00]

    // Map x,y -> row bit; assume bit7 is column 0, bit0 is column 7
    for (int y = 0; y < 8; y++) { //y=0
        for (int x = 0; x < 8; x++) {//x=1
            if (ms_get(m1, x, y)) rows_matrix_1[y] |= (1u << (x)); //(1u << (x)) === 00000001 << 1 === 000000010 
            // |= (1u << (x)) === 000000001 | 000000010 === 00000000011
            if (ms_get(m2, x, y)) rows_matrix_2[y] |= (1u << (x));
        }
    }

    // Only necessary if the matrix is placed in such a way that the (7,7) fot on the nearest and (0,0) on the furthest don't match up
    // Add or remove this for each matrix that needs to be turned
    // rotate90_clockwise helper also exists if necessary
    rotate90_counterclockwise(rows_matrix_1);
    for (int r = 0; r < 8; r++) {
        max7219_write_to_displays(MAX_DIGIT0 + r, rows_matrix_1[r], rows_matrix_2[r]);
    }
}

void app_main(void) {
    // Small delay to let USB serial reconnect after wake-up
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_ERROR_CHECK(i2c_init());
    ESP_ERROR_CHECK(mpu_init());
    max7219_init();

    // Configure MPU motion interrupt (tune as needed)
    // Example: threshold=20 (~40 mg), duration=40 ms
    mpu_config_motion_interrupt(1, 5);

    // Print the wakeup reason
    esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
    switch (wakeup_cause) {
        case ESP_SLEEP_WAKEUP_TIMER:
            ESP_LOGI("TEST", "Woke up from timer");
            break;
        case ESP_SLEEP_WAKEUP_UNDEFINED:
            ESP_LOGI("TEST", "Normal power on or reset");
            break;
        default:
            ESP_LOGI("TEST", "Wakeup caused by %d", wakeup_cause);
            break;
    }
        printf("Normal erhtsrrhjyr, starting deep sleep cycle 2 \n");
            vTaskDelay(pdMS_TO_TICKS(20000)); // Wait before measuring
        printf("Normal boot, starting deep sleep cycle 2 \n");
 fflush(stdout);


    // Two 8x8 matrices, arranged corner-to-corner:
    // touching corner is m1(7,7) with m2(0,0)
    MatrixSand sand1, sand2;

    if (has_saved_state) {
        // Restore saved state from RTC memory after wake-up
        memcpy(&sand1, &saved_sand1, sizeof(MatrixSand));
        memcpy(&sand2, &saved_sand2, sizeof(MatrixSand));
    } else {
        // First boot: initialize matrices to defaults
        ms_init(&sand1);
        ms_init(&sand2);

        // Fill upper panel with sand; remove 3 grains as per original logic
        for (int x = 0; x < 8; x++) {
            for (int y = 0; y < 8; y++) ms_set(&sand1, x, y, true);
        }
        ms_set(&sand1, 0, 0, false);
        ms_set(&sand1, 0, 1, false);
        ms_set(&sand1, 1, 0, false);
        ms_set(&sand1, 0, 2, false);
        ms_set(&sand1, 1, 1, false);
        ms_set(&sand1, 2, 0, false);
    }


    update_matrices(&sand1, &sand2);

    bool updated1 = false, updated2 = false;

    // Clear any pending motion interrupt by reading INT_STATUS after boot or wake
    uint8_t dummy;
    i2c_master_write_read_device(I2C_NUM_0, 0x68, (uint8_t[]){MPU_INT_STATUS}, 1, &dummy, 1, pdMS_TO_TICKS(50));

    last_motion_us = now_us();


    while (1) {
        // Read raw accel (use raw counts; ratios only)
        int16_t axi, ayi, azi;
        if (mpu_read_accel_raw(&axi, &ayi, &azi) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
            continue;
        }

        // Rotate coords to diagonal frame:
        // xx = -ax - ay, yy = -ax + ay, zz = az
        float ax = (float)axi, ay = (float)ayi, az = (float)azi;
        float xx = -az - ax;
        float yy = -az + ax;
        float zz = ay;

        // Corner handoff rule
        if (yy > 0 && ms_get(&sand1, 7, 7) && !ms_get(&sand2, 0, 0) && !updated2) {
            ms_set(&sand1, 7, 7, false);
            ms_set(&sand2, 0, 0, true);
            updated1 = updated2 = true;
        } else if (yy <= 0 && ms_get(&sand2, 0, 0) && !ms_get(&sand1, 7, 7) && !updated1) {
            ms_set(&sand2, 0, 0, false);
            ms_set(&sand1, 7, 7, true);
            updated1 = updated2 = true;
        } else {
            updated1 = ms_iterate(&sand1, xx, yy, zz);
            updated2 = ms_iterate(&sand2, xx, yy, zz);
        }


        if (updated1 || updated2) {
            update_matrices(&sand1, &sand2);
            last_motion_us = now_us();
        }

        updated1 = updated2 = false;
        
        // Inactivity -> deep sleep
        if ((now_us() - last_motion_us) / 1000ULL >= INACTIVITY_MS) {

            max7219_write_to_displays(MAX7219_REG_SHUTDOWN, 0x00, 0x00);
            // Ensure INT is configured (already done) and pending status cleared
            i2c_master_write_read_device(I2C_NUM_0, 0x68, (uint8_t[]){MPU_INT_STATUS}, 1, &dummy, 1, pdMS_TO_TICKS(50));

            memcpy(&saved_sand1, &sand1, sizeof(MatrixSand));
            memcpy(&saved_sand2, &sand2, sizeof(MatrixSand));
            has_saved_state = true;

            // Enter deep sleep; wake on INT high (motion)
            enter_deep_sleep_wait_motion();
            
    // ESP_LOGI(TAG, "Going to sleep now for 10 seconds...");
    // // Configure timer to wake up after 10 seconds
    esp_sleep_enable_timer_wakeup(10 * 1000000ULL);
 
    // // Enter deep sleep
    esp_deep_sleep_start();

        }

        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }
}
