ESP32 Matrix Sand Clock
ESP32 firmware simulating a sand clock/hourglass using two 8x8 LED dot matrices. MPU6050 accelerometer detects tilt, moving "sand" grains between matrices with realistic physics and deep sleep power saving.

Features
Dual 8x8 MAX7219 dot matrix displays arranged corner-to-corner

MPU6050 motion detection with interrupt-driven deep sleep (~5s inactivity)

RTC memory state persistence across power cycles

Realistic sand physics with diagonal tilt compensation

Corner grain handoff between matrices

Low-power operation with GPIO wakeup

Hardware Setup
text
ESP32 connections:
├── MPU6050 (I2C): SDA=GPIO21, SCL=GPIO22, INT=GPIO0
├── MAX7219 #1 (SPI, nearest): DIN=GPIO23, CS=GPIO18, CLK=GPIO19
├── MAX7219 #2 (SPI, farther):  DIN=GPIO13, CS=GPIO5, CLK=GPIO19 (shared)
└── Physical layout: Matrix1(7,7) touches Matrix2(0,0)
Software Dependencies
ESP-IDF v5.x

mpu_6050.h - MPU6050 driver (I2C motion interrupt)

dot_matrix.h - Dual MAX7219 driver (max7219_write_to_displays())

FreeRTOS, SPI, I2C, esp_timer, deep sleep APIs

Physics Simulation
Sand grains move based on accelerometer vector:

Primary direction from tilt angle ratios

Collision resolution with side-counting priority

Coordinate rotation for diagonal matrix frame

Grain transfer at touching corner (1,7)-(2,0)

Power Management
text
Inactivity timeout: 5 seconds
Wake sources: MPU INT (GPIO0 high), timer fallback
RTC saves: MatrixSand states (128 bytes total)
Shutdown: MAX7219s powered off before sleep
Build & Flash
bash
idf.py menuconfig  # Verify pins/I2C config
idf.py build flash monitor
Initial boot fills Matrix1 with sand (minus 7 corner grains). Tilt to pour between matrices.

Customization
INACTIVITY_MS: Adjust sleep sensitivity

mpu_config_motion_interrupt(1,5): Threshold=1g/20, duration=5*20ms

DELAY_MS: Physics update rate (200ms default)

Matrix rotation: Toggle rotate90_counterclockwise() calls
