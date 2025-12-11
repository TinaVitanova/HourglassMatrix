# ESP32 Matrix Sand Clock 🕰️

[![ESP32](https://img.shields.io/badge/ESP32-Sand%20Clock-blue?style=flat-square&logo=espressif)](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.x-green?style=flat-square&logo=esp-idf)](https://docs.espressif.com/projects/esp-idf/en/latest/)
[![License](https://img.shields.io/badge/License-MIT-yellow?style=flat-square)](LICENSE)

**ESP32 firmware simulating a realistic sand clock/hourglass** using **two 8x8 LED dot matrices**. MPU6050 accelerometer detects tilt, moving "sand" grains between matrices with physics simulation, corner handoff, and ultra-low-power deep sleep.

> Tilt to pour sand between matrices! Perfect for electronics teaching demos 🎓

## ✨ Features

- **Dual 8x8 MAX7219** dot matrices (corner-to-corner layout)
- **MPU6050 motion detection** with interrupt-driven deep sleep (~5s inactivity)
- **RTC memory persistence** - state survives power cycles
- **Realistic physics**: diagonal tilt compensation, collision resolution
- **Corner grain handoff** between touching matrices
- **Ultra-low power**: GPIO wakeup, MAX7219 shutdown

## 🛠️ Hardware Connections

| Component | Pin | ESP32 GPIO |
|-----------|-----|------------|
| **MPU6050** | VCC | 3.3V |
| | GND | GND |
| | SDA | **GPIO8** |
| | SCL | **GPIO9** |
| | INT | **GPIO0** (wakeup) |
| **MAX7219 #1** (Upper) | DIN | **GPIO5** |
| | CS | **GPIO2** |
| | CLK | **GPIO4** |
| **MAX7219 #2** (Lower) | DIN | DOUT of #1 |
| | CS | CS OUT of #1 |
| | CLK | CLK OUT of #1 |


## 🔬 Physics Engine

1. **Read raw accel** `(axi, ayi, azi)`
2. **Rotate to diagonal frame**: `xx=-az-ax, yy=-az+ax, zz=ay`
3. **Compute move direction** from tilt ratios `tan(22.5°/67.5°)`
4. **Collision resolution**: side-count priority `(left≥right → X-axis)`
5. **Corner handoff**: grain transfers at touching corner
6. **Update matrices** → MAX7219

## ⚡ Power Management

| State | Trigger | Action |
|-------|---------|--------|
| **Active** | Motion detected | Physics + display update |
| **Sleep** | 5s inactivity | Save RTC, MAX7219 off, deep sleep |
| **Wake** | MPU INT (GPIO0 HIGH) | Restore RTC, resume physics |
