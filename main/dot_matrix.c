#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "esp_check.h"

#define MOSI_PIN 5
#define SCLK_PIN 4
#define CS_PIN 2

#define MAX7219_REG_DIGIT0 0x01
#define MAX7219_REG_DECODE 0x09
#define MAX7219_REG_INTENSITY 0x0A
#define MAX7219_REG_SCANLIMIT 0x0B
#define MAX7219_REG_SHUTDOWN 0x0C
#define MAX7219_REG_DISPLAYTEST 0x0F

static spi_device_handle_t spi;

void max7219_broadcast(uint8_t addr, uint8_t data) {
    uint8_t buf[4] = { addr, data, addr, data };
    spi_transaction_t t = {
        .length = 32,
        .tx_buffer = buf,
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &t));
}

void max7219_write_to_displays(uint8_t addr, uint8_t data1, uint8_t data2) {
    uint8_t buf[4] = { addr, data1, addr, data2 };
    spi_transaction_t t = {
        .length = 32,
        .tx_buffer = buf,
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &t));
}

void max7219_clear(void) {
    for (uint8_t r = 0; r < 8; r++) {
        max7219_broadcast(MAX7219_REG_DIGIT0 + r, 0x00);
    }
}

void max7219_init(void) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = MOSI_PIN,
        .miso_io_num = -1,
        .sclk_io_num = SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED));

    spi_device_interface_config_t devcfg = {
        .mode = 0,
        .clock_speed_hz = 1000000,
        .spics_io_num = CS_PIN,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));

    max7219_broadcast(MAX7219_REG_DISPLAYTEST, 0x00);
    max7219_broadcast(MAX7219_REG_SHUTDOWN, 0x01);
    max7219_broadcast(MAX7219_REG_DECODE, 0x00);
    max7219_broadcast(MAX7219_REG_SCANLIMIT, 0x07);
    max7219_broadcast(MAX7219_REG_INTENSITY, 0x08);

    max7219_clear();
}