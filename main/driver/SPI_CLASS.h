//
// Created by Justin on 29/01/2022.
//

#ifndef FOCX_SPI_CLASS_H
#define FOCX_SPI_CLASS_H

#include "driver//spi_master.h"
#include "driver/gpio.h"
#include "esp32/rom/ets_sys.h"

#define DMA_CHAN 2

class MySPI {
public:
    MySPI();
    void set_pins(uint8_t miso, uint8_t mosi, uint8_t clk, uint8_t cs);
    void init();

    esp_err_t spi_write(spi_device_handle_t spi, uint8_t *data, uint8_t len);
    esp_err_t spi_read(spi_device_handle_t spi, uint8_t *data);
    esp_err_t write_and_read(spi_device_handle_t spi, uint8_t *writeData, uint8_t len, uint8_t *readData);

private:
    uint8_t miso;
    uint8_t mosi;
    uint8_t clk;
    uint8_t cs;

    spi_bus_config_t spiBusConfig;
    spi_device_handle_t spiDeviceHandle;
    spi_device_interface_config_t spiDeviceInterfaceConfig;
};


#endif //FOCX_SPI_CLASS_H
