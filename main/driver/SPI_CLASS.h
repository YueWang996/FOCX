//
// Created by Justin on 29/01/2022.
//

#ifndef FOCX_SPI_CLASS_H
#define FOCX_SPI_CLASS_H

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp32/rom/ets_sys.h"
#include "../print_utils.h"

#define DMA_CHAN 2

class MySPI {
public:
    MySPI();
    void set_pins(uint8_t miso, uint8_t mosi, uint8_t clk, uint8_t cs);
    void spi_init();

    void spi_send(uint16_t *send, uint8_t slen);
    void spi_send_receive(uint16_t *send, uint8_t slen, uint16_t *receive, uint8_t rlen);


private:
    uint8_t miso;
    uint8_t mosi;
    uint8_t clk;
    uint8_t cs;

    spi_bus_config_t spiBusConfig;
    spi_device_interface_config_t spiDeviceInterfaceConfig;
    spi_device_handle_t spiDeviceHandle;
};


#endif //FOCX_SPI_CLASS_H
