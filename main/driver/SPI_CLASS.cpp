//
// Created by Justin on 29/01/2022.
//

#include <cstring>
#include "SPI_CLASS.h"

void MySPI::set_pins(uint8_t miso, uint8_t mosi, uint8_t clk, uint8_t cs) {
    this->miso = miso;
    this->mosi = mosi;
    this->clk = clk;
    this->cs = cs;
}

MySPI::MySPI() {

}

void MySPI::init() {
    spiBusConfig = {
            .mosi_io_num = mosi,
            .miso_io_num = miso,
            .sclk_io_num = clk,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 64*8
    };

    spiDeviceInterfaceConfig = {
            .mode = 0,
            .clock_speed_hz = SPI_MASTER_FREQ_10M,
            .spics_io_num = -1,
            .queue_size = 7
    };

    esp_err_t ret;
    // 初始化SPI总线
    ret=spi_bus_initialize(SPI2_HOST, &spiBusConfig, DMA_CHAN);
    ESP_ERROR_CHECK(ret);
    // 添加SPI总线驱动
    ret=spi_bus_add_device(SPI2_HOST, &spiDeviceInterfaceConfig, &spiDeviceHandle);
    ESP_ERROR_CHECK(ret);

    gpio_pad_select_gpio(cs);                // 选择一个GPIO
    gpio_set_direction(static_cast<gpio_num_t>(cs), GPIO_MODE_OUTPUT);// 把这个GPIO作为输出

}

esp_err_t MySPI::spi_write(spi_device_handle_t spi, uint8_t *data, uint8_t len) {
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) {
        ret = ESP_FAIL;
        return ret;
    }             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction

    gpio_set_level(static_cast<gpio_num_t>(cs), 0);

    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.

    gpio_set_level(static_cast<gpio_num_t>(cs), 1);
    return ret;
}

esp_err_t MySPI::spi_read(spi_device_handle_t spi, uint8_t *data) {
    spi_transaction_t t;

    gpio_set_level(static_cast<gpio_num_t>(cs), 0);

    memset(&t, 0, sizeof(t));
    t.length=8;
    t.flags = SPI_TRANS_USE_RXDATA;
    t.user = (void*)1;
    esp_err_t ret = spi_device_polling_transmit(spi, &t);
    assert( ret == ESP_OK );
    *data = t.rx_data[0];

    gpio_set_level(static_cast<gpio_num_t>(cs), 1);

    return ret;
}

esp_err_t MySPI::write_and_read(spi_device_handle_t spi, uint8_t *writeData, uint8_t len, uint8_t *readData) {
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) {
        ret = ESP_FAIL;
        return ret;                 //no need to send anything
    }
    memset(&t, 0, sizeof(t));       //Zero out the transaction

    gpio_set_level(static_cast<gpio_num_t>(cs), 0);

    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=writeData;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!

    gpio_set_level(static_cast<gpio_num_t>(cs), 1);
    gpio_set_level(static_cast<gpio_num_t>(cs), 0);

    ets_delay_us(1);

    gpio_set_level(static_cast<gpio_num_t>(cs), 1);
    memset(&t, 0, sizeof(t));
    gpio_set_level(static_cast<gpio_num_t>(cs), 0);

    t.length=8;
    t.flags = SPI_TRANS_USE_RXDATA;
    t.user = (void*)1;
    ret = spi_device_polling_transmit(spi, &t);
    assert( ret == ESP_OK );
    *readData = t.rx_data[0];

    gpio_set_level(static_cast<gpio_num_t>(cs), 1);

    return ret;
}
