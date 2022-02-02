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

void MySPI::spi_init() {
    //----------SPI CONFIG-----------//
    this->spiBusConfig = {
            .mosi_io_num        =       this->mosi,//MOSI信号线，可复用为QSPI的D1
            .miso_io_num        =       this->miso,//MISO信号线，可复用为QSPI的D0
            .sclk_io_num        =       this->clk,//SCLK信号线
            .quadwp_io_num      =       -1,//WP信号线，专用于QSPI的D2
            .quadhd_io_num      =       -1,//HD信号线，专用于QSPI的D3
            .max_transfer_sz    =       64*8,//最大传输数据大小，单位字节，默认为4094
            //.intr_flags         =       UNSET//中断指示位
    };

    this->spiDeviceInterfaceConfig = {
            .mode           = 0,                                    // SPI mode 0
            .clock_speed_hz = APB_CLK_FREQ/80,                      //  Clock at 1 MHz
            //.clock_speed_hz = SPI_MASTER_FREQ_10M,                // Clock at 10 MHz
            .input_delay_ns = 500,
            .spics_io_num   = -1,
            .queue_size     = 3,                                    // 传输队列大小，决定了等待传输数据的数量
    };

    //Initialize the SPI bus
    esp_err_t ret;

    printf("Initializing spi bus...");
    ret = spi_bus_initialize(SPI3_HOST, &this->spiBusConfig, DMA_CHAN);
    ESP_ERROR_CHECK(ret);

    printf("Adding spi bus device...");
    ret = spi_bus_add_device(SPI3_HOST, &this->spiDeviceInterfaceConfig, &this->spiDeviceHandle);
    ESP_ERROR_CHECK(ret);

    gpio_pad_select_gpio(this->cs);                // 选择一个GPIO
    gpio_set_direction(static_cast<gpio_num_t>(this->cs), GPIO_MODE_OUTPUT);// 把这个GPIO作为输出
}

void MySPI::spi_send_receive(uint16_t *send, uint8_t slen, uint16_t *receive, uint8_t rlen) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    //send phase
    t.length = slen;
    t.tx_buffer = send;
    gpio_set_level(static_cast<gpio_num_t>(this->cs), 0);
    spi_device_transmit(spiDeviceHandle, &t);

    //receive phase
    memset(&t, 0, sizeof(t));
    t.length = slen;
    t.tx_buffer = nullptr;
    t.rx_buffer = receive;
    t.rxlength = rlen;
    spi_device_transmit(spiDeviceHandle, &t);
    gpio_set_level(static_cast<gpio_num_t>(this->cs), 1);
}

void MySPI::spi_send(uint16_t *send, uint8_t slen) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    //send phase
    t.length = slen;
    t.tx_buffer = send;
    gpio_set_level(static_cast<gpio_num_t>(this->cs), 0);
    spi_device_transmit(spiDeviceHandle, &t);
    gpio_set_level(static_cast<gpio_num_t>(this->cs), 1);
}
