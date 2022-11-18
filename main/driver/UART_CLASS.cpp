//
// Created by Justin on 06/02/2022.
//

#include "UART_CLASS.h"

#define RX_BUF_SIZE 1024
#define TX_PIN 1
#define RX_PIN 3

UART_CLASS::UART_CLASS() {
    uartConfig = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk= UART_SCLK_APB
    };

}

void UART_CLASS::init() {
    uart_driver_install(UART_NUMBER, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUMBER, &uartConfig);
    uart_set_pin(UART_NUMBER, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

int UART_CLASS::uart_send(const unsigned char *data, int data_len) {
    //const int data_len = strlen(reinterpret_cast<const char *>(data));
    const int txBytes = uart_write_bytes(UART_NUMBER, data, data_len);
    uart_flush(UART_NUMBER);
    return txBytes;
}

size_t UART_CLASS::uart_available() {
    size_t size;
    uart_get_buffered_data_len(UART_NUMBER, &size);
    return size;
}

int UART_CLASS::uart_read(uint8_t *s, int len) {
    char *data = (char *) pvPortMalloc(RX_BUF_SIZE+1);
    const int rxBytes = uart_read_bytes(UART_NUMBER, data, RX_BUF_SIZE * 2, 20 / portTICK_RATE_MS);
    if(rxBytes > 0) {
        data[rxBytes] = 0;

        // reset string
        memset(s,'\0',sizeof(char) * len);
        strncat(reinterpret_cast<char *>(s), data, len);

        //uart_flush(UART_NUMBER);
        return 1;
    }
    vPortFree(data);
    return 0;
}
