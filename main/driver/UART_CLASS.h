//
// Created by Justin on 06/02/2022.
//

#ifndef FOCX_UART_CLASS_H
#define FOCX_UART_CLASS_H

#include "driver/uart.h"
#include <cstring>
#include "../conf.h"

class UART_CLASS {
public:
    UART_CLASS();
    void init();
    int uart_send(const char *data);
    size_t uart_available();
    int uart_read(char *s, int len);
private:
    uart_config_t uartConfig;
};


#endif //FOCX_UART_CLASS_H
