//
// Created by Justin on 01/02/2022.
//

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "TLE5012B.h"

TLE5012B::TLE5012B(uint8_t cs) {
    this->cs = cs;
}

void TLE5012B::init() {
    //mySpi.set_pins(cs);
    //mySpi.spi_init();

    getAngle(&current_abs_angle);
    getAngle(&last_abs_angle);

    last_time_stamp = esp_timer_get_time();

    getRevolution(&revolution);
    last_angle = ANGLE_2PI_VAL * revolution + last_abs_angle;

    update();
}

void TLE5012B::getAngle(double *angle) {
    uint16_t command = READ_SENSOR | REG_AVAL | UDP_LOW | SAFE_HIGH;
    uint16_t data, received;

    data = SPI_SWAP_DATA_TX(command, 16);

    //if(!mySpi->spi_is_reading) {
    //    mySpi->spi_send_receive(&data, 16, &received, 16, cs);
    //} else {
    //    vTaskDelay(2/portTICK_PERIOD_MS);
    //    if(!mySpi->spi_is_reading) {
    //        mySpi->spi_send_receive(&data, 16, &received, 16, cs);
    //    } else {
    //        return;
    //    }
    //}
    //while(mySpi->spi_is_reading);
    mySpi->spi_send_receive(&data, 16, &received, 16, cs);



    received = SPI_SWAP_DATA_RX(received, 16);
    received &= DELETE_BIT_15;
    if (received & CHECK_BIT_14)
    {
        received = received - CHANGE_UINT_TO_INT_15;
        *angle = ANGLE_2PI_VAL + (RAW_ANGLE_TO_RAD * ((int16_t) received));
    } else {
        *angle = RAW_ANGLE_TO_RAD * ((int16_t) received);
    }
}

void TLE5012B::getRevolution(int16_t *revolution) {
    uint16_t command = READ_SENSOR | REG_AREV | UDP_LOW | SAFE_HIGH;
    uint16_t data, received;

    data = SPI_SWAP_DATA_TX(command, 16);

    //while(!mySpi->spi_is_reading);
    //mySpi->spi_send_receive(&data, 16, &received, 16, cs);
    if(!mySpi->spi_is_reading) {
        mySpi->spi_send_receive(&data, 16, &received, 16, cs);
    } else {
        vTaskDelay(10/portTICK_PERIOD_MS);
        if(!mySpi->spi_is_reading) {
            mySpi->spi_send_receive(&data, 16, &received, 16, cs);
        } else {
            return;
        }
    }

    received = SPI_SWAP_DATA_RX(received, 16);
    received &= DELETE_7BITS;
    if (received & CHECK_BIT_9)
    {
        received = received - CHANGE_UNIT_TO_INT_9;
    }

    *revolution = (int16_t) received;
}

void TLE5012B::update() {
    getAngle(&current_abs_angle);
    current_time_stamp = esp_timer_get_time();

    getRevolution(&revolution);
    current_angle = ANGLE_2PI_VAL * revolution + current_abs_angle;

    velocity = (current_angle - last_angle) / ((double)(current_time_stamp - last_time_stamp) / 1000000.0);

    last_abs_angle = current_abs_angle;
    last_angle = current_angle;
    last_time_stamp = current_time_stamp;
}

double TLE5012B::getAngleValue() {
    return current_angle;
}

int16_t TLE5012B::getRevolutionValue() {
    return revolution;
}

double TLE5012B::getAngularVelocity() {
    return velocity;
}

double TLE5012B::getAbsoluteAngleValue() {
    return current_abs_angle;
}

void TLE5012B::setSPI(MySPI *spi) {
    mySpi = spi;
}

