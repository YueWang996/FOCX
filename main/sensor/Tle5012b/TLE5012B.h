//
// Created by Justin on 01/02/2022.
//

#ifndef FOCX_TLE5012B_H
#define FOCX_TLE5012B_H

#include "../../driver/SPI_CLASS.h"
#include "esp_timer.h"

#define REG_AVAL 0x0020U
#define REG_ASPD 0x0030U
#define REG_AREV 0x0040U
#define REG_MODE_2 0x0080U
#define REG_MODE_3 0x0090U

#define UDP_LOW  0x0000
#define SAFE_HIGH 0x0001

#define READ_SENSOR 0x8000
#define WRITE_SENSOR 0x5000

#define DELETE_7BITS 0x01FF
#define DELETE_BIT_15 0x7FFF
#define CHECK_BIT_9 0x0100
#define CHECK_BIT_14 0x4000
#define CHANGE_UINT_TO_INT_15 0x8000
#define CHANGE_UNIT_TO_INT_9 0x0200
#define GET_BIT_14_4 0x7FF0

#define ANGLE_360_VAL 360.0
#define ANGLE_2PI_VAL 6.28318530718
#define ANGLE_PI_VAL 3.14159265358979323846
#define POW_2_15 32768.0
#define RAW_ANGLE_TO_RAD 0.00019174759

class TLE5012B {
public:
    TLE5012B(uint8_t miso, uint8_t mosi, uint8_t clk, uint8_t cs);
    void init();

    void update();
    double getAngularVelocity();
    double getAbsoluteAngleValue();
    double getAngleValue();
    int16_t getRevolutionValue();

private:
    void getAngle(double *angle);
    void getRevolution(int16_t *revolution);

    MySPI mySpi;

    uint8_t miso;
    uint8_t mosi;
    uint8_t clk;
    uint8_t cs;

    uint64_t last_time_stamp;
    uint64_t current_time_stamp;

    double last_abs_angle;
    double current_abs_angle;
    double last_angle;
    double current_angle;
    int16_t revolution;
    double velocity;
};


#endif //FOCX_TLE5012B_H
