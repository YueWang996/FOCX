//
// Created by Justin on 02/02/2022.
//

#ifndef FOCX_FOC_H
#define FOCX_FOC_H

#include "../driver/MotorControlPWM.h"
#include "../sensor/Tle5012b/TLE5012B.h"
#include "foc_utilities.h"
#include "../conf.h"

class FOC {
public:
    FOC();
#ifndef ENABLE_2_CHANNEL_PWM
    void setPWMPins(uint8_t ch1_u, uint8_t ch1_v, uint8_t ch1_w);
#else
    void setPWMPins(uint8_t ch1_u, uint8_t ch1_v, uint8_t ch1_w, uint8_t ch2_u, uint8_t ch2_v, uint8_t ch2_w);
#endif

    void setVoltage(float Vdc, float Vref);
    void setPolePares(uint8_t pole_pares);
    void setSensor(TLE5012B *tle5012B);

    void init();

    float getEAngle();

    void svpwm(float vq, float vd, float angle);

private:

    MotorControlPWM motorControlPwm;
    TLE5012B *tle5012B;

    int Ts;
    float t0, t4, t6;
    float ch1, ch2, ch3;

    float Vdc;
    float Vlimit;

    float angle;
    float e_angle;
    uint8_t pole_pares;
};


#endif //FOCX_FOC_H
