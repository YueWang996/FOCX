//
// Created by Justin on 02/02/2022.
//

#include "FOC.h"

#ifndef ENABLE_2_CHANNEL_PWM
void FOC::setPWMPins(uint8_t ch1_u, uint8_t ch1_v, uint8_t ch1_w) {
    //printf("Setting pwm pins...\n");
    motorControlPwm.channel1 = {.p1 = ch1_u, .p2 = ch1_v, .p3 = ch1_w};
    motorControlPwm.setChannel1Pins(motorControlPwm.channel1);
}
#else
void FOC::setPWMPins(uint8_t ch1_u, uint8_t ch1_v, uint8_t ch1_w, uint8_t ch2_u, uint8_t ch2_v, uint8_t ch2_w) {
    motorControlPwm.channel1 = {.p1 = ch1_u, .p2 = ch1_v, .p3 = ch1_w};
    motorControlPwm.setChannel1Pins(motorControlPwm.channel1);
    motorControlPwm.channel2 = {.p1 = ch2_u, .p2 = ch2_v, .p3 = ch2_w};
    motorControlPwm.setChannel2Pins(motorControlPwm.channel2);
}
#endif

void FOC::init() {
    //printf("Initializing mcpwm...\n");
    motorControlPwm.init(&Ts);
    tle5012B->init();
}

void FOC::svpwm(float vq, float vd, float angle) {
    vq = fast_constrain(vq, 0, Vlimit);
    float cosVal = fast_cos(angle);
    float sinVal = fast_sin(angle);

    float va = vd * cosVal - vq * sinVal;
    float vb = vq * cosVal + vd * sinVal;

    float v1 = vb;
    float v2 = (_SQRT3 * va - vb) / 2;
    float v3 = (-_SQRT3 * va - vb) / 2;

    uint8_t a, b, c;

    if(v1 > 0) a = 1; else a = 0;
    if(v2 > 0) b = 1; else b = 0;
    if(v3 > 0) c = 1; else c = 0;

    uint8_t sector = (c << 2) | (b << 1) | a;

    switch (sector) {
        case 3:
            t4 = _SQRT3 * Ts * v2 / Vdc;
            t6 = _SQRT3 * Ts * v1 / Vdc;
            t0 = (Ts - t4 - t6) / 2;

            ch1 = t4 + t6 + t0;
            ch2 = t6 + t0;
            ch3 = t0;
            break;
        case 1:
            t4 = -_SQRT3 * Ts * v2 / Vdc;
            t6 = -_SQRT3 * Ts * v3 / Vdc;
            t0 = (Ts - t4 - t6) / 2;

            ch1 = t6 + t0;
            ch2 = t4 + t6 + t0;
            ch3 = t0;
            break;
        case 5:
            t4 = _SQRT3 * Ts * v1 / Vdc;
            t6 = _SQRT3 * Ts * v3 / Vdc;
            t0 = (Ts - t4 - t6) / 2;

            ch1 = t0;
            ch2 = t4 + t6 + t0;
            ch3 = t6 + t0;
            break;
        case 4:
            t4 = -_SQRT3 * Ts * v1 / Vdc;
            t6 = -_SQRT3 * Ts * v2 / Vdc;
            t0 = (Ts - t4 - t6) / 2;
            ch1 = t0;
            ch2 = t6 + t0;
            ch3 = t4 + t6 + t0;
            break;
        case 6:
            t4 = _SQRT3 * Ts * v3 / Vdc;
            t6 = _SQRT3 * Ts * v2 / Vdc;
            t0 = (Ts - t4 - t6) / 2;
            ch1 = t6 + t0;
            ch2 = t0;
            ch3 = t4 + t6 + t0;
            break;
        case 2:
            t4 = -_SQRT3 * Ts * v3 / Vdc;
            t6 = -_SQRT3 * Ts * v1 / Vdc;
            t0 = (Ts - t4 - t6) / 2;
            ch1 = t4 + t6 + t0;
            ch2 = t0;
            ch3 = t6 + t0;
            break;
        default:
            ch1 = 0;
            ch2 = 0;
            ch3 = 0;
    }
    ch1 = fast_constrain(ch1, 0, Ts);
    ch2 = fast_constrain(ch2, 0, Ts);
    ch3 = fast_constrain(ch3, 0, Ts);
    motorControlPwm.setChannel1Duty(ch1 * 100.0 / Ts, ch2 * 100.0 / Ts, ch3 * 100.0 / Ts);
    //printf("CH1:%f, CH2:%f, CH3:%f\n", ch1, ch2, ch3);
}

void FOC::setVoltage(float Vdc, float Vlim) {
    this->Vdc = Vdc;
    this->Vlimit = Vlim;
}

void FOC::setPolePares(uint8_t pole_pares) {
    this->pole_pares = pole_pares;
}

float FOC::getEAngle() {
    tle5012B->update();
    printf("abs angle: %f\n", tle5012B->getAbsoluteAngleValue());
    return tle5012B->getAbsoluteAngleValue() * pole_pares;
}

void FOC::setSensor(TLE5012B *tle5012B) {
    this->tle5012B = tle5012B;
}

FOC::FOC() {
    motorControlPwm = MotorControlPWM();
}
