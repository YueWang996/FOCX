//
// Created by Justin on 02/02/2022.
//

#include <freertos//FreeRTOS.h>
#include <freertos/task.h>
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
    motorControlPwm.setChannel1Duty(50, 50, 50);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    motorControlPwm.adc_calibration();

    tle5012B->init();

    alignElectricalAngle();
}

void FOC::svpwm(float vq, float vd) {
    vq = fast_constrain(vq, -Vlimit, Vlimit);
    //float cosVal = fast_cos(angle);
    //float sinVal = fast_sin(angle);
    float va = vd * cosVal - vq * sinVal;
    float vb = vq * cosVal + vd * sinVal;

    float v1 = vb;
    float v2 = (_SQRT3 * va - vb) / 2;
    float v3 = (-_SQRT3 * va - vb) / 2;

    uint8_t a, b, c;

    if(v1 > 0) a = 1; else a = 0;
    if(v2 > 0) b = 1; else b = 0;
    if(v3 > 0) c = 1; else c = 0;

    uint8_t sector = (c << 2) + (b << 1) + a;

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
    //printf("CH1:%f, CH2:%f, CH3:%f, sector:%d\n", ch1, ch2, ch3, sector);
    ch1 = fast_constrain(ch1, 0, Ts);
    ch2 = fast_constrain(ch2, 0, Ts);
    ch3 = fast_constrain(ch3, 0, Ts);
    motorControlPwm.setChannel1Duty(ch1 * 100.0 / Ts, ch2 * 100.0 / Ts, ch3 * 100.0 / Ts);
}

void FOC::setVoltage(float Vdc, float Vlim) {
    this->Vdc = Vdc;
    this->Vlimit = Vlim;
}

void FOC::setPolePares(uint8_t pole_pares) {
    this->pole_pares = pole_pares;
}

void FOC::getEAngle() {
    tle5012B->update();
    //printf("abs angle: %f\n", tle5012B->getAbsoluteAngleValue());
    angle = normalise_angle((float) (pole_pares * direction) * (float) tle5012B->getAbsoluteAngleValue() - zero_electrical_angle);
}

void FOC::setSensor(TLE5012B *tle5012B) {
    this->tle5012B = tle5012B;
}

FOC::FOC() {
    motorControlPwm = MotorControlPWM();
}


void FOC::ClarkeParkTransform() {
    phaseVoltage = motorControlPwm.getADCVoltage();

    // Clarke transform
    float i_alpha = CH1_VOLTAGE2CURRENT(phaseVoltage.U);
    float i_beta = _1_SQRT3 * CH1_VOLTAGE2CURRENT(phaseVoltage.U) + _2_SQRT3 * CH1_VOLTAGE2CURRENT(phaseVoltage.V);

    // Park transform
    id = i_alpha * cosVal + i_beta * sinVal;
    iq = i_beta * cosVal - i_alpha * sinVal;
    id = LPF(id, lpf_last_id, lpf_ratio_id);
    iq = LPF(iq, lpf_last_iq, lpf_ratio_iq);
    lpf_last_id = id;
    lpf_last_iq = iq;
}

void FOC::motorStart(float target) {
    switch (mode) {
        case Torque: {
            // Get electrical angle
            getEAngle();
            angle = LPF(angle, lpf_last_angle, lpf_ratio_angle);
            cosVal = fast_cos(angle);
            sinVal = fast_sin(angle);
            lpf_last_angle = angle;

            ClarkeParkTransform();

            // Current loop PID;
            float vd = PIDController(idPID, -id);
            float vq = PIDController(iqPID, target - iq);

            //printf("id:%.2f,iq:%.2f\n", id, iq);
            svpwm(vq, vd);
            break;
        }
        case Velocity: {
            break;
        }
        case Position: {
            tle5012B->update();
            realAngle = (float) tle5012B->getAngleValue() * (float) direction;
            realAngle = LPF(realAngle, lpf_last_angle, lpf_ratio_angle);

            angle = normalise_angle((float) (pole_pares) * realAngle - zero_electrical_angle);
            cosVal = fast_cos(angle);
            sinVal = fast_sin(angle);
            lpf_last_angle = realAngle;

            ClarkeParkTransform();

            float target_iq = PIDController(positionPID, target - realAngle);

            // Current loop PID;
            float vd = PIDController(idPID, -id);
            float vq = PIDController(iqPID, target_iq - iq);
            //printf("vd:%f,vq:%f\n", vd, vq);

            svpwm(vq, vd);
            break;
        }
    }
}

void FOC::alignElectricalAngle() {
    // Find sensor direction
    tle5012B->update();
    float temp_angle = tle5012B->getAngleValue();
    for (int i = 0; i <= 500; i++) {
        float a = _3PI_2 + _2PI * i / 500.0f;
        spwm(Valign, 0, a);
        vTaskDelay(3 / portTICK_PERIOD_MS);
    }
    tle5012B->update();
    if (tle5012B->getAngleValue() > temp_angle) direction = SensorDirection::CW;
    else direction = SensorDirection::CCW;
    for (int i = 500; i >= 0; i--) {
        float a = _3PI_2 + _2PI * i / 500.0f;
        spwm(Valign, 0, a);
        vTaskDelay(3 / portTICK_PERIOD_MS);
    }

    // Align mechanical angle and electrical angle
    spwm(Valign, 0, _3PI_2);
    vTaskDelay(700 / portTICK_PERIOD_MS);
    tle5012B->update();
    zero_electrical_angle = 0;
    getEAngle();    // update angle value
    zero_electrical_angle = angle;
    lpf_last_angle = angle;
    motorControlPwm.setChannel1Duty(0, 0, 0);
}

void FOC::spwm(float vq, float vd, float angle) {
    // Sinusoidal PWM modulation
    // Inverse Park + Clarke transformation

    // angle normalization in between 0 and 2pi
    // only necessary if using _sin and _cos - approximation functions
    float angle_el = normalise_angle(angle);
    // Inverse park transform
    float Ualpha =  cosf(angle_el) * vd - sinf(angle_el) * vq;  // -sin(angle) * Uq;
    float Ubeta =  sinf(angle_el) * vd + cosf(angle_el) * vq;    //  cos(angle) * Uq;

    // center = modulation_centered ? (driver->voltage_limit)/2 : Uq;
    float center = Vlimit / 2;
    // Clarke transform
    float Ua = Ualpha + center;
    float Ub = -0.5f * Ualpha  + _SQRT3_2 * Ubeta + center;
    float Uc = -0.5f * Ualpha - _SQRT3_2 * Ubeta + center;

    //if (!modulation_centered) {
        float Umin = fminf(Ua, fminf(Ub, Uc));
        Ua -= Umin;
        Ub -= Umin;
        Uc -= Umin;
    //}
    motorControlPwm.setChannel1Duty(Ua * 100.0 / Vdc, Ub * 100.0 / Vdc, Uc * 100.0 / Vdc);

}

float FOC::getZeroEAngle() {
    return zero_electrical_angle;
}
