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

    void svpwm(float vq, float vd);
    void spwm(float vq, float vd, float angle);

    void motorStart(float target);

    void getEAngle();
    void alignElectricalAngle();

    MotorControlPWM motorControlPwm;

    // PID things
    PIDControlParameters idPID;
    PIDControlParameters iqPID;
    float target_id = 0;
private:
    TLE5012B *tle5012B;

    void ClarkeParkTransform();

    // SVPWM Parameters
    int Ts;
    float t0, t4, t6;
    float ch1, ch2, ch3;
    float cosVal;
    float sinVal;

    // Power Parameters
    float Vdc;
    float Vlimit;
    float Valign = 0.5;

    // Angles
    float angle;
    float e_angle;
    float zero_electrical_angle;
    uint8_t pole_pares;
    int8_t direction = SensorDirection::CW;

    // Foc currents
    PhaseVoltage phaseVoltage;
    float id, iq;

    // Low-pass filter
    float lpf_ratio = 0.1;
    float lpf_last_phaseVoltageU;
    float lpf_last_phaseVoltageV;
    float lpf_last_id;
    float lpf_last_iq;

};


#endif //FOCX_FOC_H
