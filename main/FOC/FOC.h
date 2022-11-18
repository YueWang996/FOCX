//
// Created by Justin on 02/02/2022.
//

#ifndef FOCX_FOC_H
#define FOCX_FOC_H

#include "../driver/MotorControlPWM.h"
#include "../sensor/Tle5012b/TLE5012B.h"
#include "foc_utilities.h"
#include "../conf.h"
#include "LowPass.h"

class FOC {
public:
    /// Instantiate FOC driver
    /// \param ch Channel of driver. There are two channels are available on FOC X v5.0 board.
    FOC(Channel ch);

    /// Set PWM pins. Don't use!
    void setPWMPins(uint8_t ch1_u, uint8_t ch1_v, uint8_t ch1_w);
    /// Set ADC pins. Don't use!
    void setADCPins(uint8_t pinA, uint8_t pinB);

    /// Set voltages
    /// \param Vdc Battery voltage
    /// \param Vlim Output voltage limitation
    void setVoltage(float Vdc, float Vlim);

    /// Configure pole pares
    /// \param pole_pares Number of pole pare
    void setPolePares(uint8_t pole_pares);

    /// Connect magnetic sensor
    /// \param tle5012B Pointer to initialised sensor
    /// \note Sensor must be initialised!
    void setSensor(TLE5012B *tle5012B);

    /// Connect motor control PWM unit
    /// \param mcp Pointer to motor control PWM unit
    /// \note MotorControlPWM must be initialised!
    void setPWMController(MotorControlPWM *mcp);

    /// Set SVPWM period
    /// \param ts SVPWM period. When MCPWM is initialised, this value can be get.
    void setTs(int ts);

    /// Initialise FOC driver
    void init();

    /// Space Vector PWM output unit. Inverse Clarke transform has been integrated.
    /// \param vq q-axis voltage
    /// \param vd d-axis voltage
    void svpwm(float vq, float vd);

    /// Sinusoidal PWM output unit.
    /// \param vq q-axis voltage
    /// \param vd d-axis voltage
    /// \param angle angle
    /// \note SPWM has lower efficiency than SVWPM, so this unit is not recommanded.
    void spwm(float vq, float vd, float angle);

    /// Start FOC loop. Torque, Velocity and Position mode are supported.
    /// \param target set the target torque, velocity or position.
    void motorStart();

    /// Get electrical angle
    void getEAngle();

    /// Get velocity of the shaft (rad/s)
    void getVelocity();

    /// Get zero electrical angle. Can be used to identify zero electrical so in the real project, magnetic angle align can be ignored.
    /// \return Zero electrical angle.
    float getZeroEAngle();

    /// Align electrical angle
    void alignElectricalAngle();

    /// Pointer to MCPWM
    MotorControlPWM *motorControlPwm;

    /// PID configuration
    PIDControlParameters idPID;
    PIDControlParameters iqPID;
    PIDControlParameters velocityPID;
    PIDControlParameters positionPID;

    //float lpf_ratio_angle = 0.01;

    /// FOC mode setting. Default: Torque
    FOC_MODE mode = FOC_MODE::Torque;

    /// Low-pass filters
    LowPass<2> ch1_id_lp = LowPass<2>(3, 1886, true);   // Second order low-pass filter
    LowPass<2> ch1_iq_lp = LowPass<2>(3, 1886, true);
    //LowPass<2> ch1_angle_lp = LowPass<2>(2, 1886, true);
    LowPass<2> ch1_velocity_lp = LowPass<2>(2, 1886, true);

    LowPass<2> ch2_id_lp = LowPass<2>(3, 1886, true);   // Second order low-pass filter
    LowPass<2> ch2_iq_lp = LowPass<2>(3, 1886, true);
    //LowPass<2> ch2_angle_lp = LowPass<2>(2, 1886, true);
    LowPass<2> ch2_velocity_lp = LowPass<2>(2, 1886, true);

    /// Foc current and voltage
    PhaseVoltage phaseVoltage;
    float id, iq;

    /// Pointer to magnetic sensor
    TLE5012B *tle5012B;

    /// Magnetic sensor variables
    float angle;
    float realAngle;
    float velocity;
    float zero_electrical_angle;
    uint8_t pole_pares;
    int8_t direction = SensorDirection::CW;

    float target = 0.0;

private:
    /// Channel to use
    Channel channel;

    /// Clarke and Park transform. It will read two of three phase voltage
    /// and then calculate phase currents and convert them to DQ current
    void ClarkeParkTransform();

    /// SVPWM parameters
    int Ts;
    float t0, t4, t6;
    float ch1, ch2, ch3;
    float cosVal;
    float sinVal;

    /// Power supply arameters
    float Vdc;
    float Vlimit;
    float Valign = 0.5;

};


#endif //FOCX_FOC_H
