//
// Created by Justin on 02/02/2022.
//

#include <freertos//FreeRTOS.h>
#include <freertos/task.h>
#include "FOC.h"

FOC::FOC(Channel ch) {
    channel = ch;
}

void FOC::setPWMPins(uint8_t ch1_u, uint8_t ch1_v, uint8_t ch1_w) {
    //printf("Setting pwm pins...\n");
#ifdef ENABLE_2_CHANNEL_PWM
    if(channel == Channel::CH1) {
        //motorControlPwm->channel1 = {.p1 = ch1_u, .p2 = ch1_v, .p3 = ch1_w};
        //motorControlPwm->setChannel1Pins(motorControlPwm->channel1);
    } else {
        //motorControlPwm->channel2 = {.p1 = ch1_u, .p2 = ch1_v, .p3 = ch1_w};
        //motorControlPwm->setChannel2Pins(motorControlPwm->channel2);
    }
#else
    //motorControlPwm->channel1 = {.p1 = ch1_u, .p2 = ch1_v, .p3 = ch1_w};
    //motorControlPwm->setChannel1Pins(motorControlPwm->channel1);
#endif
}

void FOC::init() {
    printf("Setting PWM pins...\n");
#ifdef ENABLE_2_CHANNEL_PWM
    if(channel == Channel::CH1) {
        motorControlPwm->setChannel1Duty(50, 50, 50);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    } else {
        motorControlPwm->setChannel2Duty(50, 50, 50);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
#else
    motorControlPwm->setChannel1Duty(50, 50, 50);
    vTaskDelay(100 / portTICK_PERIOD_MS);
#endif
    printf("Aligning Electrical Angle...\n");
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

#ifdef ENABLE_2_CHANNEL_PWM
    if(channel == Channel::CH1) {
        motorControlPwm->setChannel1Duty(ch1 * 100.0 / Ts, ch2 * 100.0 / Ts, ch3 * 100.0 / Ts);
    } else {
        motorControlPwm->setChannel2Duty(ch1 * 100.0 / Ts, ch2 * 100.0 / Ts, ch3 * 100.0 / Ts);
    }
#else
    motorControlPwm->setChannel1Duty(ch1 * 100.0 / Ts, ch2 * 100.0 / Ts, ch3 * 100.0 / Ts);
#endif

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

void FOC::getVelocity() {
    velocity = tle5012B->getAngularVelocity() * direction;
}

void FOC::setSensor(TLE5012B *tle5012B) {
    this->tle5012B = tle5012B;
}

void FOC::setPWMController(MotorControlPWM *mcp) {
    motorControlPwm = mcp;
}

void FOC::setADCPins(uint8_t pinA, uint8_t pinB) {
#ifdef ENABLE_2_CHANNEL_PWM
    if(channel == Channel::CH1) {
        motorControlPwm->setMotorCH1ADCPins(pinA, pinB);
    } else {
        motorControlPwm->setMotorCH2ADCPins(pinA, pinB);
    }
#else
    motorControlPwm->setMotorCH1ADCPins(pinA, pinB);
#endif
}

void FOC::setTs(int ts) {
    Ts = ts;
}

void FOC::ClarkeParkTransform() {
#ifdef ENABLE_2_CHANNEL_PWM
    if(channel == Channel::CH1) {
        phaseVoltage = motorControlPwm->getADCVoltageCH1();
        phaseVoltage.U = ch1_id_lp.filter(phaseVoltage.U);
        phaseVoltage.V = ch1_iq_lp.filter(phaseVoltage.V);
    } else {
        phaseVoltage = motorControlPwm->getADCVoltageCH2();
        phaseVoltage.U = ch2_id_lp.filter(phaseVoltage.U);
        phaseVoltage.V = ch2_iq_lp.filter(phaseVoltage.V);
    }
#else
    phaseVoltage = motorControlPwm->getADCVoltageCH1();
    phaseVoltage.U = ch1_id_lp.filter(phaseVoltage.U);
    phaseVoltage.V = ch1_iq_lp.filter(phaseVoltage.V);
#endif

    // Clarke transform
    float i_alpha = CH1_VOLTAGE2CURRENT(phaseVoltage.U);
    float i_beta = _1_SQRT3 * CH1_VOLTAGE2CURRENT(phaseVoltage.U) + _2_SQRT3 * CH1_VOLTAGE2CURRENT(phaseVoltage.V);
    //printf("i_alpha:%.2f,i_beta:%.2f\n", i_alpha, i_beta);

    // Park transform
    id = i_alpha * cosVal + i_beta * sinVal;
    iq = i_beta * cosVal - i_alpha * sinVal;
}

void FOC::motorStart() {
    getVelocity();
    switch (mode) {
        case Torque: {
            // Get electrical angle
            getEAngle();
            //angle = LPF(angle, lpf_last_angle, lpf_ratio_angle);
            realAngle = (float) tle5012B->getAngleValue() * direction;
            angle = roundf(angle * 100) / 100;
            //printf("angle%d: %f\n", channel, angle);
            cosVal = fast_cos(angle);
            sinVal = fast_sin(angle);
            //lpf_last_angle = angle;

            ClarkeParkTransform();

            // Current loop PID;
            float vd = PIDController(idPID,-id);
            float vq = PIDController(iqPID, target - iq);

            svpwm(vq, vd);
            break;
        }
        case Velocity: {
            getEAngle();
            getVelocity();
            angle = roundf(angle * 100) / 100;
            cosVal = fast_cos(angle);
            sinVal = fast_sin(angle);
            ClarkeParkTransform();

            if(channel == Channel::CH1) {
                velocity = ch1_velocity_lp.filter(velocity);
            } else {
                velocity = ch2_velocity_lp.filter(velocity);
            }
            float target_iq = PIDController(velocityPID, target - velocity);
            float vd = PIDController(idPID, -id);
            float vq = PIDController(iqPID, target_iq - iq);
            svpwm(vq, vd);

            break;
        }
        case Position: {
            getEAngle();

            realAngle = (float) tle5012B->getAngleValue() * direction;
            angle = roundf(angle * 100) / 100;
            realAngle = roundf(realAngle * 100) / 100;

            cosVal = fast_cos(angle);
            sinVal = fast_sin(angle);

            ClarkeParkTransform();

            float target_iq;
#ifdef ENABLE_2_CHANNEL_PWM
            if(channel == Channel::CH1) {
                target_iq = PIDController(positionPID, (target/* * CH1_TARGET_DIR + CH1_REAL_ANGLE_OFFSET*/) - realAngle);
            } else {
                target_iq = PIDController(positionPID, (target/* * CH2_TARGET_DIR + CH2_REAL_ANGLE_OFFSET*/) - realAngle);
            }
#else
            target_iq = PIDController(positionPID, (target/* * CH1_TARGET_DIR + CH1_REAL_ANGLE_OFFSET*/) - realAngle);
#endif

            // Current loop PID;
            float vd = PIDController(idPID, -id);
            float vq = PIDController(iqPID, target_iq - iq);
            //printf("CH%dTargetIq:%f, realAngle:%f, vq:%f\n", channel, target_iq, realAngle, vq);
            //printf("vd:%f,vq:%f\n", vd, vq);

            svpwm(vq, vd);
            break;
        }
    }
}

void FOC::alignElectricalAngle() {
    /** This piece of code is finding zero electrical angle and sensor direction.
     *  If these two parameters are already known, we don't really need this code.
     */
    if(IF_ALIGN_E_ANGLE) {
        // Find sensor direction
        tle5012B->update();
        float temp_angle = tle5012B->getAngleValue();
        for (int i = 0; i <= 500; i++) {
            float a = _3PI_2 + _2PI * i / 500.0f;
            cosVal = fast_cos(a);
            sinVal = fast_sin(a);
            svpwm(Valign, 0);
            //spwm(Valign, 0, a);
            //delay_clock(240*1000);
            vTaskDelay(3 / portTICK_PERIOD_MS);
        }
        tle5012B->update();
        if (tle5012B->getAngleValue() > temp_angle) direction = SensorDirection::CW;
        else direction = SensorDirection::CCW;
        for (int i = 500; i >= 0; i--) {
            float a = _3PI_2 + _2PI * i / 500.0f;
            cosVal = fast_cos(a);
            sinVal = fast_sin(a);
            svpwm(Valign, 0);
            //spwm(Valign, 0, a);
            //delay_clock(240*1000);
            vTaskDelay(3 / portTICK_PERIOD_MS);
        }

        // Align mechanical angle and electrical angle
        cosVal = fast_cos(_3PI_2);
        sinVal = fast_sin(_3PI_2);
        svpwm(Valign, 0);
        //spwm(Valign, 0, _3PI_2);
        vTaskDelay(200 / portTICK_PERIOD_MS);
        tle5012B->update();
        zero_electrical_angle = 0;
        getEAngle();    // update angle value
        zero_electrical_angle = angle;
    } else {
#ifdef ENABLE_2_CHANNEL_PWM
        if(channel == Channel::CH1) {
            zero_electrical_angle = CH1_ZERO_ELECTRICAL_ANGLE;
            direction = CH1_SENSOR_DIRECTION;
        } else {
            zero_electrical_angle = CH2_ZERO_ELECTRICAL_ANGLE;
            direction = CH2_SENSOR_DIRECTION;
        }
        tle5012B->update();
#else
        zero_electrical_angle = CH1_ZERO_ELECTRICAL_ANGLE;
        tle5012B->update();
#endif
    }


#ifdef ENABLE_2_CHANNEL_PWM
    if(channel == Channel::CH1) {
        motorControlPwm->setChannel1Duty(50, 50, 50);
        //zero_electrical_angle = CH1_ZERO_ELECTRICAL_ANGLE;
        //direction = CH1_SENSOR_DIRECTION;
    } else {
        //float a = 0;
        //Valign = 2.6;
        //while(1) {
        //    cosVal = fast_cos(a);
        //    sinVal = fast_sin(a);
        //    svpwm(Valign, 0);
        //    vTaskDelay(5/portTICK_PERIOD_MS);
        //    a+= 0.01;
        //}

        motorControlPwm->setChannel2Duty(50, 50, 50);
        //0zero_electrical_angle = CH2_ZERO_ELECTRICAL_ANGLE;
        //0direction = CH2_SENSOR_DIRECTION;
    }
#else
    motorControlPwm->setChannel1Duty(50, 50, 50);
#endif
    /// Do NOT move the motor during calibration
    printf("Calibrating ADC...\n");
    motorControlPwm->adc_calibration();
    vTaskDelay(100 / portTICK_PERIOD_MS);
/*
#ifdef ENABLE_2_CHANNEL_PWM
    if(channel == Channel::CH1) {
        printf("Aligning current sensor...\n");
        uint8_t alignResult = motorControlPwm->ch1CurrentSenseAlign();
        printf("Align result: %d\n", alignResult);
    } else {
        printf("Aligning current sensor...\n");
        uint8_t alignResult = motorControlPwm->ch2CurrentSenseAlign();
        printf("Align result: %d\n", alignResult);
    }
#else
    printf("Aligning current sensor...\n");
    uint8_t alignResult = motorControlPwm->ch1CurrentSenseAlign();
    printf("Align result: %d\n", alignResult);
#endif
*/
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

#ifdef ENABLE_2_CHANNEL_PWM
    if(channel == Channel::CH1) {
        motorControlPwm->setChannel1Duty(Ua * 100.0 / Vdc, Ub * 100.0 / Vdc, Uc * 100.0 / Vdc);
    } else {
        motorControlPwm->setChannel2Duty(Ua * 100.0 / Vdc, Ub * 100.0 / Vdc, Uc * 100.0 / Vdc);
    }
#else
    motorControlPwm->setChannel1Duty(Ua * 100.0 / Vdc, Ub * 100.0 / Vdc, Uc * 100.0 / Vdc);
#endif

}

float FOC::getZeroEAngle() {
    return zero_electrical_angle;
}

