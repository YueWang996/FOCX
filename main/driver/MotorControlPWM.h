//
// Created by Justin on 02/02/2022.
//

#ifndef FOCX_MOTORCONTROLPWM_H
#define FOCX_MOTORCONTROLPWM_H

#include <soc/mcpwm_struct.h>
#include "driver/mcpwm.h"
#include <cmath>

#define PWM_FREQUENCY           25000
#define PWM_RESOLUTION_DEFAULT  2048
#define PWM_MIN_RESOLUTION      1500
#define PWM_MAX_RESOLUTION      3000
#define MCPWM_BUS_FREQ          160000000

#define fast_constrain(x, low, high)    ((x)<(low)?(low):((x) >(high)?(high):(x)))


class MotorControlPWM {
public:
    typedef struct {
        uint8_t p1;
        uint8_t p2;
        uint8_t p3;
    } three_phase_pwm_pins_t;

    typedef enum {
        UNUSED = 0,
        USED
    } used_channel_t;

    MotorControlPWM() = default;

    void setChannel1Pins(three_phase_pwm_pins_t channel1);
    void setChannel2Pins(three_phase_pwm_pins_t channel2);

    void init(int *pwm_period);

    void setChannel1Duty(float a, float b, float c);
    void setChannel2Duty(float a, float b, float c);

    three_phase_pwm_pins_t channel1, channel2;

private:
    mcpwm_config_t mcpwmConfig;

    used_channel_t ch1 = UNUSED, ch2 = UNUSED;

    mcpwm_dev_t *mcpwmDev[2] = {&MCPWM0, &MCPWM1};
};


#endif //FOCX_MOTORCONTROLPWM_H
