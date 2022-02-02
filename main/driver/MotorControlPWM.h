//
// Created by Justin on 02/02/2022.
//

#ifndef FOCX_MOTORCONTROLPWM_H
#define FOCX_MOTORCONTROLPWM_H

#include "driver/mcpwm.h"

#define PWM_FREQUENCY 160000000


class MCPWM {
public:
    typedef struct {
        uint8_t p1;
        uint8_t p2;
        uint8_t p3;
    } three_phase_pwm_pins_t;

    MCPWM() = default;

    void setPWMpins(three_phase_pwm_pins_t channel1, three_phase_pwm_pins_t channel2);

    void init();

private:
    mcpwm_io_signals_t mcpwmIoSignals;
    mcpwm_pin_config_t mcpwmPinConfig;
    mcpwm_timer_t mcpwmTimer;
    mcpwm_config_t mcpwmConfig;

};


#endif //FOCX_MOTORCONTROLPWM_H
