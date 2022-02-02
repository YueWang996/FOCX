//
// Created by Justin on 02/02/2022.
//

#include "MotorControlPWM.h"

void MCPWM::setPWMpins(MCPWM::three_phase_pwm_pins_t channel1, MCPWM::three_phase_pwm_pins_t channel2) {
    mcpwmPinConfig.mcpwm0a_out_num = channel1.p1;
    mcpwmPinConfig.mcpwm0b_out_num = channel1.p2;
    mcpwmPinConfig.mcpwm1a_out_num = channel1.p3;
    mcpwmPinConfig.mcpwm1b_out_num = channel2.p1;
    mcpwmPinConfig.mcpwm2a_out_num = channel2.p2;
    mcpwmPinConfig.mcpwm2b_out_num = channel2.p3;
}

void MCPWM::init() {
    mcpwm_set_pin(MCPWM_UNIT_0, &mcpwmPinConfig);
    mcpwmConfig = {
            .frequency = PWM_FREQUENCY,
            .cmpr_a = 0,
            .cmpr_b = 0,
            .duty_mode = MCPWM_DUTY_MODE_0,
            .counter_mode = MCPWM_UP_DOWN_COUNTER
            };

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &mcpwmConfig);
    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);

    mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_SELECT_SYNC0, 0);

}
