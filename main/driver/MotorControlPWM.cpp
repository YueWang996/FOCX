//
// Created by Justin on 02/02/2022.
//

#include "MotorControlPWM.h"

void MotorControlPWM::setChannel1Pins(MotorControlPWM::three_phase_pwm_pins_t channel1) {
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, gpio_num_t(channel1.p1));
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, gpio_num_t(channel1.p2));
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, gpio_num_t(channel1.p3));
    ch1 = USED;
}

void MotorControlPWM::setChannel2Pins(MotorControlPWM::three_phase_pwm_pins_t channel2) {
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, gpio_num_t(channel2.p1));
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, gpio_num_t(channel2.p2));
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, gpio_num_t(channel2.p3));
    ch2 = USED;
}

void MotorControlPWM::init(int *pwm_period) {
    int16_t prescaler = ceil((double) MCPWM_BUS_FREQ / (double) PWM_RESOLUTION_DEFAULT / 2.0 / (double) PWM_FREQUENCY) - 1;
    prescaler = fast_constrain(prescaler, 0, 128);
    int resolution_corrected = (double) MCPWM_BUS_FREQ / 2.0f / (double) PWM_FREQUENCY / (double)(prescaler + 1);
    if(resolution_corrected < PWM_MIN_RESOLUTION && prescaler > 0 )
        resolution_corrected = (double) MCPWM_BUS_FREQ / 2.0f / (double) PWM_FREQUENCY / (double)(--prescaler + 1);
    resolution_corrected = fast_constrain(resolution_corrected, PWM_MIN_RESOLUTION, PWM_MAX_RESOLUTION);
    *pwm_period = resolution_corrected;

    mcpwmConfig = {
            .frequency = 2 * PWM_FREQUENCY,
            .cmpr_a = 95.0,
            .cmpr_b = mcpwmConfig.cmpr_a,
            .duty_mode = MCPWM_DUTY_MODE_0,
            .counter_mode = MCPWM_UP_DOWN_COUNTER
            };

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &mcpwmConfig);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &mcpwmConfig);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &mcpwmConfig);

    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_1);
    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_2);

    mcpwmDev[MCPWM_UNIT_0]->clk_cfg.prescale = 0;
    mcpwmDev[MCPWM_UNIT_0]->timer[0].period.prescale = prescaler;
    mcpwmDev[MCPWM_UNIT_0]->timer[1].period.prescale = prescaler;
    mcpwmDev[MCPWM_UNIT_0]->timer[2].period.prescale = prescaler;
    mcpwmDev[MCPWM_UNIT_0]->timer[0].period.period = resolution_corrected;
    mcpwmDev[MCPWM_UNIT_0]->timer[1].period.period = resolution_corrected;
    mcpwmDev[MCPWM_UNIT_0]->timer[2].period.period = resolution_corrected;
    mcpwmDev[MCPWM_UNIT_0]->timer[0].period.upmethod = 0;
    mcpwmDev[MCPWM_UNIT_0]->timer[1].period.upmethod = 0;
    mcpwmDev[MCPWM_UNIT_0]->timer[2].period.upmethod = 0;

    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_1);
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_2);

    mcpwmDev[MCPWM_UNIT_0]->int_ena.timer0_tep_int_ena = true;//A PWM timer 0 TEP event will trigger this interrupt
    mcpwmDev[MCPWM_UNIT_0]->int_ena.timer1_tep_int_ena = true;//A PWM timer 1 TEP event will trigger this interrupt

    //TODO: do ADC in ISR call back

    mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, (mcpwm_sync_signal_t)1, 0);
    mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, (mcpwm_sync_signal_t)1, 0);
    mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, (mcpwm_sync_signal_t)1, 0);

    mcpwmDev[MCPWM_UNIT_0]->timer[MCPWM_TIMER_0].sync.out_sel = 1;
}

void MotorControlPWM::setChannel1Duty(float a, float b, float c) {
    if(ch1) {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, a);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, b);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_A, c);
    }
}

void MotorControlPWM::setChannel2Duty(float a, float b, float c) {
    if(ch2) {
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_GEN_B, a);
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_B, b);
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_GEN_B, c);
    }
}
