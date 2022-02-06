//
// Created by Justin on 02/02/2022.
//

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "MotorControlPWM.h"
MotorControlPWM *pointerToThisClass = nullptr;
static void IRAM_ATTR adc_isr(void*);
static void IRAM_ATTR adc_isr(void*) {
    pointerToThisClass->adc_interrupt_handler();
}

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
    pointerToThisClass = this;  //Important!

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

    mcpwmDev[MCPWM_UNIT_0]->int_ena.timer0_tez_int_ena = true;//A PWM timer 0 TEZ event will trigger this interrupt
    mcpwmDev[MCPWM_UNIT_0]->int_ena.timer1_tez_int_ena = true;//A PWM timer 1 TEZ event will trigger this interrupt
    mcpwm_isr_register(MCPWM_UNIT_0, adc_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);

    mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, (mcpwm_sync_signal_t)1, 0);
    mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, (mcpwm_sync_signal_t)1, 0);
    mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, (mcpwm_sync_signal_t)1, 0);

    mcpwmDev[MCPWM_UNIT_0]->timer[MCPWM_TIMER_0].sync.out_sel = 1;

    // ADC Init
    adc_chars = static_cast<esp_adc_cal_characteristics_t *>(
            calloc(1, sizeof(esp_adc_cal_characteristics_t)));
    val_type = esp_adc_cal_characterize(ADC_UNIT_1,
                                        ADC_ATTEN_DB_11,
                                        ADC_WIDTH_BIT_12,
                                        ESP_ADC_CAL_VAL_EFUSE_VREF,
                                        adc_chars);

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CH1_A,ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC_CH1_B,ADC_ATTEN_DB_11);
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


void MotorControlPWM::setMotorCH1ADCPins(uint8_t pinA, uint8_t pinB) {
    if(pinA > 39 || pinA < 32 || pinB > 39 || pinB < 32) return;
    ADC_CH1_A = ADC_MAT[39 - pinA];
    ADC_CH1_B = ADC_MAT[39 - pinB];
    gpio_pad_select_gpio(pinA);
    gpio_set_direction(static_cast<gpio_num_t>(pinA), GPIO_MODE_INPUT);
    gpio_pad_select_gpio(pinB);
    gpio_set_direction(static_cast<gpio_num_t>(pinB), GPIO_MODE_INPUT);
}

void MotorControlPWM::setMotorCH2ADCPins(uint8_t pinA, uint8_t pinB) {
    if(pinA > 39 || pinA < 32 || pinB > 39 || pinB < 32) return;
    ADC_CH1_A = ADC_MAT[pinA - 39];
    ADC_CH1_B = ADC_MAT[pinB - 39];
    gpio_pad_select_gpio(pinA);
    gpio_set_direction(static_cast<gpio_num_t>(pinA), GPIO_MODE_INPUT);
    gpio_pad_select_gpio(pinB);
    gpio_set_direction(static_cast<gpio_num_t>(pinB), GPIO_MODE_INPUT);
}

void MotorControlPWM::adc_interrupt_handler() {
    uint32_t mcpwm_intr_status_0 = mcpwmDev[MCPWM_UNIT_0]->int_st.timer0_tez_int_st;
    uint32_t mcpwm_intr_status_1 = mcpwmDev[MCPWM_UNIT_0]->int_st.timer1_tez_int_st;
    switch (state) {
        case 1:
            if (mcpwm_intr_status_0 > 0) adc_raw1 = adc1_get_raw(ADC_CH1_A);
            state = 2;
            break;
        case 2:
            if (mcpwm_intr_status_1 > 0) adc_raw2 = adc1_get_raw(ADC_CH1_B);
            state = 1;
            break;
    }

    mcpwmDev[MCPWM_UNIT_0]->int_clr.timer0_tez_int_clr = mcpwm_intr_status_0;
    mcpwmDev[MCPWM_UNIT_0]->int_clr.timer1_tez_int_clr = mcpwm_intr_status_1;
}

PhaseVoltage MotorControlPWM::getADCVoltage() {
    PhaseVoltage voltage;
    //voltage.U = (-0.00006f * (float) adc_raw1 * (float) adc_raw1 + 1.2843f * (float) adc_raw1 + 11.616f) * ADC_CONVERSION;
    //voltage.V = (-0.00006f * (float) adc_raw2 * (float) adc_raw2 + 1.2843f * (float) adc_raw2 + 11.616f) * ADC_CONVERSION;
    voltage.U = ((float) esp_adc_cal_raw_to_voltage(adc_raw1, adc_chars) - adc_offset1)*(-1);   //Minus offset
    voltage.V = ((float) esp_adc_cal_raw_to_voltage(adc_raw2, adc_chars) - adc_offset2)*(-1);
    voltage.W = 0;
    return voltage;
}

void MotorControlPWM::adc_calibration() {
    uint16_t iteration = 1000;
    float adc1_sum = 0, adc2_sum = 0;
    for(int i = 0; i < iteration; i++) {
        adc1_sum += (float) esp_adc_cal_raw_to_voltage(adc_raw1, adc_chars);
        adc2_sum += (float) esp_adc_cal_raw_to_voltage(adc_raw2, adc_chars);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    adc_offset1 = adc1_sum / (float) iteration;
    adc_offset2 = adc2_sum / (float) iteration;
}
