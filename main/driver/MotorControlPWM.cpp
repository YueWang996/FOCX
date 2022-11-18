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

void MotorControlPWM::setChannel1Pins(three_phase_pwm_pins_t channel1) {
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, gpio_num_t(channel1.p1));
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, gpio_num_t(channel1.p2));
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, gpio_num_t(channel1.p3));
    ch1 = USED;
}

void MotorControlPWM::setChannel2Pins(three_phase_pwm_pins_t channel2) {
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
            .cmpr_a = 0,
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
    delay_clock(240000);
    mcpwmDev[MCPWM_UNIT_0]->timer[0].period.prescale = prescaler;
    mcpwmDev[MCPWM_UNIT_0]->timer[1].period.prescale = prescaler;
    mcpwmDev[MCPWM_UNIT_0]->timer[2].period.prescale = prescaler;
    delay_clock(240000);
    mcpwmDev[MCPWM_UNIT_0]->timer[0].period.period = resolution_corrected;
    mcpwmDev[MCPWM_UNIT_0]->timer[1].period.period = resolution_corrected;
    mcpwmDev[MCPWM_UNIT_0]->timer[2].period.period = resolution_corrected;
    delay_clock(240000);
    mcpwmDev[MCPWM_UNIT_0]->timer[0].period.upmethod = 0;
    mcpwmDev[MCPWM_UNIT_0]->timer[1].period.upmethod = 0;
    mcpwmDev[MCPWM_UNIT_0]->timer[2].period.upmethod = 0;
    delay_clock(240000);

    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_1);
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_2);
    delay_clock(240000);


    mcpwmDev[MCPWM_UNIT_0]->int_ena.timer1_tez_int_ena = true;//A PWM timer 1 TEZ event will trigger this interrupt
    //mcpwmDev[MCPWM_UNIT_0]->int_ena.timer1_tep_int_ena = true;//A PWM timer 1 TEP event will trigger this interrupt
    mcpwm_isr_register(MCPWM_UNIT_0, adc_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);

    delay_clock(240000);
    mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, (mcpwm_sync_signal_t)1, 0);
    mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, (mcpwm_sync_signal_t)1, 0);
    mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, (mcpwm_sync_signal_t)1, 0);

    delay_clock(240000);
    mcpwmDev[MCPWM_UNIT_0]->timer[MCPWM_TIMER_0].sync.out_sel = 1;
    delay_clock(240000);
    mcpwmDev[MCPWM_UNIT_0]->timer[MCPWM_TIMER_0].sync.out_sel = 0;

    //mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &mcpwmConfig);
    //mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &mcpwmConfig);
    //mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_2, &mcpwmConfig);
    //delay_clock(240000);
    //mcpwm_stop(MCPWM_UNIT_1, MCPWM_TIMER_0);
    //mcpwm_stop(MCPWM_UNIT_1, MCPWM_TIMER_1);
    //mcpwm_stop(MCPWM_UNIT_1, MCPWM_TIMER_2);
    //delay_clock(240000);
    //mcpwmDev[MCPWM_UNIT_1]->clk_cfg.prescale = 0;
    //delay_clock(240000);
    //mcpwmDev[MCPWM_UNIT_1]->timer[0].period.prescale = prescaler;
    //mcpwmDev[MCPWM_UNIT_1]->timer[1].period.prescale = prescaler;
    //mcpwmDev[MCPWM_UNIT_1]->timer[2].period.prescale = prescaler;
    //delay_clock(240000);
    //mcpwmDev[MCPWM_UNIT_1]->timer[0].period.period = resolution_corrected;
    //mcpwmDev[MCPWM_UNIT_1]->timer[1].period.period = resolution_corrected;
    //mcpwmDev[MCPWM_UNIT_1]->timer[2].period.period = resolution_corrected;
    //delay_clock(240000);
    //mcpwmDev[MCPWM_UNIT_1]->timer[0].period.upmethod = 0;
    //mcpwmDev[MCPWM_UNIT_1]->timer[1].period.upmethod = 0;
    //mcpwmDev[MCPWM_UNIT_1]->timer[2].period.upmethod = 0;
    //delay_clock(240000);
    //mcpwm_start(MCPWM_UNIT_1, MCPWM_TIMER_0);
    //mcpwm_start(MCPWM_UNIT_1, MCPWM_TIMER_1);
    //mcpwm_start(MCPWM_UNIT_1, MCPWM_TIMER_2);
    ////delay_clock(240000);
    ////mcpwmDev[MCPWM_UNIT_1]->int_ena.timer0_tez_int_ena = true;//A PWM timer 0 TEZ event will trigger this interrupt
    ////mcpwmDev[MCPWM_UNIT_1]->int_ena.timer1_tez_int_ena = true;//A PWM timer 1 TEZ event will trigger this interrupt
    ////mcpwm_isr_register(MCPWM_UNIT_1, adc_isr2, NULL, ESP_INTR_FLAG_IRAM, NULL);
    //delay_clock(240000);
    //mcpwm_sync_enable(MCPWM_UNIT_1, MCPWM_TIMER_0, (mcpwm_sync_signal_t)1, 0);
    //mcpwm_sync_enable(MCPWM_UNIT_1, MCPWM_TIMER_1, (mcpwm_sync_signal_t)1, 0);
    //mcpwm_sync_enable(MCPWM_UNIT_1, MCPWM_TIMER_2, (mcpwm_sync_signal_t)1, 0);
    //delay_clock(240000);
//
    //mcpwmDev[MCPWM_UNIT_1]->timer[MCPWM_TIMER_0].sync.out_sel = 1;
    //delay_clock(240000);
    //mcpwmDev[MCPWM_UNIT_1]->timer[MCPWM_TIMER_0].sync.out_sel = 0;

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
    adc1_config_channel_atten(ADC_CH2_A,ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC_CH2_B,ADC_ATTEN_DB_11);
    adc1_config_channel_atten(BAT_PIN,ADC_ATTEN_DB_11);
    //adc_power_acquire();

    temperature_adc_chars = static_cast<esp_adc_cal_characteristics_t *>(
            calloc(1, sizeof(esp_adc_cal_characteristics_t)));
    temperature_adc_type = esp_adc_cal_characterize(ADC_UNIT_2,
                                                    ADC_ATTEN_DB_11,
                                                    ADC_WIDTH_BIT_12,
                                                    ESP_ADC_CAL_VAL_EFUSE_VREF,
                                                    temperature_adc_chars);
    adc2_config_channel_atten(ADC_CH1_TEMP,ADC_ATTEN_DB_11);
    adc2_config_channel_atten(ADC_CH2_TEMP,ADC_ATTEN_DB_11);

    gpio_pad_select_gpio(interrupt_output_pin);
    gpio_set_direction(static_cast<gpio_num_t>(interrupt_output_pin), GPIO_MODE_OUTPUT);
}

void MotorControlPWM::setChannel1Duty(float a, float b, float c) {
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, a);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, b);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, c);
}

void MotorControlPWM::setChannel2Duty(float a, float b, float c) {
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, a);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, b);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, c);
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
    ADC_CH2_A = ADC_MAT[pinA - 39];
    ADC_CH2_B = ADC_MAT[pinB - 39];
    gpio_pad_select_gpio(pinA);
    gpio_set_direction(static_cast<gpio_num_t>(pinA), GPIO_MODE_INPUT);
    gpio_pad_select_gpio(pinB);
    gpio_set_direction(static_cast<gpio_num_t>(pinB), GPIO_MODE_INPUT);
}

void MotorControlPWM::adc_interrupt_handler() {
    //uint32_t mcpwm_intr_status_0 = mcpwmDev[MCPWM_UNIT_0]->int_st.val;
    uint32_t mcpwm_intr_status_0 = mcpwmDev[MCPWM_UNIT_0]->int_st.timer1_tez_int_st;

    switch (state) {
        case 1:
            if (mcpwm_intr_status_0 > 0) {
                adc_raw1a = adc1_get_raw(ADC_CH1_A);
                gpio_set_level(static_cast<gpio_num_t>(interrupt_output_pin), (uint32_t)level);
                level = !level;
                adc_raw2a = adc1_get_raw(ADC_CH2_A);
                state = 2;
            }
            break;
        case 2:
            if (mcpwm_intr_status_0 > 0) {
                adc_raw1b = adc1_get_raw(ADC_CH1_B);
                //adc_raw2b = adc1_get_raw(ADC_CH2_B);
                state = 3;
            }
            break;
        case 3:
            if (mcpwm_intr_status_0 > 0) {
                adc_raw2a = adc1_get_raw(ADC_CH2_A);
                state = 4;
            }
            break;
        case 4:
            if (mcpwm_intr_status_0 > 0) {
                adc_raw2b = adc1_get_raw(ADC_CH2_B);
                state = 1;
            }
            break;
    }

    //mcpwmDev[MCPWM_UNIT_0]->int_clr.val = mcpwm_intr_status_0;
    mcpwmDev[MCPWM_UNIT_0]->int_clr.timer1_tez_int_clr = mcpwm_intr_status_0;
}


PhaseVoltage MotorControlPWM::getADCVoltageCH1() {
    PhaseVoltage voltage;
    voltage.U = ((float) esp_adc_cal_raw_to_voltage(adc_raw1a, adc_chars) - adc_offset1a) * ch1_gain_u;    // *-1 to invert voltage
    voltage.V = ((float) esp_adc_cal_raw_to_voltage(adc_raw1b, adc_chars) - adc_offset1b) * ch1_gain_v;// * (-1);
    voltage.W = 0;
    return voltage;
}

PhaseVoltage MotorControlPWM::getADCVoltageCH2() {
    PhaseVoltage voltage;
    voltage.U = ((float) esp_adc_cal_raw_to_voltage(adc_raw2a, adc_chars) - adc_offset2a) * ch2_gain_u;    // *-1 to invert voltage
    voltage.V = ((float) esp_adc_cal_raw_to_voltage(adc_raw2b, adc_chars) - adc_offset2b) * ch2_gain_v;
    voltage.W = 0;
    return voltage;
}

void MotorControlPWM::adc_calibration() {
    uint16_t iteration = 1500;
    float adc1a_sum = 0, adc1b_sum = 0;
#ifdef ENABLE_2_CHANNEL_PWM
    float adc2a_sum = 0, adc2b_sum = 0;
#endif
    for(int i = 0; i < iteration; i++) {
        adc1a_sum += (float) esp_adc_cal_raw_to_voltage(adc_raw1a, adc_chars);
        adc1b_sum += (float) esp_adc_cal_raw_to_voltage(adc_raw1b, adc_chars);
#ifdef ENABLE_2_CHANNEL_PWM
        adc2a_sum += (float) esp_adc_cal_raw_to_voltage(adc_raw2a, adc_chars);
        adc2b_sum += (float) esp_adc_cal_raw_to_voltage(adc_raw2b, adc_chars);
#endif
        delay_clock(240000*1);
    }
    adc_offset1a = adc1a_sum / (float) iteration;
    adc_offset1b = adc1b_sum / (float) iteration;
#ifdef ENABLE_2_CHANNEL_PWM
    adc_offset2a = adc2a_sum / (float) iteration;
    adc_offset2b = adc2b_sum / (float) iteration;
#endif
}

float MotorControlPWM::getChannel1Temperature() {
    int read_raw;
    float v_adc;
    adc2_get_raw(ADC_CH1_TEMP, ADC_WIDTH_BIT_12, &read_raw);
    v_adc = (float)esp_adc_cal_raw_to_voltage(read_raw, temperature_adc_chars) / 1000.0f;
    float R_2 = (VCC - v_adc) * R_1 / v_adc;
    return 1.0f / (logf(R_2 / R_0) / B_CONSTANT + _1_T_0);
}

float MotorControlPWM::getChannel2Temperature() {
    int read_raw;
    float v_adc;
    adc2_get_raw(ADC_CH2_TEMP, ADC_WIDTH_BIT_12, &read_raw);
    v_adc = (float)esp_adc_cal_raw_to_voltage(read_raw, temperature_adc_chars) / 1000.0f;
    float R_2 = (VCC - v_adc) * R_1 / v_adc;
    return 1.0f / (logf(R_2 / R_0) / B_CONSTANT + _1_T_0);
}

float MotorControlPWM::getBatteryVoltage() {
    int bat_raw = adc1_get_raw(BAT_PIN);
    return (((float)esp_adc_cal_raw_to_voltage(bat_raw, adc_chars) / 1000.0f) * (BAT_R_1 + BAT_R_2)) / BAT_R_2;
}

uint8_t MotorControlPWM::ch1CurrentSenseAlign() {
    uint8_t flag = 1;

    setChannel1Duty(5, 0, 0);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    PhaseVoltage phaseVoltage = getADCVoltageCH1();
    float c_u = CH1_VOLTAGE2CURRENT(phaseVoltage.U);
    float c_v = CH1_VOLTAGE2CURRENT(phaseVoltage.V);
    float c_u_1, c_v_1;
    for (int i = 0; i < 100; i++) {
        phaseVoltage = getADCVoltageCH1();
        c_u_1 = CH1_VOLTAGE2CURRENT(phaseVoltage.U);
        c_v_1 = CH1_VOLTAGE2CURRENT(phaseVoltage.V);
        c_u = c_u*0.6f + 0.4f*c_u_1;
        c_v = c_v*0.6f + 0.4f*c_v_1;
        vTaskDelay(3/portTICK_PERIOD_MS);
    }
    setChannel1Duty(0, 0, 0);

    float uv_ratio = fabs(c_u / c_v);

    if( uv_ratio > 1.5f ){ // should be ~2
        ch1_gain_u = _sign(c_u);
    }else if( uv_ratio < 0.7f ){ // should be ~0.5
        // switch phase A and B
        adc1_channel_t tmp_pinA = ADC_CH1_A;
        ADC_CH1_A = ADC_CH1_B;
        ADC_CH1_B = tmp_pinA;
        ch1_gain_u = _sign(c_v);

        flag = 2;
    } else {
        return 0;
    }


    setChannel1Duty(0, 5, 0);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    phaseVoltage = getADCVoltageCH1();
    c_u = CH1_VOLTAGE2CURRENT(phaseVoltage.U);
    c_v = CH1_VOLTAGE2CURRENT(phaseVoltage.V);
    for (int i = 0; i < 100; i++) {
        phaseVoltage = getADCVoltageCH1();
        c_u_1 = CH1_VOLTAGE2CURRENT(phaseVoltage.U);
        c_v_1 = CH1_VOLTAGE2CURRENT(phaseVoltage.V);
        c_u = c_u*0.6f + 0.4f*c_u_1;
        c_v = c_v*0.6f + 0.4f*c_v_1;
        vTaskDelay(3/portTICK_PERIOD_MS);
    }
    setChannel1Duty(0, 0, 0);

    float vu_ratio = fabs(c_v / c_u);

    if( vu_ratio > 1.5f ){ // should be ~2
        ch1_gain_v = _sign(c_v);
    }else if( vu_ratio < 0.7f ){ // should be ~0.5
        // switch phase A and B
        adc1_channel_t tmp_pinB = ADC_CH1_B;
        ADC_CH1_B = ADC_CH1_A;
        ADC_CH1_A = tmp_pinB;
        ch1_gain_v = _sign(c_u);

        flag = 2;
    } else {
        return 0;
    }

    if(ch1_gain_u < 0 || ch1_gain_v < 0) flag +=2;

    return flag;
}

uint8_t MotorControlPWM::ch2CurrentSenseAlign() {
    uint8_t flag = 1;

    setChannel2Duty(10, 0, 0);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    PhaseVoltage phaseVoltage = getADCVoltageCH2();
    float c_u = CH1_VOLTAGE2CURRENT(phaseVoltage.U);
    float c_v = CH1_VOLTAGE2CURRENT(phaseVoltage.V);
    float c_u_1, c_v_1;
    for (int i = 0; i < 100; i++) {
        phaseVoltage = getADCVoltageCH2();
        c_u_1 = CH1_VOLTAGE2CURRENT(phaseVoltage.U);
        c_v_1 = CH1_VOLTAGE2CURRENT(phaseVoltage.V);
        c_u = c_u*0.6f + 0.4f*c_u_1;
        c_v = c_v*0.6f + 0.4f*c_v_1;
        vTaskDelay(3/portTICK_PERIOD_MS);
    }
    setChannel2Duty(0, 0, 0);

    float uv_ratio = fabs(c_u / c_v);

    if( uv_ratio > 1.5f ){ // should be ~2
        ch2_gain_u = _sign(c_u);
    }else if( uv_ratio < 0.7f ){ // should be ~0.5
        // switch phase A and B
        adc1_channel_t tmp_pinA = ADC_CH2_A;
        ADC_CH2_A = ADC_CH2_B;
        ADC_CH2_B = tmp_pinA;
        ch2_gain_u = _sign(c_v);

        flag = 2;
    } else {
        return 0;
    }


    setChannel2Duty(0, 10, 0);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    phaseVoltage = getADCVoltageCH2();
    c_u = CH1_VOLTAGE2CURRENT(phaseVoltage.U);
    c_v = CH1_VOLTAGE2CURRENT(phaseVoltage.V);
    for (int i = 0; i < 100; i++) {
        phaseVoltage = getADCVoltageCH2();
        c_u_1 = CH1_VOLTAGE2CURRENT(phaseVoltage.U);
        c_v_1 = CH1_VOLTAGE2CURRENT(phaseVoltage.V);
        c_u = c_u*0.6f + 0.4f*c_u_1;
        c_v = c_v*0.6f + 0.4f*c_v_1;
        vTaskDelay(3/portTICK_PERIOD_MS);
    }
    setChannel2Duty(0, 0, 0);

    float vu_ratio = fabs(c_v / c_u);

    if( vu_ratio > 1.5f ){ // should be ~2
        ch2_gain_v = _sign(c_v);
    }else if( vu_ratio < 0.7f ){ // should be ~0.5
        // switch phase A and B
        adc1_channel_t tmp_pinB = ADC_CH1_B;
        ADC_CH2_B = ADC_CH2_A;
        ADC_CH2_A = tmp_pinB;
        ch2_gain_v = _sign(c_u);

        flag = 2;
    } else {
        return 0;
    }

    if(ch2_gain_u < 0 || ch2_gain_v < 0) flag +=2;

    return flag;
}
