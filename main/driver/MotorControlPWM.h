//
// Created by Justin on 02/02/2022.
//

#ifndef FOCX_MOTORCONTROLPWM_H
#define FOCX_MOTORCONTROLPWM_H

#include <soc/mcpwm_struct.h>
#include "driver/mcpwm.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include <cmath>
#include <driver/adc_common.h>
#include <esp_adc_cal.h>
#include "../conf.h"


#define PWM_FREQUENCY           15000
#define PWM_RESOLUTION_DEFAULT  2048
#define PWM_MIN_RESOLUTION      1500
#define PWM_MAX_RESOLUTION      3000
#define MCPWM_BUS_FREQ          160000000

#define _sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )
#define fast_constrain(x, low, high)    ((x)<(low)?(low):((x) >(high)?(high):(x)))

class MotorControlPWM {
public:
    typedef enum {
        UNUSED = 0,
        USED
    } used_channel_t;

    MotorControlPWM() = default;

    void setChannel1Pins(three_phase_pwm_pins_t channel1);
    void setChannel2Pins(three_phase_pwm_pins_t channel2);
    void setMotorCH1ADCPins(uint8_t pinA, uint8_t pinB);
    void setMotorCH2ADCPins(uint8_t pinA, uint8_t pinB);

    void init(int *pwm_period);

    void setChannel1Duty(float a, float b, float c);
    void setChannel2Duty(float a, float b, float c);

    void adc_interrupt_handler();
    void adc_calibration();
    PhaseVoltage getADCVoltageCH1();
    PhaseVoltage getADCVoltageCH2();

    float getChannel1Temperature();
    float getChannel2Temperature();
    float getBatteryVoltage();

    uint8_t ch1CurrentSenseAlign();
    uint8_t ch2CurrentSenseAlign();

private:
    mcpwm_config_t mcpwmConfig;
    used_channel_t ch1 = UNUSED, ch2 = UNUSED;
    mcpwm_dev_t *mcpwmDev[2] = {&MCPWM0, &MCPWM1};


    adc1_channel_t ADC_CH1_A, ADC_CH1_B;
    adc1_channel_t ADC_CH2_A, ADC_CH2_B;
    adc1_channel_t BAT_PIN = ADC1_CHANNEL_6;
    adc1_channel_t ADC_MAT[8] = {ADC1_CHANNEL_3, ADC1_CHANNEL_2, ADC1_CHANNEL_1, ADC1_CHANNEL_0, ADC1_CHANNEL_7, ADC1_CHANNEL_6, ADC1_CHANNEL_5, ADC1_CHANNEL_4};

    esp_adc_cal_characteristics_t *adc_chars;
    esp_adc_cal_characteristics_t *temperature_adc_chars;
    esp_adc_cal_value_t val_type;
    esp_adc_cal_value_t temperature_adc_type;

    adc2_channel_t ADC_CH1_TEMP = CH1_TEMP_ADC_PIN;
    adc2_channel_t ADC_CH2_TEMP = CH2_TEMP_ADC_PIN;

    uint8_t state = 1;
    uint8_t state1 = 1;
    uint8_t adc_delay = 3;
    int adc_raw1a, adc_raw1b, adc_raw2a, adc_raw2b;
    float adc_offset1a, adc_offset1b, adc_offset2a, adc_offset2b;
    int8_t ch1_gain_u = -1, ch1_gain_v = -1;
    int8_t ch2_gain_u = -1, ch2_gain_v = -1;

    uint8_t interrupt_output_pin = 9;   // GPIO 9
    bool level = false;
};


#endif //FOCX_MOTORCONTROLPWM_H
