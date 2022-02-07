//
// Created by Justin on 03/02/2022.
//

#ifndef FOCX_CONF_H
#define FOCX_CONF_H

//#define ENABLE_2_CHANNEL_PWM

#define MISO 18
#define MOSI 5
#define CLK  23
#define CS   19

/// ADC Parameters
#define CH1_VOLTAGE_GAIN    106.4
#define CH2_VOLTAGE_GAIN    51.0
#define CH1_SAMPLING_RESISTANCE     0.001
#define CH2_SAMPLING_RESISTANCE     0.001
#define CH1_VOLTAGE2CURRENT(v) (v) * 1000.0f / CH1_VOLTAGE_GAIN   /// mV to mA
#define CH2_VOLTAGE2CURRENT(v) (v) * 1000.0f / CH2_VOLTAGE_GAIN

//#define ADC_VREF_CALIBRATION 1135

/// UART
#define UART_NUMBER UART_NUM_0  //usb transmission

/// Watch dog
#define TWDT_TIMEOUT_S 2

#define CHECK_ERROR_CODE(returned, expected) ({                        \
            if(returned != expected){                                  \
                printf("TWDT ERROR\n");                                \
                abort();                                               \
            }                                                          \
})

typedef struct {
    float U;
    float V;
    float W;
} PhaseVoltage;

typedef struct {
    float d;
    float q;
} DQCurrent;

typedef enum {
    CW = 1,
    CCW = -1
} SensorDirection;

typedef struct {
    float kp;
    float ki;
    float kd;
    float last_error;
    float error_sum;
    float error_sum_constrain;
    float output_constrain;
} PIDControlParameters;

typedef enum {
    Torque,
    Velocity,
    Position
} FOC_MODE;

#endif //FOCX_CONF_H
