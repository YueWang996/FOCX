//
// Created by Justin on 03/02/2022.
//

#ifndef FOCX_CONF_H
#define FOCX_CONF_H
#include <cstdint>
/// Uncomment this when there is only one motor
//#define ENABLE_2_CHANNEL_PWM
#define IF_ALIGN_E_ANGLE true

/// SPI pins
#define MISO 18
#define MOSI 5
#define CLK  23
#define CS0  19
#define CS1  22

/// ADC Parameters and current conversion
#define CH1_VOLTAGE_GAIN    107.4
#define CH2_VOLTAGE_GAIN    50
#define CH1_SAMPLING_RESISTANCE     1   // mOhm
#define CH2_SAMPLING_RESISTANCE     1   // mOhm
#define CH1_VOLTAGE2CURRENT(v) ((v) / CH1_VOLTAGE_GAIN / CH1_SAMPLING_RESISTANCE)   // mV to A
#define CH2_VOLTAGE2CURRENT(v) ((v) / CH2_VOLTAGE_GAIN / CH2_SAMPLING_RESISTANCE)
#define BATTERY_ADC_PIN     ADC1_CHANNEL_6  // GPIO34
#define BAT_R_1     10000.0f
#define BAT_R_2     1000.0f

/// Thermal detection
#define CH1_TEMP_ADC_PIN    ADC2_CHANNEL_4  // GPIO13
#define CH2_TEMP_ADC_PIN    ADC2_CHANNEL_5  // GPIO12
#define R_0     10000.0f
#define R_1     1000.0f
#define B_CONSTANT  3434    // 3428 (25C-80C), 3434 (25C-85C), 3455 (25C-100C)
#define VCC     3.3f
#define _1_T_0  0.04f       // 1/25

/// UART
#define UART_NUMBER UART_NUM_0  //usb transmission

/// CAN
#define CAN_SPEED CAN_SPEED_500KBPS
#define CAN_RX_QUEUE_SIZE 10

#define MSG_ID 0x001
//#define MSG_ID 0x002
//#define MSG_ID 0x004
typedef enum {
    NoError = 0,
    UnMatchedID,
    UnMatchedCommand
} CAN_ERROR;

/// Angle and direction
#if MSG_ID == 0x001
    #define CH1_ZERO_ELECTRICAL_ANGLE   3.219249
    #define CH2_ZERO_ELECTRICAL_ANGLE   3.050321
    #define CH1_SENSOR_DIRECTION    SensorDirection::CW
    #define CH2_SENSOR_DIRECTION    SensorDirection::CCW
#endif

/// Watch dog
#define TWDT_TIMEOUT_S 10
#define CHECK_ERROR_CODE(returned, expected) ({                        \
            if(returned != expected){                                  \
                printf("TWDT ERROR\n");                                \
                abort();                                               \
            }                                                          \
})

/// Phase voltage type
typedef struct {
    float U;
    float V;
    float W;
} PhaseVoltage;

/// d-axis and q-axis current type
typedef struct {
    float d;
    float q;
} DQCurrent;

/// sensor direction
typedef enum {
    CW = 1,
    CCW = -1
} SensorDirection;

/// channel setting
typedef enum {
    CH1 = 1,
    CH2 = 2
} Channel;

/// PID configuration
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

typedef struct {
    uint8_t p1;
    uint8_t p2;
    uint8_t p3;
} three_phase_pwm_pins_t;

/// Define clock to delay (blocking delay). On ESP32-PICO-D4 (240MHz version), 1 ms = 240,000 ticks.
/// So to delay 1 ms, call delay_clock(240000);
static __inline void delay_clock(int ts)
{
    uint32_t start, curr;

    __asm__ __volatile__("rsr %0, ccount" : "=r"(start));
    do
            __asm__ __volatile__("rsr %0, ccount" : "=r"(curr));
    while (curr - start <= ts);
}


#define S_PULL_PARAMETERS 0x01

#define S_SET_SAMPLING_FREQ 0x02

#define S_SET_Q_PHASE_KP 0x04
#define S_SET_Q_PHASE_KI 0x05
#define S_SET_Q_PHASE_KD 0x06
#define S_SET_Q_PHASE_MAX 0x07
#define S_SET_Q_PHASE_MIN 0x08
#define S_SET_Q_PHASE_MAX_ERR 0x09

#define S_SET_D_PHASE_KP 0x0A
#define S_SET_D_PHASE_KI 0x0B
#define S_SET_D_PHASE_KD 0x0C
#define S_SET_D_PHASE_MAX 0x0D
#define S_SET_D_PHASE_MIN 0x0E
#define S_SET_D_PHASE_MAX_ERR 0x10

#define S_SET_FOC_MODE 0x03
#define S_SET_TORQUE_MODE 0x01
#define S_SET_VELOCITY_MODE 0x02
#define S_SET_POSITION_MODE 0x03

//-------------------------------//
#define S_PLOT_CURRENT 0x11
#define S_SET_TARGET 0x12
#define S_PLOT_ANGLE_VELOCITY 0x13

//------------------------------//
#define S_DATA_Q_PHASE_KP 0x20
#define S_DATA_Q_PHASE_KI 0x21
#define S_DATA_Q_PHASE_KD 0x22
#define S_DATA_Q_PHASE_MAX 0x23
#define S_DATA_Q_PHASE_MIN 0x24
#define S_DATA_Q_PHASE_MAX_ERR 0x25
#define S_DATA_D_PHASE_KP 0x26
#define S_DATA_D_PHASE_KI 0x27
#define S_DATA_D_PHASE_KD 0x28
#define S_DATA_D_PHASE_MAX 0x29
#define S_DATA_D_PHASE_MIN 0x2A
#define S_DATA_D_PHASE_MAX_ERR 0x2B

#endif //FOCX_CONF_H
