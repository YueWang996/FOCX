//
// Created by Justin on 03/02/2022.
//

#ifndef FOCX_CONF_H
#define FOCX_CONF_H

/// Uncomment this when there is only one motor
#define ENABLE_2_CHANNEL_PWM


/// SPI pins
#define MISO 19
#define MOSI 23
#define CLK  18
#define CS0  22
#define CS1  5

/// ADC Parameters and current conversion
#define CH1_VOLTAGE_GAIN    50
#define CH2_VOLTAGE_GAIN    50
#define CH1_SAMPLING_RESISTANCE     3   // mOhm
#define CH2_SAMPLING_RESISTANCE     3   // mOhm
#define CH1_VOLTAGE2CURRENT(v) ((v) / CH1_VOLTAGE_GAIN / CH1_SAMPLING_RESISTANCE)   // mV to A
#define CH2_VOLTAGE2CURRENT(v) ((v) / CH2_VOLTAGE_GAIN / CH2_SAMPLING_RESISTANCE)


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
    #define CH1_ZERO_ELECTRICAL_ANGLE   2.879094
    #define CH2_ZERO_ELECTRICAL_ANGLE   0.949154
    #define CH1_SENSOR_DIRECTION    SensorDirection::CCW
    #define CH2_SENSOR_DIRECTION    SensorDirection::CCW
    #define CH1_REAL_ANGLE_OFFSET   0.0
    #define CH2_REAL_ANGLE_OFFSET   0.0
#elif MSG_ID == 0x002
    #define CH1_ZERO_ELECTRICAL_ANGLE   5.767382
    #define CH2_ZERO_ELECTRICAL_ANGLE   0.630849
    #define CH1_SENSOR_DIRECTION    SensorDirection::CCW
    #define CH2_SENSOR_DIRECTION    SensorDirection::CW
    #define CH1_REAL_ANGLE_OFFSET   0.0
    #define CH2_REAL_ANGLE_OFFSET   0.0
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

#endif //FOCX_CONF_H
