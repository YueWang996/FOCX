#include <stdio.h>
#include <cstring>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "esp_task_wdt.h"
#include "driver/MotorControlPWM.h"
#include "sensor/Tle5012b/TLE5012B.h"
#include "FOC/FOC.h"
#include "driver/SPI_CLASS.h"
#include "driver/UART_CLASS.h"
#include "driver/CANCommunication.h"
#include "conf.h"


extern "C" {
    void app_main(void);
}

/// Create magnetic sensors
TLE5012B tle5012BCH1 = TLE5012B(CS0);
TLE5012B tle5012BCH2 = TLE5012B(CS1);

/// Create foc drivers for each channel
FOC focCH1(Channel::CH1);
FOC focCH2(Channel::CH2);

/// Create motor control pwm unit
MotorControlPWM motorControlPwm;
/// PWM period, used in SVPWM
int ts;

/// SPI unit
MySPI spi;
uint8_t cs_pins[2] = {CS0, CS1};

/// CAN Bus
CANCommunication can(MSG_ID);

/// Two FOC task initialisation
static void foc_task_ch1_init();
static void foc_task_ch2_init();
bool InitialisationFinished = false;

/// Configuration task via UART
static void config_task(void *arg);

/// Message print task
static void print_task(void *arg);
static void debug_task(void *arg);
static void debug_task(void *arg){
    while(1) {
        if (focCH2.target == 0.0) {
            focCH1.target = 6.28;
            focCH2.target = 6.28;
        } else {
            focCH1.target = 0.0;
            focCH2.target = 0.0;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/// CAN bus task
static void CAN_task(void *arg);
CAN_device_t CAN_cfg;

/// FOC loops execution control. There are controlled by
/// FreeRTOS software timer to make sure FOC loop is
/// running in a fixed frequency
void focTimerCallback(TimerHandle_t xTimer);

/// FreeRTOS timer handler
static TimerHandle_t focTimer = nullptr;
void focTimerCallback(TimerHandle_t xTimer) {
    focCH1.motorStart();
    vTaskDelay(1 / portTICK_PERIOD_MS);
    focCH2.motorStart();
    //focCH1.getEAngle();
    //focCH2.getEAngle();
    //vTaskDelay(1 / portTICK_PERIOD_MS);
}

static void focTask(void *arg);
static void focTask(void *arg) {
    while(1) {
        focCH1.motorStart();
        focCH2.motorStart();
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void app_main(void) {
    // Initialize or reinitialize TWDT
    //CHECK_ERROR_CODE(esp_task_wdt_init(TWDT_TIMEOUT_S, false), ESP_OK);

    /// MCPWM initialisation
    ///     1 - set ADC pins for both channels
    ///     2 - set PWM pins for both channels
    ///     3 - initialise MCPWM
    motorControlPwm.setMotorCH1ADCPins(38, 39);
    motorControlPwm.setMotorCH2ADCPins(36, 37);
    three_phase_pwm_pins_t pwmPinsCH1{32, 33, 25};
    three_phase_pwm_pins_t pwmPinsCH2{14, 27, 26};
    motorControlPwm.setChannel1Pins(pwmPinsCH1);
    motorControlPwm.setChannel2Pins(pwmPinsCH2);
    motorControlPwm.init(&ts);

    /// SPI initialisation
    ///     1 - set SPI CS pins for both channels
    ///     2 - initialise SPI
    ///     3 - hook SPI upon to two encoders
    ///     4 - initialise encoders
    spi.set_pins(cs_pins, 2);
    spi.spi_init();
    tle5012BCH1.setSPI(&spi);
    tle5012BCH2.setSPI(&spi);
    tle5012BCH1.init();
    tle5012BCH2.init();

    foc_task_ch1_init();
    foc_task_ch2_init();
    InitialisationFinished = true;

    xTaskCreatePinnedToCore(
            focTask,                   // Function to be called
            "focTask",         // Name of task
            8192,            // Stack size in byte
            NULL,            // Parameter to pass to function
            1,                  // Task priority
            NULL,           // Task handle
            1                   // Core id
    );

    xTaskCreatePinnedToCore(
            CAN_task,                   // Function to be called
            "CAN_task",         // Name of task
            2048,            // Stack size in byte
            NULL,            // Parameter to pass to function
            1,                  // Task priority
            NULL,            // Task handle
            0                   // Core id
    );

    //xTaskCreate(
    //        print_task,                   // Function to be called
    //        "print_task",         // Name of task
    //        2048,            // Stack size in byte
    //        NULL,            // Parameter to pass to function
    //        1,                  // Task priority
    //        NULL           // Task handle
    //        //1                   // Core id
    //);

    //xTaskCreate(
    //        debug_task,                   // Function to be called
    //        "debug_task",         // Name of task
    //        1024,            // Stack size in byte
    //        NULL,            // Parameter to pass to function
    //        1,                  // Task priority
    //        NULL           // Task handle
    //        //1                   // Core id
    //);

    //xTaskCreate(
    //        config_task,                   // Function to be called
    //        "config_task",         // Name of task
    //        4096,            // Stack size in byte
    //        NULL,            // Parameter to pass to function
    //        1,                  // Task priority
    //        NULL           // Task handle
    //        //1                   // Core id
    //);
    vTaskDelete(NULL);
}

static void foc_task_ch1_init() {
    /// Configure some variables and connect MCPWM and Sensor
    focCH1.setVoltage(7.6, 2);
    focCH1.setPolePares(11);
    focCH1.setPWMController(&motorControlPwm);
    focCH1.setTs(ts);
    focCH1.setSensor(&tle5012BCH1);

    /// PID parameters
    focCH1.idPID.kp = 1.1;
    focCH1.idPID.ki = 0.2;
    focCH1.idPID.kd = 0.0;
    focCH1.idPID.error_sum_constrain = 2.0;
    focCH1.idPID.output_constrain = 2.0;
    focCH1.idPID.last_error = 0;

    focCH1.iqPID.kp = 1.1;
    focCH1.iqPID.ki = 0.2;
    focCH1.iqPID.kd = 0.0;
    focCH1.iqPID.error_sum_constrain = 2.0;
    focCH1.iqPID.output_constrain = 2.0;
    focCH1.iqPID.last_error = 0;

    focCH1.velocityPID.kp = 0.0;
    focCH1.velocityPID.ki = 0.0;
    focCH1.velocityPID.kd = 0;
    focCH1.velocityPID.error_sum_constrain = 50.0;
    focCH1.velocityPID.output_constrain = 50.0;
    focCH1.velocityPID.last_error = 0;

    focCH1.positionPID.kp = 0.8;
    focCH1.positionPID.ki = 0.1;
    focCH1.positionPID.kd = 0.1;
    focCH1.positionPID.error_sum_constrain = 3.0;
    focCH1.positionPID.output_constrain = 3.0;
    focCH1.positionPID.last_error = 0;

    /// Mode set
    focCH1.mode = FOC_MODE::Position;

    /// Target set
    focCH1.target = 0.0;

    focCH1.init();

}

static void foc_task_ch2_init() {
    focCH2.setVoltage(7.6, 2);
    focCH2.setPolePares(11);
    focCH2.setPWMController(&motorControlPwm);
    focCH2.setTs(ts);
    focCH2.setSensor(&tle5012BCH2);

    focCH2.idPID.kp = 1.1;
    focCH2.idPID.ki = 0.2;
    focCH2.idPID.kd = 0.0;
    focCH2.idPID.error_sum_constrain = 2.0;
    focCH2.idPID.output_constrain = 2.0;
    focCH2.idPID.last_error = 0;

    focCH2.iqPID.kp = 1.1;
    focCH2.iqPID.ki = 0.2;
    focCH2.iqPID.kd = 0.0;
    focCH2.iqPID.error_sum_constrain = 2.0;
    focCH2.iqPID.output_constrain = 2.0;
    focCH2.iqPID.last_error = 0;

    focCH2.velocityPID.kp = 0.0;
    focCH2.velocityPID.ki = 0.0;
    focCH2.velocityPID.kd = 0;
    focCH2.velocityPID.error_sum_constrain = 50.0;
    focCH2.velocityPID.output_constrain = 50.0;
    focCH2.velocityPID.last_error = 0;

    focCH2.positionPID.kp = 0.8;
    focCH2.positionPID.ki = 0.1;
    focCH2.positionPID.kd = 0.1;
    focCH2.positionPID.error_sum_constrain = 3.0;
    focCH2.positionPID.output_constrain = 3.0;
    focCH2.positionPID.last_error = 0;

    focCH2.mode = FOC_MODE::Position;

    focCH2.target = 0.0;

    focCH2.init();

}

static void print_task(void *arg) {
    CHECK_ERROR_CODE(esp_task_wdt_add(NULL), ESP_OK);
    CHECK_ERROR_CODE(esp_task_wdt_status(NULL), ESP_OK);
    while(1) {
        //printf("M1id:%f, M1iq:%f\n", focCH1.id, focCH1.iq);
        //printf("M2id:%f, M2iq:%f, M2Angle:%f\n", focCH2.id, focCH2.iq, focCH2.realAngle);
        //printf("M1U:%f, M1V:%f, M1W:%f\n", CH1_VOLTAGE2CURRENT(focCH1.phaseVoltage.U) *100, CH1_VOLTAGE2CURRENT(focCH1.phaseVoltage.V) * 100, (-CH1_VOLTAGE2CURRENT(focCH1.phaseVoltage.U) - CH1_VOLTAGE2CURRENT(focCH1.phaseVoltage.V)) * 100);
        //printf("M2U:%f, M2V:%f\n", focCH2.phaseVoltage.U*100, focCH2.phaseVoltage.V*100);
        //printf("CH1Angle:%f, CH2Angle:%f\n", focCH1.realAngle, focCH2.realAngle);
        printf("CH1ZeroElectricalAngle:%f, CH2ZeroElectricalAngle:%f\n", focCH1.zero_electrical_angle, focCH2.zero_electrical_angle);
        printf("CH1SensorDir:%d, CH2SensorDir:%d\n", focCH1.direction, focCH2.direction);
        //printf("velocity:%f\n", focCH1.velocity);
        vTaskDelay(50/portTICK_PERIOD_MS);
        CHECK_ERROR_CODE(esp_task_wdt_reset(), ESP_OK);
    }
}

static void config_task(void *arg) {
    UART_CLASS uartClass;
    uartClass.init();

    char received[100];
    char *p = nullptr;
    const char *delim = "_";

    float parameter;
    bool mutex_flag;

    while(1) {
        if (uartClass.uart_read(received, 100)) {
            uartClass.uart_send(received);
            mutex_flag = true;
            p = strtok(received, delim);

            // Change target
            if (strcmp(p, "T") == 0) {
                p = strtok(nullptr, delim);
                if (strcmp(p, "1") == 0) {
                    p = strtok(nullptr, delim);
                    parameter = strtof(p, nullptr);
                    focCH1.target = parameter;
                    uartClass.uart_send("Success setting target!\n");
                }
                if (strcmp(p, "2") == 0) {
                    p = strtok(nullptr, delim);
                    parameter = strtof(p, nullptr);
                    focCH2.target = parameter;
                    uartClass.uart_send("Success setting target!\n");
                }
            }

            if(strcmp(p, "S") == 0) {
                p = strtok(nullptr, delim);
                if (strcmp(p, "P") == 0) {
                    p = strtok(nullptr, delim);
                    parameter = strtof(p, nullptr);
                    focCH1.iqPID.kp = parameter;
                    uartClass.uart_send("Success setting kp!\n");
                }
                if (strcmp(p, "I") == 0) {
                    p = strtok(nullptr, delim);
                    parameter = strtof(p, nullptr);
                    focCH1.iqPID.ki = parameter;
                    uartClass.uart_send("Success setting ki!\n");
                }
                if (strcmp(p, "D") == 0) {
                    p = strtok(nullptr, delim);
                    parameter = strtof(p, nullptr);
                    focCH1.iqPID.kd = parameter;
                    uartClass.uart_send("Success setting kd!\n");
                }
            }
        }
        p = nullptr;
    }
}

static void CAN_task(void *arg) {
    can.registerDriver(&focCH1, &focCH2);
    CAN_cfg.speed = CAN_SPEED;
    CAN_cfg.tx_pin_id = GPIO_NUM_2;
    CAN_cfg.rx_pin_id = GPIO_NUM_15;
    CAN_cfg.rx_queue = xQueueCreate(CAN_RX_QUEUE_SIZE, sizeof(CAN_frame_t));

    can.configFilter();
    can.init();

    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = MSG_ID;
    tx_frame.FIR.B.RTR =CAN_RTR;
    tx_frame.FIR.B.DLC = 0;
    ESP32Can.CANWriteFrame(&tx_frame);

    //can.registerDevice();

    while(1) {
        if (xQueueReceive(CAN_cfg.rx_queue, &can.rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {
            if (can.rx_frame.FIR.B.RTR == CAN_no_RTR) {
                if(can.rx_frame.MsgID == MSG_ID) {
                    can.dataUnpack();
                }
            }
            if (can.rx_frame.FIR.B.RTR == CAN_RTR) {
                //printf(" RTR from 0x%08X, DLC %d\r\n", can.rx_frame.MsgID,  can.rx_frame.FIR.B.DLC);
            }
        }
    }
}