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
//CANCommunication can(MSG_ID);

/// Battery voltage
float battery_voltage = 8.2;
float voltage_limit = 2.5;

/// Two FOC task initialisation
static void foc_task_ch1_init();
static void foc_task_ch2_init();
bool InitialisationFinished = false;

/// Configuration task via UART
static void config_task(void *arg);

/// Message print task
static void print_task(void *arg);

/// CAN bus task
static void CAN_task(void *arg);
//CAN_device_t CAN_cfg;
bool CAN_CONNECTED = false;

/// FOC loops execution control. There are controlled by
/// FreeRTOS software timer to make sure FOC loop is
/// running in a fixed frequency
void focTimerCallback(TimerHandle_t xTimer);

/// FreeRTOS timer handler
static TimerHandle_t reportPositionTimer = nullptr;

/*
void reportPositionTimerCallback(TimerHandle_t xTimer) {
    if(CAN_CONNECTED) {
        can.reportSensorPosition();
        can.reportSensorVelocity();
    }
}
*/

static void focTask(void *arg);
static void focTask(void *arg) {
    while(1) {
        focCH1.motorStart();
        //focCH2.motorStart();
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

static TimerHandle_t changeDirectionTimer = nullptr;
void changeDirectionTimerCallback(TimerHandle_t xTimer) {
    if(focCH1.target < 3.14) {
        focCH1.target = 3.142;
    } else {
        focCH1.target = -3.142;
    }
}

void app_main(void) {
    // Initialize or reinitialize TWDT
    //CHECK_ERROR_CODE(esp_task_wdt_init(TWDT_TIMEOUT_S, false), ESP_OK);

    /// MCPWM initialisation
    ///     1 - set ADC pins for both channels
    ///     2 - set PWM pins for both channels
    ///     3 - initialise MCPWM
    motorControlPwm.setMotorCH1ADCPins(39, 38);
    //motorControlPwm.setMotorCH2ADCPins(36, 37);
    three_phase_pwm_pins_t pwmPinsCH1{25, 33, 32};
    three_phase_pwm_pins_t pwmPinsCH2{14, 27, 26};
    motorControlPwm.setChannel1Pins(pwmPinsCH1);
    //motorControlPwm.setChannel2Pins(pwmPinsCH2);
    motorControlPwm.init(&ts);

    /// SPI initialisation
    ///     1 - set SPI CS pins for both channels
    ///     2 - initialise SPI
    ///     3 - hook SPI upon two encoders
    ///     4 - initialise encoders
    spi.set_pins(cs_pins, 2);
    spi.spi_init();
    tle5012BCH1.setSPI(&spi);
    //tle5012BCH2.setSPI(&spi);
    tle5012BCH1.init();
    //tle5012BCH2.init();

    foc_task_ch1_init();
    //foc_task_ch2_init();
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

    //changeDirectionTimer = xTimerCreate(
    //        "changeDirectionTimer",
    //        1500 / portTICK_PERIOD_MS,
    //        pdTRUE,
    //        (void *) 0,
    //        changeDirectionTimerCallback
    //);
    //if(changeDirectionTimer == nullptr) {
    //    printf("Could not create the reportPositionTimer!\n");
    //    while(1);
    //} else {
    //    xTimerStart(changeDirectionTimer, portMAX_DELAY);
    //}

    //xTaskCreatePinnedToCore(
    //        CAN_task,                   // Function to be called
    //        "CAN_task",         // Name of task
    //        4096,            // Stack size in byte
    //        NULL,            // Parameter to pass to function
    //        1,                  // Task priority
    //        NULL,            // Task handle
    //        0                   // Core id
    //);

    //xTaskCreatePinnedToCore(
    //        print_task,                   // Function to be called
    //        "print_task",         // Name of task
    //        2048,            // Stack size in byte
    //        NULL,            // Parameter to pass to function
    //        1,                  // Task priority
    //        NULL,           // Task handle
    //        0                   // Core id
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

    xTaskCreate(
            config_task,                   // Function to be called
            "config_task",         // Name of task
            4096,            // Stack size in byte
            NULL,            // Parameter to pass to function
            1,                  // Task priority
            NULL           // Task handle
            //1                   // Core id
    );
    vTaskDelete(NULL);
}

static void foc_task_ch1_init() {
    /// Configure some variables and connect MCPWM and Sensor
    focCH1.setVoltage(battery_voltage, voltage_limit);
    focCH1.setPolePares(7);
    focCH1.setPWMController(&motorControlPwm);
    focCH1.setTs(ts);
    focCH1.setSensor(&tle5012BCH1);

    /// PID parameters
    focCH1.idPID.kp = 0.75;
    focCH1.idPID.ki = 0.1;
    focCH1.idPID.kd = 0.0;
    focCH1.idPID.error_sum_constrain = 5.0;
    focCH1.idPID.output_constrain = 5.0;
    focCH1.idPID.last_error = 0;

    focCH1.iqPID.kp = 0.75;
    focCH1.iqPID.ki = 0.1;
    focCH1.iqPID.kd = 0.0;
    focCH1.iqPID.error_sum_constrain = 5.0;
    focCH1.iqPID.output_constrain = 5.0;
    focCH1.iqPID.last_error = 0;

    focCH1.velocityPID.kp = 0.0;
    focCH1.velocityPID.ki = 0.0;
    focCH1.velocityPID.kd = 0.0;
    focCH1.velocityPID.error_sum_constrain = 50.0;
    focCH1.velocityPID.output_constrain = 50.0;
    focCH1.velocityPID.last_error = 0;

    focCH1.positionPID.kp = 0.8;
    focCH1.positionPID.ki = 0.1;
    focCH1.positionPID.kd = 0.1;
    focCH1.positionPID.error_sum_constrain = 10.0;
    focCH1.positionPID.output_constrain = 10.0;
    focCH1.positionPID.last_error = 0;

    /// Mode set
    focCH1.mode = FOC_MODE::Torque;

    /// Target set
    focCH1.target = 0.0;

    focCH1.init();

}

static void foc_task_ch2_init() {
    focCH2.setVoltage(battery_voltage, voltage_limit);
    focCH2.setPolePares(11);
    focCH2.setPWMController(&motorControlPwm);
    focCH2.setTs(ts);
    focCH2.setSensor(&tle5012BCH2);

    focCH2.idPID.kp = 0.75;
    focCH2.idPID.ki = 0.1;
    focCH2.idPID.kd = 0.0;
    focCH2.idPID.error_sum_constrain = 5.0;
    focCH2.idPID.output_constrain = 5.0;
    focCH2.idPID.last_error = 0;

    focCH2.iqPID.kp = 0.75;
    focCH2.iqPID.ki = 0.1;
    focCH2.iqPID.kd = 0.0;
    focCH2.iqPID.error_sum_constrain = 5.0;
    focCH2.iqPID.output_constrain = 5.0;
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
    focCH2.positionPID.error_sum_constrain = 10.0;
    focCH2.positionPID.output_constrain = 10.0;
    focCH2.positionPID.last_error = 0;

    focCH2.mode = FOC_MODE::Torque;

    focCH2.target = 0.0;

    //focCH2.init();

}

static void print_task(void *arg) {
    CHECK_ERROR_CODE(esp_task_wdt_add(NULL), ESP_OK);
    CHECK_ERROR_CODE(esp_task_wdt_status(NULL), ESP_OK);
    while(1) {
        //printf("t1:%f, t2:%f\n", focCH1.target, focCH1.target);
        //printf("M1id:%f, M1iq:%f\n", focCH1.id, focCH1.iq);
        //printf("M2id:%f, M2iq:%f, M2Angle:%f\n", focCH2.id, focCH2.iq, focCH2.realAngle);
        //printf("M1U:%f, M1V:%f, M1W:%f\n", CH1_VOLTAGE2CURRENT(focCH1.phaseVoltage.U) *100, CH1_VOLTAGE2CURRENT(focCH1.phaseVoltage.V) * 100, (-CH1_VOLTAGE2CURRENT(focCH1.phaseVoltage.U) - CH1_VOLTAGE2CURRENT(focCH1.phaseVoltage.V)) * 100);
        //printf("M2U:%f, M2V:%f\n", focCH2.phaseVoltage.U, focCH2.phaseVoltage.V);
        //printf("CH1Angle:%f, CH2Angle:%f\n", focCH1.realAngle, focCH2.realAngle);
        //printf("CH1ZeroElectricalAngle:%f, CH2ZeroElectricalAngle:%f\n", focCH1.zero_electrical_angle, focCH2.zero_electrical_angle);
        //printf("CH1SensorDir:%d, CH2SensorDir:%d\n", focCH1.direction, focCH2.direction);
        printf("DesiredPosition:%f,RealPosition:%f\n", focCH1.target, focCH1.realAngle);
        vTaskDelay(100/portTICK_PERIOD_MS);
        CHECK_ERROR_CODE(esp_task_wdt_reset(), ESP_OK);
    }
}

void byteArrayToFloat(uint8_t *byteArray, float *f) {
    ((uint8_t*)f)[0] = byteArray[1];
    ((uint8_t*)f)[1] = byteArray[2];
    ((uint8_t*)f)[2] = byteArray[3];
    ((uint8_t*)f)[3] = byteArray[4];
}

uint8_t DATA_LIST[10] = {S_DATA_Q_PHASE_KP,
        S_DATA_Q_PHASE_KI,
        S_DATA_Q_PHASE_KD,
        S_DATA_Q_PHASE_MAX,
        S_DATA_Q_PHASE_MAX_ERR,
        S_DATA_D_PHASE_KP,
        S_DATA_D_PHASE_KI,
        S_DATA_D_PHASE_KD,
        S_DATA_D_PHASE_MAX,
        S_DATA_D_PHASE_MAX_ERR};

static void config_task(void *arg) {
    UART_CLASS uartClass;
    uartClass.init();

    uint8_t received[16];
    float *data = nullptr;
    float sampling_rate = 100;
    uint8_t current_info[9] = {0};
    uint8_t sensor_info[9] = {0};
    while(1) {
        if (uartClass.uart_read(received, 16)) {
            uint8_t opcode = received[0];
            switch (opcode) {
                case S_SET_TARGET:
                    byteArrayToFloat(received, &focCH1.target);
                    break;
                case S_PULL_PARAMETERS:{
                    uint8_t buf[6] = {0};
                    for (uint8_t par : DATA_LIST) {
                        buf[0] = S_PULL_PARAMETERS;
                        buf[1] = par;
                        if(par == S_DATA_Q_PHASE_KP) {
                            data = &focCH1.iqPID.kp;
                            buf[2] = ((uint8_t*)data)[0];
                            buf[3] = ((uint8_t*)data)[1];
                            buf[4] = ((uint8_t*)data)[2];
                            buf[5] = ((uint8_t*)data)[3];
                        } else if(par == S_DATA_Q_PHASE_KI) {
                            data = &focCH1.iqPID.ki;
                            buf[2] = ((uint8_t*)data)[0];
                            buf[3] = ((uint8_t*)data)[1];
                            buf[4] = ((uint8_t*)data)[2];
                            buf[5] = ((uint8_t*)data)[3];
                        } else if(par == S_DATA_Q_PHASE_KD) {
                            data = &focCH1.iqPID.kd;
                            buf[2] = ((uint8_t*)data)[0];
                            buf[3] = ((uint8_t*)data)[1];
                            buf[4] = ((uint8_t*)data)[2];
                            buf[5] = ((uint8_t*)data)[3];
                        } else if(par == S_DATA_Q_PHASE_MAX) {
                            data = &focCH1.iqPID.output_constrain;
                            buf[2] = ((uint8_t*)data)[0];
                            buf[3] = ((uint8_t*)data)[1];
                            buf[4] = ((uint8_t*)data)[2];
                            buf[5] = ((uint8_t*)data)[3];
                        } else if(par == S_DATA_Q_PHASE_MAX_ERR) {
                            data = &focCH1.iqPID.error_sum_constrain;
                            buf[2] = ((uint8_t*)data)[0];
                            buf[3] = ((uint8_t*)data)[1];
                            buf[4] = ((uint8_t*)data)[2];
                            buf[5] = ((uint8_t*)data)[3];
                        } else if(par == S_DATA_D_PHASE_KP) {
                            data = &focCH1.idPID.kp;
                            buf[2] = ((uint8_t*)data)[0];
                            buf[3] = ((uint8_t*)data)[1];
                            buf[4] = ((uint8_t*)data)[2];
                            buf[5] = ((uint8_t*)data)[3];
                        } else if(par == S_DATA_D_PHASE_KI) {
                            data = &focCH1.idPID.ki;
                            buf[2] = ((uint8_t*)data)[0];
                            buf[3] = ((uint8_t*)data)[1];
                            buf[4] = ((uint8_t*)data)[2];
                            buf[5] = ((uint8_t*)data)[3];
                        } else if(par == S_DATA_D_PHASE_KD) {
                            data = &focCH1.idPID.kd;
                            buf[2] = ((uint8_t*)data)[0];
                            buf[3] = ((uint8_t*)data)[1];
                            buf[4] = ((uint8_t*)data)[2];
                            buf[5] = ((uint8_t*)data)[3];
                        } else if(par == S_DATA_D_PHASE_MAX) {
                            data = &focCH1.idPID.output_constrain;
                            buf[2] = ((uint8_t*)data)[0];
                            buf[3] = ((uint8_t*)data)[1];
                            buf[4] = ((uint8_t*)data)[2];
                            buf[5] = ((uint8_t*)data)[3];
                        } else if(par == S_DATA_D_PHASE_MAX_ERR) {
                            data = &focCH1.idPID.error_sum_constrain;
                            buf[2] = ((uint8_t*)data)[0];
                            buf[3] = ((uint8_t*)data)[1];
                            buf[4] = ((uint8_t*)data)[2];
                            buf[5] = ((uint8_t*)data)[3];
                        }
                        uartClass.uart_send(buf, 6);
                        data = nullptr;
                        vTaskDelay(10);
                    }
                    data = nullptr;
                    break;}
                case S_SET_Q_PHASE_KP:
                    byteArrayToFloat(received, &focCH1.iqPID.kp);
                    break;
                case S_SET_Q_PHASE_KI:
                    byteArrayToFloat(received, &focCH1.iqPID.ki);
                    break;
                case S_SET_Q_PHASE_KD:
                    byteArrayToFloat(received, &focCH1.iqPID.kd);
                    break;
                case S_SET_Q_PHASE_MAX:
                    byteArrayToFloat(received, &focCH1.iqPID.output_constrain);
                    break;
                case S_SET_Q_PHASE_MAX_ERR:
                    byteArrayToFloat(received, &focCH1.iqPID.error_sum_constrain);
                    break;
                case S_SET_D_PHASE_KP:
                    byteArrayToFloat(received, &focCH1.idPID.kp);
                    break;
                case S_SET_D_PHASE_KI:
                    byteArrayToFloat(received, &focCH1.idPID.ki);
                    break;
                case S_SET_D_PHASE_KD:
                    byteArrayToFloat(received, &focCH1.idPID.kd);
                    break;
                case S_SET_D_PHASE_MAX:
                    byteArrayToFloat(received, &focCH1.idPID.output_constrain);
                    break;
                case S_SET_D_PHASE_MAX_ERR:
                    byteArrayToFloat(received, &focCH1.idPID.error_sum_constrain);
                    break;
                case S_SET_SAMPLING_FREQ:
                    byteArrayToFloat(received, &sampling_rate);
                    break;
                case S_SET_FOC_MODE:
                    if(received[1] == S_SET_TORQUE_MODE) focCH1.mode = FOC_MODE::Torque;
                    else if(received[1] == S_SET_POSITION_MODE) focCH1.mode = FOC_MODE::Position;
                    break;
            }
        }
        current_info[0] = S_PLOT_CURRENT;
        current_info[1] = ((uint8_t*)&focCH1.id)[0];
        current_info[2] = ((uint8_t*)&focCH1.id)[1];
        current_info[3] = ((uint8_t*)&focCH1.id)[2];
        current_info[4] = ((uint8_t*)&focCH1.id)[3];
        current_info[5] = ((uint8_t*)&focCH1.iq)[0];
        current_info[6] = ((uint8_t*)&focCH1.iq)[1];
        current_info[7] = ((uint8_t*)&focCH1.iq)[2];
        current_info[8] = ((uint8_t*)&focCH1.iq)[3];
        uartClass.uart_send(current_info, 9);
        vTaskDelay((int)(10000.0f/sampling_rate));
        sensor_info[0] = S_PLOT_ANGLE_VELOCITY;
        sensor_info[1] = ((uint8_t*)&focCH1.velocity)[0];
        sensor_info[2] = ((uint8_t*)&focCH1.velocity)[1];
        sensor_info[3] = ((uint8_t*)&focCH1.velocity)[2];
        sensor_info[4] = ((uint8_t*)&focCH1.velocity)[3];
        sensor_info[5] = ((uint8_t*)&focCH1.realAngle)[0];
        sensor_info[6] = ((uint8_t*)&focCH1.realAngle)[1];
        sensor_info[7] = ((uint8_t*)&focCH1.realAngle)[2];
        sensor_info[8] = ((uint8_t*)&focCH1.realAngle)[3];
        uartClass.uart_send(sensor_info, 9);
        vTaskDelay((int)(10000.0f/sampling_rate));
    }
}

/*
static void CAN_task(void *arg) {
    can.registerDriver(&focCH1, &focCH2);
    CAN_cfg.speed = CAN_SPEED_500KBPS;
    CAN_cfg.tx_pin_id = GPIO_NUM_2;
    CAN_cfg.rx_pin_id = GPIO_NUM_15;
    CAN_cfg.rx_queue = xQueueCreate(CAN_RX_QUEUE_SIZE, sizeof(CAN_frame_t));

    //can.configFilter();
    //can.init();
    ESP32Can.CANInit();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = MSG_ID;
    tx_frame.FIR.B.RTR =CAN_RTR;
    tx_frame.FIR.B.DLC = 0;

    //vTaskDelay(100 / portTICK_PERIOD_MS);

    reportPositionTimer = xTimerCreate(
            "reportPositionTimer",
            5 / portTICK_PERIOD_MS,
            pdTRUE,
            (void *) 0,
            reportPositionTimerCallback
    );
    if(reportPositionTimer == nullptr) {
        printf("Could not create the reportPositionTimer!\n");
        while(1);
    } else {
        xTimerStart(reportPositionTimer, portMAX_DELAY);
    }

    //can.registerDevice();
    ESP32Can.CANWriteFrame(&tx_frame);
    printf("Start frame is sent\n");

    while(1) {
        if (xQueueReceive(CAN_cfg.rx_queue, &can.rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {
            //printf("xQueueReceive! ID: %d, length: %d\n", can.rx_frame.MsgID, can.rx_frame.FIR.B.DLC);
            if (can.rx_frame.FIR.B.RTR == CAN_no_RTR) {
                can.dataUnpack();
            }
            if (can.rx_frame.FIR.B.RTR == CAN_RTR) {
                if(can.rx_frame.MsgID == MASTER_RESPONSE) {
                    printf("CAN_CONNECTED!\n");
                    CAN_CONNECTED = true;
                }
                //printf(" RTR from 0x%08X, DLC %d\r\n", can.rx_frame.MsgID,  can.rx_frame.FIR.B.DLC);
            }
        }
    }
}
*/