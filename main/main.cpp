#include <stdio.h>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_task_wdt.h"


extern "C" {
    void app_main(void);
}

#include "driver/MotorControlPWM.h"
#include "sensor/Tle5012b/TLE5012B.h"
#include "FOC/FOC.h"
#include "driver/UART_CLASS.h"


static void foc_task(void *arg);
static void monitor_task(void *arg);

TLE5012B tle5012B = TLE5012B(MISO, MOSI, CLK, CS);
FOC myFoc;
float target;

static SemaphoreHandle_t mutex;

void app_main(void) {
    //Initialize or reinitialize TWDT
    CHECK_ERROR_CODE(esp_task_wdt_init(TWDT_TIMEOUT_S, false), ESP_OK);

    mutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(
            monitor_task,                   // Function to be called
            "monitor_task",         // Name of task
            4096,            // Stack size in byte
            NULL,            // Parameter to pass to function
            1,                  // Task priority
            NULL,           // Task handle
            1                   // Core id
    );

    xTaskCreatePinnedToCore(
            foc_task,                   // Function to be called
            "foc_task",         // Name of task
            4096,            // Stack size in byte
            NULL,            // Parameter to pass to function
            1,                  // Task priority
            NULL,           // Task handle
            0                   // Core id
    );

}

static void foc_task(void *arg) {
    //Subscribe this task to TWDT, then check if it is subscribed
    CHECK_ERROR_CODE(esp_task_wdt_add(NULL), ESP_OK);
    CHECK_ERROR_CODE(esp_task_wdt_status(NULL), ESP_OK);

    if (xSemaphoreTake(mutex, 0) == pdTRUE) {
        myFoc.setPWMPins(25, 33, 32);
        myFoc.setVoltage(8.2, 2);
        myFoc.setPolePares(7);
        myFoc.setSensor(&tle5012B);
        myFoc.motorControlPwm.setMotorCH1ADCPins(37, 36);

        myFoc.iqPID.kp = 0.00048;
        myFoc.iqPID.ki = 0.000004;
        myFoc.iqPID.kd = 0;
        myFoc.iqPID.error_sum_constrain = 2.0;
        myFoc.iqPID.output_constrain = 2.0;

        myFoc.idPID.kp = 0.0002;
        myFoc.idPID.ki = 0.000003;
        myFoc.iqPID.kd = 0;
        myFoc.idPID.error_sum_constrain = 2.0;
        myFoc.idPID.output_constrain = 2.0;

        myFoc.mode = FOC_MODE::Torque;

        target = 800.0;

        myFoc.init();

        xSemaphoreGive(mutex);
    }

    //uint64_t last_time_stamp = esp_timer_get_time();
    //uint64_t current_time_stamp;
    //float angle = 0;
    while(1){
        if (xSemaphoreTake(mutex, 0) == pdTRUE) {
            //printf("Target:%f\n", target);
            myFoc.motorStart(target);
            xSemaphoreGive(mutex);
        }

        // Feed dog
        CHECK_ERROR_CODE(esp_task_wdt_reset(), ESP_OK);
    }
}

static void monitor_task(void *arg) {
    CHECK_ERROR_CODE(esp_task_wdt_add(NULL), ESP_OK);
    CHECK_ERROR_CODE(esp_task_wdt_status(NULL), ESP_OK);

    UART_CLASS uartClass;
    uartClass.init();

    char received[100];
    char *p = nullptr;
    const char *delim = "_";

    float parameter;
    bool mutex_flag = true;

    while(1) {
        if(uartClass.uart_read(received, 100)) {
            uartClass.uart_send(received);
            mutex_flag = true;
            p = strtok(received, delim);

            if(strcmp(p, "T") == 0) {
                p = strtok(nullptr, delim);
                parameter = strtof(p, nullptr);
                while(mutex_flag) {
                    if (xSemaphoreTake(mutex, 0) == pdTRUE) {
                        target = parameter;
                        printf("Successful: %f", target);
                        xSemaphoreGive(mutex);
                        mutex_flag = false;
                    }
                }
            }

            if(strcmp(p, "S") == 0) {
                p = strtok(nullptr, delim);
                if(strcmp(p, "C1") == 0) {
                    p = strtok(nullptr, delim);
                    if(strcmp(p, "CUR") == 0) {
                        p = strtok(nullptr, delim);
                        if(strcmp(p, "D") == 0) {
                            p = strtok(nullptr, delim);
                            if (strcmp(p, "P") == 0) {
                                p = strtok(nullptr, delim);
                                parameter = strtof(p, nullptr);
                                if (xSemaphoreTake(mutex, 0) == pdTRUE) {
                                    xSemaphoreGive(mutex);
                                }
                            }
                            if (strcmp(p, "I") == 0) {
                                p = strtok(nullptr, delim);
                                parameter = strtof(p, nullptr);
                            }
                            if (strcmp(p, "D") == 0) {
                                p = strtok(nullptr, delim);
                                parameter = strtof(p, nullptr);
                            }
                            if (strcmp(p, "CLPF") == 0) {
                                p = strtok(nullptr, delim);
                                parameter = strtof(p, nullptr);
                            }
                            if (strcmp(p, "VLPF") == 0) {
                                p = strtok(nullptr, delim);
                                parameter = strtof(p, nullptr);
                            }
                        }
                    }
                }
            }
        }
        p = nullptr;

        CHECK_ERROR_CODE(esp_task_wdt_reset(), ESP_OK);
    }

}