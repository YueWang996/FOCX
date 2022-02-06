#include <stdio.h>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"


extern "C" {
    void app_main(void);
}

#include "driver/MotorControlPWM.h"
#include "sensor/Tle5012b/TLE5012B.h"
#include "FOC/FOC.h"
#include "conf.h"
#include "driver/UART_CLASS.h"


void app_main(void)
{
    TLE5012B tle5012B = TLE5012B(MISO, MOSI, CLK, CS);
    FOC myFoc;

    myFoc.setPWMPins(25, 33, 32);
    myFoc.setVoltage(8.2, 2);
    myFoc.setPolePares(7);
    myFoc.setSensor(&tle5012B);
    myFoc.motorControlPwm.setMotorCH1ADCPins(37, 36);

    myFoc.iqPID.kp = 0.00048;
    myFoc.iqPID.ki = 0.000004;
    myFoc.iqPID.error_sum_constrain = 2.0;
    myFoc.iqPID.output_constrain = 2.0;

    myFoc.idPID.kp = 0.0002;
    myFoc.idPID.ki = 0.000003;
    myFoc.idPID.error_sum_constrain = 2.0;
    myFoc.idPID.output_constrain = 2.0;

    myFoc.init();

    //uint64_t last_time_stamp = esp_timer_get_time();
    //uint64_t current_time_stamp;
    //float angle = 0;
    while(1){
        TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
        TIMERG0.wdt_feed=1;
        TIMERG0.wdt_wprotect=0;

        //current_time_stamp = esp_timer_get_time();
        //if(current_time_stamp - last_time_stamp >= 500000) {
        //    (myFoc.target_id == 0) ? (myFoc.target_id = 0.4) : (myFoc.target_id = 0);
        //    last_time_stamp = current_time_stamp;
        //}
        myFoc.motorStart(1200);
    }
}
