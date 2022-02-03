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


void app_main(void)
{
    //TLE5012B tle5012B = TLE5012B(MISO, MOSI, CLK, CS);
    //tle5012B.init();
    //
    //double angle;
    //double absAngle;
    //int16_t revolution;
    //double velocity;
    //for(;;) {
    //    tle5012B.update();
    //    angle = tle5012B.getAngleValue();
    //    absAngle = tle5012B.getAbsoluteAngleValue();
    //    revolution = tle5012B.getRevolutionValue();
    //    velocity = tle5012B.getAngularVelocity();
    //
    //    printf("Angle:%f, Abs_angle: %f, rev:%d, velocity:%f\n", angle, absAngle, revolution, velocity);
    //
    //    vTaskDelay(pdMS_TO_TICKS(10));
    //}
    //MotorControlPWM mcpwm;
    //mcpwm.channel1 = {.p1 = 25, .p2 = 33, .p3 = 32};
    //mcpwm.setChannel1Pins(mcpwm.channel1);
    //int ts;
    //mcpwm.init(&ts);
    //mcpwm.setChannel1Duty(80.0, 40.0, 20.0);
    TLE5012B tle5012B = TLE5012B(MISO, MOSI, CLK, CS);
    FOC myFoc;
    myFoc.setPWMPins(25, 33, 32);
    myFoc.setVoltage(8.2, 2);
    myFoc.setPolePares(7);
    myFoc.setSensor(&tle5012B);

    myFoc.init();
    //float angle = 0;
    while(1){
        TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
        TIMERG0.wdt_feed=1;
        TIMERG0.wdt_wprotect=0;
        //if(angle > 6.27) angle = 0;
        myFoc.svpwm(0.8, 0.0, myFoc.getEAngle());
        //angle += 0.01;
        //vTaskDelay(pdMS_TO_TICKS(10));
    }
}
