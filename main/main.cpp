#include <stdio.h>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" {
    void app_main(void);
}

//#include "driver/spi_master.h"
//#include "driver/gpio.h"
#include "sensor/Tle5012b/TLE5012B.h"


#define MISO 18
#define MOSI 5
#define CLK  23
#define CS   19

void app_main(void)
{
    TLE5012B tle5012B = TLE5012B(MISO, MOSI, CLK, CS);
    tle5012B.init();

    double angle;
    double absAngle;
    int16_t revolution;
    double velocity;
    for(;;) {
        tle5012B.update();
        angle = tle5012B.getAngleValue();
        absAngle = tle5012B.getAbsoluteAngleValue();
        revolution = tle5012B.getRevolutionValue();
        velocity = tle5012B.getAngularVelocity();

        printf("Angle:%f, Abs_angle: %f, rev:%d, velocity:%f\n", angle, absAngle, revolution, velocity);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
