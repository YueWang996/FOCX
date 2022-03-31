//
// Created by Justin on 26/03/2022.
//

#ifndef FOCX_CANCOMMUNICATION_H
#define FOCX_CANCOMMUNICATION_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "CAN/CAN_config.h"
#include "CAN/ESP32CAN.h"
#include "../FOC/FOC.h"
#include "../conf.h"

/// Command
#define SET                             0x10
#define READ                            0x20
#define RETURN                          0x30

/// Variables
#ifdef ENABLE_2_CHANNEL_PWM
#define CH_NUMBER1                      0x01
#define CH_NUMBER2                      0x02

#define TARGET                          0x01
#define POSITION                        0x02
#define VELOCITY                        0x03

#define Q_KP                            0x02
#define Q_KI                            0x03
#define Q_KD                            0x04
#define Q_OUTPUT_CONSTRAINT             0x05
#define Q_ERROR_SUM_CONSTRAINT          0x06

#define D_KP                            0x07
#define D_KI                            0x08
#define D_KD                            0x09
#define D_OUTPUT_CONSTRAINT             0x0A
#define D_ERROR_SUM_CONSTRAINT          0x0B

#define VELOCITY_KP                     0x0C
#define VELOCITY_KI                     0x0D
#define VELOCITY_KD                     0x0E
#define VELOCITY_OUTPUT_CONSTRAINT      0x0F
#define VELOCITY_ERROR_SUM_CONSTRAINT   0x10

#define POSITION_KP                     0x11
#define POSITION_KI                     0x12
#define POSITION_KD                     0x13
#define POSITION_OUTPUT_CONSTRAINT      0x14
#define POSITION_ERROR_SUM_CONSTRAINT   0x15

#else
#define CH_NUMBER1                          0x01
#define TARGET                          0x01

#define Q_KP                            0x02
#define Q_KI                            0x03
#define Q_KD                            0x04
#define Q_OUTPUT_CONSTRAINT             0x05
#define Q_ERROR_SUM_CONSTRAINT          0x06

#define D_KP                            0x07
#define D_KI                            0x08
#define D_KD                            0x09
#define D_OUTPUT_CONSTRAINT             0x0A
#define D_ERROR_SUM_CONSTRAINT          0x0B

#define VELOCITY_KP                     0x0C
#define VELOCITY_KI                     0x0D
#define VELOCITY_KD                     0x0E
#define VELOCITY_OUTPUT_CONSTRAINT      0x0F
#define VELOCITY_ERROR_SUM_CONSTRAINT   0x10

#define POSITION_KP                     0x11
#define POSITION_KI                     0x12
#define POSITION_KD                     0x13
#define POSITION_OUTPUT_CONSTRAINT      0x14
#define POSITION_ERROR_SUM_CONSTRAINT   0x15
#endif

class CANCommunication : public ESP32CAN{
public:
    explicit CANCommunication(uint32_t id);
    void configFilter();
    void init();
    int8_t dataUnpack();
    void sendFloat(float data);
    void registerDevice();

#ifdef ENABLE_2_CHANNEL_PWM
    void registerDriver(FOC *foc_ch1, FOC *foc_ch2);
#else
    void registerDriver(FOC *foc);
#endif
    CAN_frame_t rx_frame;


private:
    float unpackFloatBit3To7();

    uint32_t can_device_id;

#ifdef ENABLE_2_CHANNEL_PWM
    FOC *_foc_ch1;
    FOC *_foc_ch2;
#else
    FOC *foc_ch1;
#endif
};


#endif //FOCX_CANCOMMUNICATION_H
