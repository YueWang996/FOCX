//
// Created by Justin on 26/03/2022.
//

#include "CANCommunication.h"

CANCommunication::CANCommunication(uint32_t id) {
    can_device_id = id;
}

void CANCommunication::init() {
    // Init CAN Module
    ESP32Can.CANInit();
}

void CANCommunication::sendFloat(float data) {
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = can_device_id;
    tx_frame.FIR.B.RTR =CAN_no_RTR;
    tx_frame.FIR.B.DLC = sizeof(float);
    xthal_memcpy(tx_frame.data.u8, &data, sizeof(float));
    ESP32Can.CANWriteFrame(&tx_frame);
}

void CANCommunication::registerDevice() {
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = can_device_id;
    tx_frame.FIR.B.RTR =CAN_RTR;
    tx_frame.FIR.B.DLC = 0;
    ESP32Can.CANWriteFrame(&tx_frame);
}

float CANCommunication::unpackFloatBit3To7() {
    float d;
    ((uint8_t*)&d)[0] = rx_frame.data.u8[3];
    ((uint8_t*)&d)[1] = rx_frame.data.u8[4];
    ((uint8_t*)&d)[2] = rx_frame.data.u8[5];
    ((uint8_t*)&d)[3] = rx_frame.data.u8[6];
    return d;
}

int8_t CANCommunication::dataUnpack() {
    //if(rx_frame.MsgID != MSG_ID) return CAN_ERROR::UnMatchedID;

    if(rx_frame.data.u8[0] == SET) {
        if(rx_frame.data.u8[1] == CH_NUMBER1) {
            switch (rx_frame.data.u8[2]) {
                case TARGET: {
                    _foc_ch1->target = unpackFloatBit3To7(); break;
                }
                case Q_KP: {
                    _foc_ch1->iqPID.kp = unpackFloatBit3To7(); break;
                }
                case Q_KI: {
                    _foc_ch1->iqPID.ki = unpackFloatBit3To7(); break;
                }
                case Q_KD: {
                    _foc_ch1->iqPID.kd = unpackFloatBit3To7(); break;
                }
                case Q_OUTPUT_CONSTRAINT: {
                    _foc_ch1->iqPID.output_constrain = unpackFloatBit3To7(); break;
                }
                case Q_ERROR_SUM_CONSTRAINT: {
                    _foc_ch1->iqPID.error_sum_constrain = unpackFloatBit3To7(); break;
                }
                case D_KP: {
                    _foc_ch1->idPID.kp = unpackFloatBit3To7(); break;
                }
                case D_KI: {
                    _foc_ch1->idPID.ki = unpackFloatBit3To7(); break;
                }
                case D_KD: {
                    _foc_ch1->idPID.kd = unpackFloatBit3To7(); break;
                }
                case D_OUTPUT_CONSTRAINT: {
                    _foc_ch1->idPID.output_constrain = unpackFloatBit3To7(); break;
                }
                case D_ERROR_SUM_CONSTRAINT: {
                    _foc_ch1->idPID.error_sum_constrain = unpackFloatBit3To7(); break;
                }
                case VELOCITY_KP: {
                    _foc_ch1->velocityPID.kp = unpackFloatBit3To7(); break;
                }
                case VELOCITY_KI: {
                    _foc_ch1->velocityPID.ki = unpackFloatBit3To7(); break;
                }
                case VELOCITY_KD: {
                    _foc_ch1->velocityPID.kd = unpackFloatBit3To7(); break;
                }
                case VELOCITY_OUTPUT_CONSTRAINT: {
                    _foc_ch1->velocityPID.output_constrain = unpackFloatBit3To7(); break;
                }
                case VELOCITY_ERROR_SUM_CONSTRAINT: {
                    _foc_ch1->velocityPID.error_sum_constrain = unpackFloatBit3To7(); break;
                }
                case POSITION_KP: {
                    _foc_ch1->positionPID.kp = unpackFloatBit3To7(); break;
                }
                case POSITION_KI: {
                    _foc_ch1->positionPID.ki = unpackFloatBit3To7(); break;
                }
                case POSITION_KD: {
                    _foc_ch1->positionPID.kd = unpackFloatBit3To7(); break;
                }
                case POSITION_OUTPUT_CONSTRAINT: {
                    _foc_ch1->positionPID.output_constrain = unpackFloatBit3To7(); break;
                }
                case POSITION_ERROR_SUM_CONSTRAINT: {
                    _foc_ch1->positionPID.error_sum_constrain = unpackFloatBit3To7(); break;
                }
                default: {
                    return CAN_ERROR::UnMatchedCommand;
                }
            }
        }

        if(rx_frame.data.u8[1] == CH_NUMBER2) {
            switch (rx_frame.data.u8[2]) {
                case TARGET: {
                    _foc_ch2->target = unpackFloatBit3To7(); break;
                }
                case Q_KP: {
                    _foc_ch2->iqPID.kp = unpackFloatBit3To7(); break;
                }
                case Q_KI: {
                    _foc_ch2->iqPID.ki = unpackFloatBit3To7(); break;
                }
                case Q_KD: {
                    _foc_ch2->iqPID.kd = unpackFloatBit3To7(); break;
                }
                case Q_OUTPUT_CONSTRAINT: {
                    _foc_ch2->iqPID.output_constrain = unpackFloatBit3To7(); break;
                }
                case Q_ERROR_SUM_CONSTRAINT: {
                    _foc_ch2->iqPID.error_sum_constrain = unpackFloatBit3To7(); break;
                }
                case D_KP: {
                    _foc_ch2->idPID.kp = unpackFloatBit3To7(); break;
                }
                case D_KI: {
                    _foc_ch2->idPID.ki = unpackFloatBit3To7(); break;
                }
                case D_KD: {
                    _foc_ch2->idPID.kd = unpackFloatBit3To7(); break;
                }
                case D_OUTPUT_CONSTRAINT: {
                    _foc_ch2->idPID.output_constrain = unpackFloatBit3To7(); break;
                }
                case D_ERROR_SUM_CONSTRAINT: {
                    _foc_ch2->idPID.error_sum_constrain = unpackFloatBit3To7(); break;
                }
                case VELOCITY_KP: {
                    _foc_ch2->velocityPID.kp = unpackFloatBit3To7(); break;
                }
                case VELOCITY_KI: {
                    _foc_ch2->velocityPID.ki = unpackFloatBit3To7(); break;
                }
                case VELOCITY_KD: {
                    _foc_ch2->velocityPID.kd = unpackFloatBit3To7(); break;
                }
                case VELOCITY_OUTPUT_CONSTRAINT: {
                    _foc_ch2->velocityPID.output_constrain = unpackFloatBit3To7(); break;
                }
                case VELOCITY_ERROR_SUM_CONSTRAINT: {
                    _foc_ch2->velocityPID.error_sum_constrain = unpackFloatBit3To7(); break;
                }
                case POSITION_KP: {
                    _foc_ch2->positionPID.kp = unpackFloatBit3To7(); break;
                }
                case POSITION_KI: {
                    _foc_ch2->positionPID.ki = unpackFloatBit3To7(); break;
                }
                case POSITION_KD: {
                    _foc_ch2->positionPID.kd = unpackFloatBit3To7(); break;
                }
                case POSITION_OUTPUT_CONSTRAINT: {
                    _foc_ch2->positionPID.output_constrain = unpackFloatBit3To7(); break;
                }
                case POSITION_ERROR_SUM_CONSTRAINT: {
                    _foc_ch2->positionPID.error_sum_constrain = unpackFloatBit3To7(); break;
                }
                default: {
                    return CAN_ERROR::UnMatchedCommand;
                }
            }
        }
    }

    if(rx_frame.data.u8[0] == READ) {

    }

    return CAN_ERROR::NoError;
}

#ifdef ENABLE_2_CHANNEL_PWM
void CANCommunication::registerDriver(FOC *foc_ch1, FOC *foc_ch2) {
    _foc_ch1 = foc_ch1;
    _foc_ch2 = foc_ch2;
}

void CANCommunication::configFilter() {
    CAN_filter_t p_filter;
    p_filter.FM = Single_Mode;
#if MSG_ID == 0x001
    p_filter.ACR0 = 0;
    p_filter.ACR1 = 0x20;
    p_filter.ACR2 = 0;
    p_filter.ACR3 = 0;
    p_filter.AMR0 = 0;
    p_filter.AMR1 = 0x20;
    p_filter.AMR2 = 0xFF;
    p_filter.AMR3 = 0xFF;
#elif MSG_ID == 0x002
    p_filter.ACR0 = 0;
    p_filter.ACR1 = 0x40;
    p_filter.ACR2 = 0;
    p_filter.ACR3 = 0;
    p_filter.AMR0 = 0;
    p_filter.AMR1 = 0x80;
    p_filter.AMR2 = 0xFF;
    p_filter.AMR3 = 0xFF;
#elif MSG_ID == 0x04
    p_filter.ACR0 = 0;
    p_filter.ACR1 = 0x80;
    p_filter.ACR2 = 0;
    p_filter.ACR3 = 0;
    p_filter.AMR0 = 0;
    p_filter.AMR1 = 0x80;
    p_filter.AMR2 = 0xFF;
    p_filter.AMR3 = 0xFF;
#endif
    ESP32Can.CANConfigFilter(&p_filter);
}

#else
void CANCommunication::registerDriver(FOC *foc) {
    _foc_ch1 = foc;
}
#endif
