//
// Created by Thomas on 2026/10/3.
//
/* UserCode/can_user.h */

#include "can.h"
#include "motor.hpp"

CAN_RxHeaderTypeDef rx_header;
CAN_TxHeaderTypeDef tx_header = {
    .StdId = 0x200,
    .IDE   = CAN_ID_STD,
    .RTR   = CAN_RTR_DATA,
    .DLC   = 8,
    .TransmitGlobalTime = DISABLE
};

uint32_t can_tx_mailbox;

CAN_FilterTypeDef can_filter_config = {
    .FilterIdHigh = 0x0000,
    .FilterIdLow = 0x0000,
    .FilterMaskIdHigh = 0x0000,
    .FilterMaskIdLow = 0x0000,
    .FilterFIFOAssignment = CAN_RX_FIFO0,
    .FilterBank = 0,
    .FilterMode = CAN_FILTERMODE_IDMASK,
    .FilterScale = CAN_FILTERSCALE_32BIT,
    .FilterActivation = ENABLE,
    .SlaveStartFilterBank = 14,
};