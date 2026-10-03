//
// Created by Thomas on 2026/10/3.
//

#include "../Inc/callback.h"
#include "main.h"
#include "tim.h"
#include "can.h"
#include "can_manager.h"
#include "motor.hpp"

Motor motor(19.2f);
bool motorOn=false;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim == &htim1)
    {
        HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
        motor.feedbackUART();
    }else if (htim==&htim6)
    {
        motorOn?motor.setTxCurrent(motor.defaultWorkAmp):motor.setTxCurrent(0.0f);
        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
            HAL_CAN_AddTxMessage(&hcan1, &tx_header,
            motor.getTxData(),
            &can_tx_mailbox);
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == KEY_Pin)
    {
        motorOn=!motorOn;

    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    uint8_t data[8];
    if (hcan != &hcan1) return;//是否CAN1
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,
          &rx_header, data)!= HAL_OK) return; //是否正常收包
    //是不是该收
    if (rx_header.IDE != CAN_ID_STD || rx_header.RTR != CAN_RTR_DATA || rx_header.DLC != 8 || rx_header.StdId != 0x201) {return;}
    motor.canRxMsgCallback(data);
}
