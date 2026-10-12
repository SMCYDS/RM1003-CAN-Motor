//
// Created by Thomas on 2026/10/11.
//

#include "motor.hpp"
#include "pid.hpp"
#include "can_manager.h"
#include "callback.h"
#include "can.h"
#include <cmath>

void Motor::canTransmit()
{
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
        HAL_CAN_AddTxMessage(&hcan1, &tx_header,
        getTxData(),
        &can_tx_mailbox);
    }
}

void Motor::feedbackUART()
{
    char callback_msg[64];

    int len = snprintf(callback_msg, sizeof(callback_msg),
                       "[a:%i,ta:%.0f; spd:%.0f amp:%.0f tmp:%.0f]\r\n",
                       uint8_t(ecdAngle_*360/8192),angle_,speedRpm_,currentA_,tempC_);

    // 发送数据到 USART1
    if (len > 0 && len < sizeof(callback_msg)) {
        HAL_UART_Transmit(&huart1, (uint8_t *)callback_msg, strlen(callback_msg), 1000);
    }
}

void Motor::canRxMsgCallback(const uint8_t rx_data[8])
{
    //解码参数
    if (esc_ == MotorMode::C620)
    {
        ecdAngle_ = static_cast<int16_t>(rx_data[1] | rx_data[0]<<8);
        speedRpm_ = static_cast<int16_t>(rx_data[3] | rx_data[2]<<8);
        currentA_ = static_cast<int16_t>(rx_data[5] | rx_data[4]<<8)*20/16384.0f;
        tempC_ = static_cast<uint8_t>(rx_data[6]);
        errorCode = static_cast<uint8_t>(rx_data[7]);
        // 2. 角度换算 ×360/8192
        // 3. 增量累加 + 过零点环绕修正
        // 4. 折到输出轴 (÷ 减速比)
        //rpm=10k时，理论一帧转0.16圈，所以不会出现一帧转超过8192个编码的情

        lastEcd_ = ecdAngle_;
    }else if (esc_ == MotorMode::C610){
        ecdAngle_ = static_cast<int16_t>(rx_data[1] | rx_data[0]<<8);
        speedRpm_ = static_cast<int16_t>(rx_data[3] | rx_data[2]<<8);
        actualTorque_ = static_cast<int16_t>(rx_data[5] | rx_data[4]<<8);
    }

    //角度编码
    int diff = ecdAngle_ - lastEcd_;
    if (diff > 4096)       diff -= 8192;
    else if (diff < -4096) diff += 8192;
    angle_ += static_cast<float>(diff) * 360.0f / 8192.0f / ratio_;

    lastEcd_ = ecdAngle_;

    if (enablePID_)//PID控制模式
    {
        if (motor.getMode()!=MMode::OFF)
        {
            const float ffw_speed_=Kgp_ * std::sin(angle_ * 3.1415926/180),ffw_intensity_=Kgs_ * std::sin(angle_ * 3.1415926/180);//todo

            PIDTargetSpeed_ = (motor.getMode()==MMode::PID?ppid_.calc(targetAngle_,angle_):0)+ ffw_speed_;
            PIDAmpress_ = (motor.getMode()==MMode::PID?spid_.calc(PIDTargetSpeed_, speedRpm_ * (2.0f * 3.1415926f / 60.0f)):0)+ ffw_intensity_;
            setTxCurrent(PIDAmpress_);
            canTransmit();
        }else
        {
            setTxCurrent(0.0f);
            canTransmit();
        }
    }else{//非PID控制模式，遗留代码
        motor.getMode()!=MMode::OFF?setTxCurrent(motor.defaultWorkAmp):setTxCurrent(0.0f);
        canTransmit();
    }

}

uint8_t* Motor::getTxData()
{

    auto amp=static_cast<int16_t>((currentSet)*16384.0f/20);
    for(int i=0;i<8;i++){
        if(i==id_*2-2){
            txData_[i]=(amp>>8) & 0xFF;
        }else if(i==id_*2-1){
            txData_[i]=(amp)&0xFF;
        }else txData_[i]=0x00;
    }


    return txData_;
};