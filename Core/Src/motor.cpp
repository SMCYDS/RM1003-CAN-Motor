//
// Created by Thomas on 2026/10/11.
//

#include "motor.hpp"

void Motor::feedbackUART()
{
    char callback_msg[64];

    int len = snprintf(callback_msg, sizeof(callback_msg),
                       "[a:%i,ta:%.0f; spd:%.0f amp:%.0f tmp:%.0f]\r\n",
                       uint8_t(ecd_angle_*360/8192),angle_,speedRpm_,currentA_,tempC_);

    // 发送数据到 USART1
    if (len > 0 && len < sizeof(callback_msg)) {
        HAL_UART_Transmit(&huart1, (uint8_t *)callback_msg, strlen(callback_msg), 1000);
    }
}

void Motor::canRxMsgCallback(const uint8_t rx_data[8])
{
    ecd_angle_ = static_cast<int16_t>(rx_data[1] | rx_data[0]<<8);
    speedRpm_ = static_cast<int16_t>(rx_data[3] | rx_data[2]<<8);
    currentA_ = static_cast<int16_t>(rx_data[5] | rx_data[4]<<8)*20/16384.0f;
    tempC_ = static_cast<uint8_t>(rx_data[6]);
    errorCode = static_cast<uint8_t>(rx_data[7]);
    // 2. 角度换算 ×360/8192
    // 3. 增量累加 + 过零点环绕修正
    // 4. 折到输出轴 (÷ 减速比)
    //rpm=10k时，理论一帧转0.16圈，所以不会出现一帧转超过8192个编码的情况
    float nowang=ecd_angle_*360/8192,lastang=last_ecd_*360/8192;
    //目前电流输入都是正的，所以这么写（
    if (lastang > nowang)lastang-=360;
    angle_ += (nowang-lastang)/ratio_;

    last_ecd_ = ecd_angle_;

}

uint8_t* Motor::getTxData(int id)
{

    auto amp=static_cast<int16_t>((currentSet)*16384.0f/20);
    for(int i=0;i<8;i++){
        if(i==id*2-2){
            tx_data_[i]=(amp>>8) & 0xFF;
        }else if(i==id*2-1){
            tx_data_[i]=(amp)&0xFF;
        }else tx_data_[i]=0x00;
    }


    return tx_data_;
};