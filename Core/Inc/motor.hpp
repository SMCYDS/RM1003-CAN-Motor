#include <string.h>
#include <stdio.h>
#include "usart.h"

class Motor {
public:
    explicit Motor(float ratio): ratio_(ratio) {}// ratio = 减速比

    void feedbackUART()
    {
        char callback_msg[64];

        int len = snprintf(callback_msg, sizeof(callback_msg),
                           "[ang:%i spd:%.0f amp:%.0f tmp:%.0f]\r\n",
                           uint8_t(ecd_angle_*360/8192),speedRpm_,currentA_,tempC_);

        // 发送数据到 USART1
        if (len > 0 && len < sizeof(callback_msg)) {
            HAL_UART_Transmit(&huart1, (uint8_t *)callback_msg, strlen(callback_msg), 1000);
        }
    }

    // 入口：CAN 接收中断调用
    void canRxMsgCallback(const uint8_t rx_data[8])
    {
        ecd_angle_ = static_cast<int16_t>(rx_data[1] | rx_data[0]<<8);
        speedRpm_ = static_cast<int16_t>(rx_data[3] | rx_data[2]<<8);
        currentA_ = static_cast<int16_t>(rx_data[5] | rx_data[4]<<8);
        tempC_ = static_cast<uint8_t>(rx_data[6]);

    }

    // 出口：控制逻辑读取
    float angle() const;           // 输出轴角度 (度)
    float speedRpm() const;        // 转子转速 (RPM)
    float currentAmps() const;     // 转矩电流 (A)
    float temperatureC() const;    // 温度 (℃)
    bool  hasFeedback() const;     // 收到过帧？

    // 命令：控制逻辑调用，内部限幅
    void setTxCurrent(float amperes)
    {
        currentSet = amperes*1000;
    };
    // 命令出口：TIM6 中断取用（HAL 形参非 const）
    uint8_t* getTxData()
    {
        auto amp=static_cast<uint16_t>(currentSet);
        tx_data_[0]=(amp>>8) & 0xFF;
        tx_data_[1]=(amp)&0xFF;;
        return tx_data_;
    };

private:
    // ---- 配置 ----
    const float ratio_;        // 减速比

    // ---- 这一帧解出来的 ----
    float ecd_angle_ = 0;      // 机械角度 (0~8191)
    float speedRpm_ = 0;       // 转速 (RPM)
    float currentA_ = 0;       // 转矩电流 (A)
    float tempC_ = 0;          // 温度 (℃)
    float currentSet = 0;

    // ---- 连续角度 ----
    float angle_ = 0;          // 输出轴累计角度
    float last_ecd_ = 0;       // 上一帧角度，用来求差
    bool  received_ = false;   // 第一帧特殊处理

    // ---- 待发送 ----
    uint8_t tx_data_[8] = {};

    // ---- TODO: 你来实现 ----
    // 1. 拆 8 字节为字段 (字节序 + 符号)
    // 2. 角度换算 ×360/8192
    // 3. 增量累加 + 过零点环绕修正
    // 4. 折到输出轴 (÷ 减速比)
    static constexpr uint16_t kEncoderRange = 8192;
};
