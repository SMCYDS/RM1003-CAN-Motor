#pragma once

#include <string.h>
#include <stdio.h>
#include "usart.h"

enum class ESC : uint8_t {
    C620,
    C610
};


class Motor {
public:
    explicit Motor(float ratio,ESC esc,int id): ratio_(ratio),esc_(esc),id_(id) {}// ratio = 减速比
    //通信
    void feedbackUART();
    void canRxMsgCallback(const uint8_t rx_data[8]);

    // 出口：控制逻辑读取
    float angle() const;           // 输出轴角度 (度)
    float speedRpm() const;        // 转子转速 (RPM)
    float currentAmps() const;     // 转矩电流 (A)
    float temperatureC() const;    // 温度 (℃)
    bool  hasFeedback() const;     // 收到过帧？

    // 命令：控制逻辑调用，内部限幅
    void setTxCurrent(float amperes)
    {
        currentSet = amperes;
    };
    // 命令出口：TIM6 中断取用（HAL 形参非 const）
    uint8_t* getTxData();
    float defaultWorkAmp=0.6f;

private:
    // ---- 配置 ----
    const float ratio_;        // 减速比
    ESC esc_;
    int id_;
    // ---- 这一帧解出来的 ----
    float ecdAngle_ = 0;      // 机械角度 (0~8191)
    float speedRpm_ = 0;       // 转速 (RPM)
    float actualTorque_ = 0;
    float currentA_ = 0;       // 转矩电流 (A)
    float tempC_ = 0;          // 温度 (℃)

    // ---- 连续角度 ----
    float angle_ = 0;          // 输出轴累计角度
    float lastEcd_ = 0;       // 上一帧角度，用来求差
    bool  received_ = false;   // 第一帧特殊处理

    // ---- 待发送 ----
    uint8_t txData_[8] = {};

    //other
    float currentSet = 0;
    float errorCode = 0;



    static constexpr uint16_t kEncoderRange = 8192;
};
