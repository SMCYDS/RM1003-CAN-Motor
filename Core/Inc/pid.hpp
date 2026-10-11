//
// Created by Thomas on 2026/10/11.
//

#ifndef RM1003_CAN_RTOS_PID_H
#define RM1003_CAN_RTOS_PID_H

#pragma once

class PID {
public:
    explicit PID(float kp, float ki, float kd,
    float i_max=30.0f, float out_max=2.0f,float d_filter_k=0.1f
    ):kp_(kp),ki_(ki),kd_(kd),i_max_(i_max),out_max_(out_max),d_filter_k_(d_filter_k){};

    void reset();
    float calc(float ref,float fdb);
private:
    //基本量
    bool running_=false;
    float last_d_,last_err_;
    float err_sum_;
    float kp_,ki_,kd_,i_max_,out_max_,d_filter_k_;

};

#endif //RM1003_CAN_RTOS_PID_H
