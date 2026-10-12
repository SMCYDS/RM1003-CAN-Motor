//
// Created by Thomas on 2026/10/11.
/*
 * 先调整内环，再调外环
 */
#include "pid.hpp"
#include<algorithm>


void PID::reset()
{
    err_sum_ = 0.0f, running_ = false, last_d_ = 0.0f, last_err_ = 0.0f;
}; //清理历史状态，电机重连reset

float PID::calc(float ref,float fdb) //ref 目标值 fdb 实际值
{
    //定义基础变量
    float err = ref - fdb, output = 0;
    //I项累加和clamp
    err_sum_ += err;
    err_sum_ = std::clamp(err_sum_, -i_max_, i_max_);
    //运算d值
    if (running_)
    {
        dn_ = kd_ * (err - last_err_); //计算差值
        dn_ = d_filter_k_ * dn_ + (1 - d_filter_k_) * last_d_; //一阶低通
    }
    else
    {
        dn_ = 0.0f;
        running_ = true;
    }
    //合成输出
    pn_ = kp_*err;in_ = err_sum_ * ki_;
    output = pn_ + in_ + dn_;
    //记录历史数据
    last_d_ = dn_;
    last_err_ = err;
    //进行clamp
    output = std::clamp(output, -out_max_, out_max_);
    return output;
};

