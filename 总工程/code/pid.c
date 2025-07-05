#include "pid.h"

PID_Controller pid1;
PID_Controller pid2;

void PID_Init(PID_Controller* pid, float Kp, float Ki, float Kd, float setpoint) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = setpoint;
    pid->integral = 0;
    pid->last_error = 0;
    pid->out_max = 1000.0f;  // 默认值
    pid->out_min = -1000.0f; // 默认值
}

float PID_Compute(PID_Controller* pid, float feedback) {
    // 计算误差
    float error = pid->setpoint - feedback;
    
    // 计算积分项(带抗饱和)
    pid->integral += error;
    if(pid->integral > pid->out_max) pid->integral = pid->out_max;
    if(pid->integral < pid->out_min) pid->integral = pid->out_min;
    
    // 计算微分项
    float derivative = error - pid->last_error;
    pid->last_error = error;
    
    // 计算PID输出
    float output = pid->Kp * error + 
                   pid->Ki * pid->integral + 
                   pid->Kd * derivative;
    
    // 输出限幅
    if(output > pid->out_max) output = pid->out_max;
    if(output < pid->out_min) output = pid->out_min;
    
    return output;
}

void PID_SetOutputLimits(PID_Controller* pid, float min, float max) {
    pid->out_min = min;
    pid->out_max = max;
}