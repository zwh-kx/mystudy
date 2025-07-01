#ifndef __PID_H__
#define __PID_H__

typedef struct {
    float Kp;          
    float Ki;         
    float Kd;          
    float setpoint;   
    float integral;    
    float last_error;  
    float out_max;     
    float out_min;     
} PID_Controller;

void PID_Init(PID_Controller* pid, float Kp, float Ki, float Kd, float setpoint);

float PID_Compute(PID_Controller* pid, float feedback);

void PID_SetOutputLimits(PID_Controller* pid, float min, float max);

#endif