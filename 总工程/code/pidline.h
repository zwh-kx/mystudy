#ifndef __pidline_H__
#define __pidline_H__



typedef struct {
    float Kp;          
    float Ki;         
    float Kd;          
    float setpoint;   
    float integral;    
    float last_error;  
    float out_max;     
    float out_min;     
} PID_Controller_line;

extern PID_Controller_line pidline;

void PID_Init_line(PID_Controller_line* pid, float Kp, float Ki, float Kd, float setpoint);

float PID_Compute_line(PID_Controller_line* pid, float feedback);

void PID_SetOutputLimits_line(PID_Controller_line* pid, float min, float max);

#endif