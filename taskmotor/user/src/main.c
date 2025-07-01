#include "zf_common_headfile.h"
#include "pid.h"
#include "isr.h"
#include "zf_driver_pit.h"


// **************************** 代码区域 ****************************
#define CHANNEL_NUMBER          (4)

#define PWM_CH1                 (TIM5_PWM_CH1_A0)
#define PWM_CH2                 (TIM5_PWM_CH2_A1)
#define PWM_CH3                 (TIM5_PWM_CH3_A2)
#define PWM_CH4                 (TIM5_PWM_CH4_A3)

PID_Controller pid;
float b;
int16 duty = 0;
uint8 channel_index = 0;
pwm_channel_enum channel_list[CHANNEL_NUMBER] = {PWM_CH1, PWM_CH2, PWM_CH3, PWM_CH4};


int main (void)
{
    clock_init(SYSTEM_CLOCK_120M);                                              // 初始化芯片时钟 工作频率为 120MHz
    debug_init();                                                               // 初始化默认 debug uart
		
		pit_ms_init(TIM6_PIT, 200);
		pit_enable(TIM6_PIT);                                                       //初始化用于pid的中断

		PID_Init(&pid,1.0f,0.1f,0.05f,100.0f);
		PID_SetOutputLimits(&pid,-500.0f,500.0f);             // 初始化pid参数

    // 此处编写用户代码 例如外设初始化代码等
    pwm_init(PWM_CH1, 17000, 0);                                                // 初始化 PWM 通道 频率 17KHz 初始占空比 0%
    pwm_init(PWM_CH2, 17000, 0);                                                // 初始化 PWM 通道 频率 17KHz 初始占空比 0%
    pwm_init(PWM_CH3, 17000, 0);                                                // 初始化 PWM 通道 频率 17KHz 初始占空比 0%
    pwm_init(PWM_CH4, 17000, 0);                                                // 初始化 PWM 通道 频率 17KHz 初始占空比 0%
    // 此处编写用户代码 例如外设初始化代码等

    while(1)
    {
        // 此处编写需要循环执行的代码
        // for(channel_index = 0; CHANNEL_NUMBER > channel_index; channel_index ++)
        // {
        //     for(duty = 0; PWM_DUTY_MAX / 2 >= duty; duty ++)                    // 输出占空比递增到 50%
        //     {
        //         pwm_set_duty(channel_list[channel_index], duty);                // 更新对应通道占空比
        //         system_delay_us(100);
        //     }
        //     for(duty = PWM_DUTY_MAX / 2; 0 <= duty; duty --)                    // 输出占空比递减到 0%
        //     {
        //         pwm_set_duty(channel_list[channel_index], duty);                // 更新对应通道占空比
        //         system_delay_us(100);
        //     }
        // }

        // for(duty = 0; duty <= PWM_DUTY_MAX/4; duty ++)                           // 输出占空比递增到 25%
        // {
        //     pwm_set_duty(PWM_CH1, duty);                                        // 更新对应通道占空比
        //     pwm_set_duty(PWM_CH2, duty);                                        // 更新对应通道占空比
        //     pwm_set_duty(PWM_CH3, duty);                                        // 更新对应通道占空比
        //     pwm_set_duty(PWM_CH4, duty);                                        // 更新对应通道占空比
        //     system_delay_us(100);
        // }

        pwm_set_duty(PWM_CH1, PWM_DUTY_MAX*0.2);                                                // 更新对应通道占空比
        // 此处编写需要循环执行的代码
    }
}
 
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     TIM6 的定时器中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(TIM6_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void TIM6_IRQHandler (void)
{
    // 此处编写用户代码
		PID_Compute(&pid,b);
    // 此处编写用户代码
    TIM6->SR &= ~TIM6->SR;                                                      // 清空中断状态
}

