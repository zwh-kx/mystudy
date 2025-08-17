#include "zf_common_headfile.h"
#include "pid.h"
#include "isr.h"
#include "zf_driver_pit.h"
#include "auto_menu.h"
#include "zf_driver_encoder.h"
#include "otsu.h"
#include "pidline.h"
#include "findline.h"
#include "element.h"
#include "whiteline.h"
#include "safe.h"
#include "buzzer.h"

#define DIR_R               (A2 )
#define PWM_R               (TIM5_PWM_CH1_A0)
#define DIR_L               (A3 )
#define PWM_L               (TIM5_PWM_CH2_A1)
#define ENCODER_1                   (TIM3_ENCODER)
#define ENCODER_1_A                 (TIM3_ENCODER_CH1_B4)
#define ENCODER_1_B                 (TIM3_ENCODER_CH2_B5)
#define ENCODER_2                   (TIM4_ENCODER)
#define ENCODER_2_A                 (TIM4_ENCODER_CH1_B6)
#define ENCODER_2_B                 (TIM4_ENCODER_CH2_B7)

float putinL;
float putinR;
float putoutL;
float turnL;
float turnR;
float L;
float R;

float putoutline;                                                              //pid输出
float putinline;                                                               //pid输入
int16 otsuflag;                                                                //大津法标志位，用于大津法分频
extern uint16 menuflag1;																											 //以下2个都是菜单标志位
extern uint16 menuflag3;
extern float Kr;                                                               //陀螺仪用于转向环的参数

int main (void)
{

		Buzzer_Init();
		menuflag1=0;                                                                //用于控制发车
		menuflag3=0;                                                                //用于视野
		clock_init(SYSTEM_CLOCK_120M);                                              // 初始化芯片时钟 工作频率为 120MHz
    debug_init();                                                               // 初始化默认 debug uart
		menu_init();                                                                //菜单初始化
		mt9v03x_init();                                                             //摄像头初始化
		imu660ra_init();                                                             //陀螺仪初始化
		gpio_init(DIR_L, GPO, GPIO_LOW, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_L, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(DIR_R, GPO, GPIO_LOW, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_R, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0

    encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);                     // 初始化编码器
    encoder_dir_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B);                      
		pit_ms_init(TIM6_PIT, 2);                                                   // 编码器中断

		PID_Init(&pid1,40.0f,15.0f,0.0f,550.0f);                                    //速度环
		PID_SetOutputLimits(&pid1,-6000.0f,6000.0f);                                
		PID_Init_line(&pidline_turn,60.0f,0.0f,0.0f,10.0f,101.0f);                      //弯道转向环
		//PID_Init_line(&pidline_line,72.0f,0.0f,0.0f,5.0f,101.0f);                    //直道转向环
		Kr=0.08;                                                                    //陀螺仪控制项
		PID_SetOutputLimits_line(&pidline_turn,-6500.0f,6500.0f);
		pit_ms_init(TIM8_PIT, 10);                                                    //pid中断

    while(1)
    {
			show_process(NULL);				//菜单启动
				if(mt9v03x_finish_flag)
				{
				image_threshold=otsuThreshold(mt9v03x_image[0],MT9V03X_W, MT9V03X_H);
					if(menuflag3==1)
					{
							ips200_show_gray_image          (0, 0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, image_threshold);
							menuflag1=2;
					}
          //根据阈值将图像二值化
					for(H=0;H<MT9V03X_H;H++)
					{
						for(W=0;W<MT9V03X_W;W++)
						{
							if(mt9v03x_image[H][W]>=image_threshold)
							{
								image_deal[H][W]=1;
							}
							else
							{
								image_deal[H][W]=0;
							}
						}
					} 
					mt9v03x_finish_flag = 0;
				}
				for(i=0;i<=188;i++)
				{
					image_deal[0][i]=0;
				}
				findline();                                                                //最长白列
		Cross_Detect();                                                            //十字检测及其处理		
			for(H=0;H<=115;H++)
			{
				Mid_Line[H]=(Left_Line [H]+Right_Line[H])/2;
			}
		//边中线显示
		if(menuflag3==1)
		{
			for(i=0;i<=100;i++)
			{
					ips200_draw_point(Right_Line[i], i, RGB565_RED);
					ips200_draw_point(Left_Line[i], i, RGB565_GREEN);
					ips200_draw_point(Mid_Line[i], i, RGB565_BLUE);
			}
			for(i=0;i<=180;i++)
			ips200_draw_point(i, 50, RGB565_BLUE);
		}
		//保护
		safe();
	  //数组清零
		Longest_White_Column_Left[0] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
		Longest_White_Column_Left[1] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
		Longest_White_Column_Right[0] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
		Longest_White_Column_Right[1] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
			for(H=0;H<=MT9V03X_W;H++)
			{
					White_Column[H]=0;
			}
		Left_Lost_Time=0;
		Right_Lost_Time=0;
		Buzzer();
		ips200_show_float(0,240,putinline,5,3);    
		}
}
				
void pit_handler (void)
{
		imu660ra_get_gyro();                                                           // 获取陀螺仪数据               
		putoutline = PID_Compute_line(&pidline_turn, putinline);         
		if(putinline>=101)                                                           //右转
		{
				turnL=-putoutline;
				turnR=putoutline;
		}
		if(putinline<101)                                                           //左转
		{
				turnL=-putoutline;
				turnR=putoutline;
		}
		if(imu660ra_gyro_z>8000)
		{
				imu660ra_gyro_z=8000;
		}
		if(imu660ra_gyro_z<-8000)
		{
				imu660ra_gyro_z=-8000;
		}
		if(imu660ra_gyro_z>0)                                                      //陀螺仪
		{
				turnL=turnL-Kr*imu660ra_gyro_z;
				turnR=turnR+Kr*imu660ra_gyro_z;
		}
		if(imu660ra_gyro_z<0)
		{
				turnL=turnL-Kr*imu660ra_gyro_z;
				turnR=turnR+Kr*imu660ra_gyro_z;
		}
		pid1.setpoint=550.0;
		if(FLAG)                                     //停车
		{
				putoutL=0;
				turnL=0;
				turnR=0;
		}
		L=putoutL/2+turnL;
		R=putoutL/2+turnR;
		if(L>10000)
		{L=10000;}
		if(R>10000)
		{R=10000;}
		if(menuflag1==1)
		{
				if(Search_Stop_Line>=108)
				{
					if(Left_Lost_Time<=5 && Right_Lost_Time<=5)
					{
						pid1.setpoint=pid1.setpoint*1.0;
					}
				}
				if(putinline<88 || putinline>112)
				{
					pid1.setpoint=pid1.setpoint*0.8;
				}
				if(putoutL/2+turnL>0)
				{
				gpio_set_level(DIR_L,GPIO_HIGH);
				pwm_set_duty(PWM_L,L);
				}
				if(putoutL/2+turnL<=0)
				{
				gpio_set_level(DIR_L,GPIO_LOW);
				pwm_set_duty(PWM_L,L);
				}
				if(putoutL/2+turnR>0)
				{
				gpio_set_level(DIR_R,GPIO_LOW);
				pwm_set_duty(PWM_R,R);
				}
				if(putoutL/2+turnR<=0)
				{
				gpio_set_level(DIR_R,GPIO_HIGH);
				pwm_set_duty(PWM_R,R);
				}
		}
}
void pit_handler1 (void)
{
		//菜单改前瞻
		if(Search_Stop_Line<120-turn_target)
		{
			turn_target=123-Search_Stop_Line;
		}
		putinline=(Mid_Line[turn_target]+Mid_Line[turn_target+1]+Mid_Line[turn_target+2]+Mid_Line[turn_target+3])/4;
		turn_target=55;                                                               //前瞻
		encoder_data_R = -encoder_get_count(ENCODER_1);                              // 获取编码器计数
    encoder_data_L = encoder_get_count(ENCODER_2);
		putinL=encoder_data_L;
		putinR=encoder_data_R;
		if(menuflag1==1)
		{
				putoutL = PID_Compute(&pid1, putinL+putinR);
		}
		encoder_clear_count(ENCODER_1);                                             // 清空编码器计数
		encoder_clear_count(ENCODER_2);                                             // 清空编码器计数
		
}
