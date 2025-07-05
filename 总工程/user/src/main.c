#include "zf_common_headfile.h"
#include "pid.h"
#include "isr.h"
#include "zf_driver_pit.h"
#include "auto_menu.h"
#include "zf_driver_encoder.h"
#include "otsu.h"
#include "pidline.h"
#include "findline.h"


#define DIR_R               (A0 )
#define PWM_R               (TIM5_PWM_CH2_A1)

#define DIR_L               (A2 )
#define PWM_L               (TIM5_PWM_CH4_A3)

#define ENCODER_1                   (TIM3_ENCODER)
#define ENCODER_1_A                 (TIM3_ENCODER_CH1_B4)
#define ENCODER_1_B                 (TIM3_ENCODER_CH2_B5)

#define ENCODER_2                   (TIM4_ENCODER)
#define ENCODER_2_A                 (TIM4_ENCODER_CH1_B6)
#define ENCODER_2_B                 (TIM4_ENCODER_CH2_B7)

#define PIT                         (TIM6_PIT )                                 // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用
#define start_column                    0	
#define end_column                      187

float putinL;
float putinR;
float putoutL;
float putoutR;
int16 encoder_data_L = 0;
int16 encoder_data_R = 0;
uint8    W;
uint8    H;
float putoutline;
float putinline;

uint8 White_Column[MT9V03X_W];
uint8 Longest_White_Column_Left[2] ={0};
uint8 Longest_White_Column_Right[2] ={0};
uint8 Search_Stop_Line;
uint8 left_border;
uint8 right_border;
uint8 Left_Line [MT9V03X_H];
uint8 Right_Line [MT9V03X_H];
uint8 image_deal[MT9V03X_H][MT9V03X_W];
uint8 Left_Lost_Flag[MT9V03X_H];
uint8 Right_Lost_Flag[MT9V03X_H];


int main (void)
{
		clock_init(SYSTEM_CLOCK_120M);                                              // 初始化芯片时钟 工作频率为 120MHz
    debug_init();                                                               // 初始化默认 debug uart
		menu_init();                                                                //菜单初始化
		

		
		gpio_init(DIR_L, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_L, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(DIR_R, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_R, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0
		
    encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);                     // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B);                     // 初始化编码器模块与引脚 正交解码编码器模式
		pit_ms_init(PIT, 200);

		PID_Init(&pid1,1.0f,0,0,100.0f);
		PID_Init(&pid2,1.0f,0,0,100.0f);
		PID_SetOutputLimits(&pid1,-500.0f,500.0f);             // 初始化pid参数
		PID_SetOutputLimits(&pid2,-500.0f,500.0f);
		pit_ms_init(TIM8_PIT, 200);
		
		
    while(1)
    {
        show_process(NULL);                                                     //菜单启动
				pwm_set_duty(PWM_L,putoutL*10);
				pwm_set_duty(PWM_R,putoutR*10);
				if(mt9v03x_finish_flag)
				{
					image_threshold=otsuThreshold(mt9v03x_image[0],MT9V03X_W, MT9V03X_H);
					for(W=0;W<MT9V03X_W;W++)
					{
						for(H=0;H<MT9V03X_H;H++)
						{
							if(mt9v03x_image[W][H]>=image_threshold)
							{
								image_deal[W][H]=1;
							}
							else
							{
								image_deal[W][H]=0;
							}
						}
					} 
					mt9v03x_finish_flag = 0;
				}
				
							//统计白线长度
		for (W =start_column; W<=end_column; W++)
		{
				for (H = MT9V03X_H - 1; H >= 0; H--)
				{
						if(image_deal[H][W] == 0)
								break;
						else
								White_Column[W]++;
				}
		}
		//从左到右找左边最长白列
    for(H=start_column;H<=end_column;H++)
    {
        if (Longest_White_Column_Left[0] < White_Column[H])//找最长的那一列
        {
            Longest_White_Column_Left[0] = White_Column[H];//【0】是白列长度
            Longest_White_Column_Left[1] = H;              //【1】是下标，第j列
        }
    }
    //从右到左找右左边最长白列
    for(H=end_column;H>=start_column;H--)//从右往左，注意条件，找到左边最长白列位置就可以停了
    {
        if (Longest_White_Column_Right[0] < White_Column[H])//找最长的那一列
        {
            Longest_White_Column_Right[0] = White_Column[H];//【0】是白列长度
            Longest_White_Column_Right[1] = H;              //【1】是下标，第j列
        }
    }
		Search_Stop_Line = Longest_White_Column_Left[0];//搜索截止行选取左或者右区别不大，他们两个理论上是一样的
		   //相关数据使用前清零
    Longest_White_Column_Left[0] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Left[1] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Right[0] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Right[1] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    for (H = MT9V03X_H - 1; H >=MT9V03X_H-Search_Stop_Line; H--)
    {//从最下面一行，访问到有效视野行
        for (W = Longest_White_Column_Right[1]; W <= MT9V03X_W - 1 - 2; W++)
        {
            if (image_deal[H][W] ==1 && image_deal[H][W + 1] == 0 && image_deal[H][W + 2] == 0)//白黑黑，找到右边界
            {
                right_border = W;
                Right_Lost_Flag[H] = 0; //右丢线数组，丢线置1，不丢线置0
                break;
            }
            else if(W>=MT9V03X_W-1-2)//没找到右边界，把屏幕最右赋值给右边界
            {
                right_border = W;
                Right_Lost_Flag[H] = 1; //右丢线数组，丢线置1，不丢线置0
                break;
            }
        }
        for (W = Longest_White_Column_Left[1]; W >= 0 + 2; W--)//往左边扫描
        {//从最下面一行，访问到我的有效是视野行
            if (image_deal[H][W] ==1 && image_deal[H][W - 1] == 0 && image_deal[H][W - 2] == 0)//黑黑白认为到达左边界
            {
                left_border = W;
                Left_Lost_Flag[H] = 0; //左丢线数组，丢线置1，不丢线置0
                break;
            }
            else if(W<=0+2)
            {
                left_border = W;//找到头都没找到边，就把屏幕最左右当做边界
                Left_Lost_Flag[H] = 1; //左丢线数组，丢线置1，不丢线置0
                break;
            }
        }
        Left_Line [H] = left_border;       //左边线线数组
        Right_Line[H] = right_border;      //右边线线数组
    }
			
			
			
			
			
			
		}
}
				
				




void pit_handler (void)
{
    encoder_data_R = encoder_get_count(ENCODER_1);                              // 获取编码器计数
    encoder_data_L = encoder_get_count(ENCODER_2);                              // 获取编码器计数
		
		//printf("OUTL counter \t%f .\r\n", putoutL);                 
		//printf("OUTR counter \t%f .\r\n", putoutR);                 
		//printf("INL counter \t%f .\r\n", putinL);                 
		//printf("INR counter \t%f .\r\n", putinR);
		//printf("ENCODEL counter \t%d .\r\n", encoder_data_L);                 
		//printf("ENCODER counter \t%d .\r\n", encoder_data_R);
		
    encoder_clear_count(ENCODER_1);                                             // 清空编码器计数
		encoder_clear_count(ENCODER_2);                                             // 清空编码器计数
}
 
void pit_handler1 (void)
{
    putoutL = PID_Compute(&pid1, putinL);
		putoutR = PID_Compute(&pid2, putinR);
		putoutline = PID_Compute_line(&pidline, putinline);
}
