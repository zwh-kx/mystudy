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


#define DIR_R               (A2 )
#define PWM_R               (TIM5_PWM_CH4_A3)
#define DIR_L               (A0 )
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

int16 encoder_data_L = 0;
int16 encoder_data_R = 0;
uint8    W;                                                                      //宽
uint8    H;                                                                      //高

uint8    FLAG=0;                                                                 //停车标志位

uint8    FLAG2;																																	 //圆环状态
uint8    FLAG3;
uint8    FLAG4;


int32 sum;                                                                       //用于编码器计数
float sumx;                                                                      //用于将x轴角度积分
float x;                                                                         //用于接收陀螺仪x轴实际数据


uint8 COUNT;
uint8 COUNT1;
uint8 count;

float putoutline;                                                              //pid输出
float putinline;                                                               //pid输入

int16 otsuflag;                                                                //大津法标志位，用于大津法分频

uint8 White_Column[MT9V03X_W];

uint8 Longest_White_Column_Left[2] ={0};                                      //0是长度，1是下标
uint8 Longest_White_Column_Right[2] ={0};

uint8 left_border;
uint8 right_border;

uint8 Mid_Line [MT9V03X_H];                                                   //中线数组
uint8 image_deal[MT9V03X_H][MT9V03X_W];

uint8 Left_Lost_Flag[MT9V03X_H];																							//左右边界丢线标志位
uint8 Right_Lost_Flag[MT9V03X_H];


uint8 down_search_start;                                                       //搜索截至行

extern uint16 menuflag1;																											 //以下三个都是菜单标志位
extern uint16 menuflag2;
extern uint16 menuflag3;

extern float Kr;                                                               //陀螺仪用于转向环的参数

int main (void)
{
		otsuflag=1;
		COUNT=0;
		count=1;
		FLAG2=0;
		
		menuflag1=0;                                                                //用于控制发车
		menuflag2=1;                                                                //用于改前瞻
		menuflag3=0;                                                                //用于视野
		
		clock_init(SYSTEM_CLOCK_120M);                                              // 初始化芯片时钟 工作频率为 120MHz
    debug_init();                                                               // 初始化默认 debug uart
		
		menu_init();                                                                //菜单初始化
		
		mt9v03x_init();                                                             //摄像头初始化
		mpu6050_init();                                                             //陀螺仪初始化
		
		gpio_init(DIR_L, GPO, GPIO_LOW, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_L, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(DIR_R, GPO, GPIO_LOW, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_R, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0
		
    encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);                     // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_dir_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B);                     // 初始化编码器模块与引脚 正交解码编码器模式
		pit_ms_init(TIM6_PIT, 30);                                                        // 编码器中断

		PID_Init(&pid1,1.0f,0.9f,0.0f,1400.0f);                                    //速度环
		PID_SetOutputLimits(&pid1,-5000.0f,5000.0f);                                
		PID_Init_line(&pidline_turn,67.0f,0.0f,10.4f,101.0f);                      //弯道转向环
		PID_Init_line(&pidline_line,46.0f,0.0f,0.04f,101.0f);                      //直道转向环
		Kr=0.0;                                                                    //陀螺仪控制项
		PID_SetOutputLimits_line(&pidline_turn,-4000.0f,4000.0f);
		pit_ms_init(TIM8_PIT, 30);                                                    //pid中断
		
		
    while(1)
    {
		pid1.setpoint=1300.0;
		if(menuflag1==0)
		{
			show_process(NULL);				//菜单启动
		}
				
		if(menuflag1==1)
		{
				if(Search_Stop_Line>=108)
				{
					pid1.setpoint=pid1.setpoint*1.3;
				}
				if(Search_Stop_Line<=94)
				{
					pid1.setpoint=pid1.setpoint*0.9;
				}
				if(putoutL/2+turnL>0)
				{
				gpio_set_level(DIR_L,GPIO_HIGH);
				pwm_set_duty(PWM_L,putoutL/2+turnL);
				}
				if(putoutL/2+turnL<=0)
				{
				gpio_set_level(DIR_L,GPIO_LOW);
				pwm_set_duty(PWM_L,putoutL/2+turnL);
				}
				if(putoutL/2+turnR>0)
				{
				gpio_set_level(DIR_R,GPIO_HIGH);
				pwm_set_duty(PWM_R,putoutL/2+turnR);
				}
				if(putoutL/2+turnR<=0)
				{
				gpio_set_level(DIR_R,GPIO_LOW);
				pwm_set_duty(PWM_R,putoutL/2+turnR);
				}
		}
				
				//ips200_show_float(0,180,putoutline,5,3);                                 
				//ips200_show_float(0,200,putoutL,5,3);
				//ips200_show_float(0,220,putoutR,5,3);
				//ips200_show_float(0,240,turnL,5,3);
				//ips200_show_float(0,260,turnR,5,3);
				//ips200_show_float(0,280,encoder_data_L,5,3);
				//ips200_show_float(0,300,encoder_data_R,5,3);
				//ips200_show_int (0, 160,continuity_change_flag_R,3);
				//ips200_show_int (0, 180,continuity_change_flag_L,3);
				//ips200_show_int (0, 200,monotonicity_change_line,3);
				//ips200_show_int (0, 220,right_down_line,3);
				//ips200_show_int (0, 220,FLAG3,3);
				//ips200_show_int (0, 240,COUNT,3);
				//ips200_show_float(0,300,sumx,4,2);
				//ips200_show_int (0, 260,COUNT1,3);
				//ips200_show_int (0, 200,FLAG,3);
				//ips200_show_int (0, 260,Search_Stop_Line,3);
				
				if(mt9v03x_finish_flag)
				{
					if(count==1)
					{
						image_threshold=otsuThreshold(mt9v03x_image[0],MT9V03X_W, MT9V03X_H);
					}
					count=-count;
				if(menuflag3==1)
				{
						ips200_show_gray_image          (0, 0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, image_threshold);
						menuflag1=2;
				}
					//ips200_displayimage03x(mt9v03x_image[0], 188, 120);
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
				
		//统计白线长度
		for (W =1; W<=187; W++)
		{
				for (H = MT9V03X_H - 1; H >= 0; H--)
				{
						if(image_deal[H][W] == 0)
						{break;}
						else
						{White_Column[W]++;}
				}
		}
		//从左到右找左边最长白列
    for(W=1;W<=188;W++)
    {
        if (Longest_White_Column_Left[0] < White_Column[W])//找最长的那一列
        {
            Longest_White_Column_Left[0] = White_Column[W];//【0】是白列长度
            Longest_White_Column_Left[1] = W;              //【1】是下标，第j列
        }
    }
    //从右到左找右左边最长白列
    for(W=188;W>=1;W--)//从右往左，注意条件，找到左边最长白列位置就可以停了
    {
        if (Longest_White_Column_Right[0] < White_Column[W])//找最长的那一列
        {
            Longest_White_Column_Right[0] = White_Column[W];//【0】是白列长度
            Longest_White_Column_Right[1] = W;              //【1】是下标，第j列
        }
    }
		Search_Stop_Line = Longest_White_Column_Right[0];//搜索截止行选取左或者右区别不大，他们两个理论上是一样的
    
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
								Right_Lost_Time++;
                break;
            }
        }
				Right_Line[H] = right_border;      //右边线线数组
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
								Left_Lost_Time++;
                break;
            }
        }
        Left_Line [H] = left_border;       //左边线线数组
    }
		
		if(Search_Stop_Line<1)                                                     //识别到的最长白列过短视为出界
		{
				FLAG=1;
		}
		if(encoder_data_L>=2700)                      //堵转保护
		{
				FLAG=1;
		}
		if(encoder_data_L<=-2700)                      
		{
				FLAG=1;
		}
		if(encoder_data_R>=2700)                      
		{
				FLAG=1;
		}
		if(encoder_data_R<=-2700)                      
		{
				FLAG=1;
		}

		Cross_Detect();                                                            //十字检测及其处理
		
		if(Find_Left_Down_Point(120,40))
		{
				if(Monotonicity_Change_Left(80,10))
				{
						if(Left_Lost_Time>=20)
						{
							if(Right_Lost_Time<5&&FLAG3==0)
							{
								//FLAG3=1;
							}
						}
				}
		}
		
		if(FLAG2==0)
		{
			if(FLAG3==1)
			{
				if(sum>0&&sum<7000)
				{
						Left_Add_Line(75,35,43,110);
				}
				
				if(sum>8502)
				{
					Right_Add_Line(80,50,150,80);
				}
				if(sum>=12600)
				{
					sum=0;
					FLAG3=2;
				}
			}
			if(FLAG3==2)
			{
				if(sumx>=220)
				{
						Right_Add_Line(3,40,140,90);
						//COUNT++;
						//FLAG4++;
						if(sumx>=295)
						{
								FLAG3=3;
								sumx=0;
						}
				}
			}
			for(H=0;H<=100;H++)
			{
				Mid_Line[H]=(Left_Line [H]+Right_Line[H])/2;
			}
			
		}
		
		//斑马线
		//count=0;
		//t=0;
		//for(i=10;i<=180;i++)
		//{
				
				//if(image_deal[90][i]!=t)
				//{
						//t=image_deal[90][i];
						//count++;
				//}
		//}
		//if(count>=11)
		//{
				//FLAG=1;
		//}
		
		
		//边中线显示
		if(menuflag3==1)
		{
			for(i=0;i<=100;i++)
			{
					ips200_draw_point(Right_Line[i], i, RGB565_RED);
					ips200_draw_point(Left_Line[i], i, RGB565_RED);
					ips200_draw_point(Mid_Line[i], i, RGB565_BLUE);
			}
		}
			
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
		}
}
				
				
void pit_handler (void)
{
    encoder_data_R = -encoder_get_count(ENCODER_1);                              // 获取编码器计数
    encoder_data_L = encoder_get_count(ENCODER_2);
		putinL=encoder_data_L;
		putinR=encoder_data_R;
		
		
		mpu6050_get_gyro();                                                           // 获取陀螺仪数据
		
		if(FLAG3==2)
		{
			x=mpu6050_gyro_transition(mpu6050_gyro_x);
			sumx=sumx+x*0.005+0.02;
		}
		
		if(FLAG3==1)
		{
			sum=sum+encoder_data_R;
		}
		
		//printf("\r\n%f", sumx);
		//printf("OUTL  \t%f .\r\n", putoutL);                 
		//printf("OUTR  \t%f .\r\n", putoutR);                 
		//printf("INL  \t%f .\r\n", putinL);                 
		//printf("INR  \t%f .\r\n", putinR);
		//printf("ENCODEL  \t%d .\r\n", encoder_data_L);                 
		//printf("ENCODER  \t%d .\r\n", encoder_data_R);
		//printf("ENCODER  \t%f.\r\n", putinline)；
		//printf("turnR  \t%f .\r\n", turnR);
		//printf("turnL  \t%f .\r\n", turnL);
		//printf("%d,",encoder_data_R);
		//printf("%d,",encoder_data_L);	
		//printf("%d\n",600);
		//printf("%d\n,",sum);
		//printf("%f,",derivative);
		//printf("%f\n,",putoutline);	

		
    encoder_clear_count(ENCODER_1);                                             // 清空编码器计数
		encoder_clear_count(ENCODER_2);                                             // 清空编码器计数
}
 
void pit_handler1 (void)
{
    if(menuflag2==1)																												//菜单改前瞻
		{
		putinline=(Mid_Line[53]+Mid_Line[54]+Mid_Line[55]+Mid_Line[56])/4;
		}
		if(menuflag2==2)
		{
		putinline=(Mid_Line[60]+Mid_Line[61]+Mid_Line[62]+Mid_Line[63])/4;
		}
		
		if(putinline>=116 && putinline<=86)                                     //转弯直道两套pid
		{
			putoutline = PID_Compute_line(&pidline_turn, putinline);                                          
		}
		if(putinline<116 || putinline>86)
		{
			putoutline = PID_Compute_line(&pidline_turn, putinline);
		}
		
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
		if(mpu6050_gyro_x>0)                                                      //陀螺仪
		{
				turnL=turnL+Kr*mpu6050_gyro_x;
				turnR=turnR-Kr*mpu6050_gyro_x;
		}
		if(mpu6050_gyro_x<0)
		{
				turnL=turnL+Kr*mpu6050_gyro_x;
				turnR=turnR-Kr*mpu6050_gyro_x;
		}
		
		if(menuflag1==1)
		{
				putoutL = PID_Compute(&pid1, putinL+putinR);
		}
		if(FLAG)                                     //停车
		{
				putoutL=0;
				turnL=0;
				turnR=0;
		}
}
