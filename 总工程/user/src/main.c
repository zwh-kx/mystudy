#include "zf_common_headfile.h"
#include "pid.h"
#include "isr.h"
#include "zf_driver_pit.h"
#include "auto_menu.h"
#include "zf_driver_encoder.h"
#include "otsu.h"
#include "pidline.h"
#include "findline.h"


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

#define PIT                         (TIM6_PIT )                                 // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用


float target;
float putinL;
float putinR;
float putoutL;
float putoutR;
float turnL;
float turnR;
float k=0;
int16 encoder_data_L = 0;
int16 encoder_data_R = 0;
uint8    W;
uint8    H;
uint8    T;
uint8    FLAG;
uint8    FLAG2;
uint8    FLAG3;
uint8    FLAG4;


int32 sum;
float sumx;
float x;


uint8 COUNT;
uint8 COUNT1;
uint8 count;

float putoutline;
float putinline;

int16 p;


int16 otsuflag;

uint8 White_Column[MT9V03X_W];
uint8 Longest_White_Column_Left[2] ={0};
uint8 Longest_White_Column_Right[2] ={0};
uint8 Search_Stop_Line;
uint8 left_border;
uint8 right_border;
uint8 Left_Line [MT9V03X_H];
uint8 Right_Line [MT9V03X_H];
uint8 Mid_Line [MT9V03X_H];
uint8 image_deal[MT9V03X_H][MT9V03X_W];
uint8 Left_Lost_Flag[MT9V03X_H];
uint8 Right_Lost_Flag[MT9V03X_H];
uint8 Left_Lost_Time;
uint8 Right_Lost_Time;
uint8 Right_Down_Find;
uint8 Left_Down_Find;
uint8 Right_Up_Find;
uint8 Left_Up_Find;
uint8 Cross_Flag;
uint8 Left_Up_Find;
uint8 Right_Up_Find;
int i,t;
uint8 Both_Lost_Time;
uint8 down_search_start;
int continuity_change_flag_R;
int continuity_change_flag_L;
int monotonicity_change_line;
int right_down_line;
int left_down_line;

extern uint16 menuflag1;
extern uint16 menuflag2;
extern uint16 menuflag3;

int16 roundflag;

extern float Kr;

//元素识别函数
/*-------------------------------------------------------------------------------------------------------------------
  @brief     找下面的两个拐点，供十字使用
  @param     搜索的范围起点，终点
  @return    修改两个全局变量
             Right_Down_Find=0;
             Left_Down_Find=0;
  Sample     Find_Down_Point(int start,int end)
  @note      运行完之后查看对应的变量，注意，没找到时对应变量将是0
-------------------------------------------------------------------------------------------------------------------*/
void Find_Down_Point(int start,int end)
{
    Right_Down_Find=0;
    Left_Down_Find=0;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-1-5)//下面5行数据不稳定，不能作为边界点来判断，舍弃
        start=MT9V03X_H-1-5;
    if(end<=MT9V03X_H-Search_Stop_Line)
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)
       end=5;
    for(i=start;i>=end;i--)
    {
        if(Left_Down_Find==0&&//只找第一个符合条件的点
           abs(Left_Line[i]-Left_Line[i+1])<=5&&//角点的阈值可以更改
           abs(Left_Line[i+1]-Left_Line[i+2])<=5&&
           abs(Left_Line[i+2]-Left_Line[i+3])<=5&&
              (Left_Line[i]-Left_Line[i-2])>=5&&
              (Left_Line[i]-Left_Line[i-3])>=8&&
              (Left_Line[i]-Left_Line[i-4])>=8)
        {
            Left_Down_Find=i;//获取行数即可
        }
        if(Right_Down_Find==0&&//只找第一个符合条件的点
           abs(Right_Line[i]-Right_Line[i+1])<=5&&//角点的阈值可以更改
           abs(Right_Line[i+1]-Right_Line[i+2])<=5&&
           abs(Right_Line[i+2]-Right_Line[i+3])<=5&&
              (Right_Line[i]-Right_Line[i-2])<=-6&&
              (Right_Line[i]-Right_Line[i-3])<=-9&&
              (Right_Line[i]-Right_Line[i-4])<=-9)
        {
            Right_Down_Find=i;
        }
        if(Left_Down_Find!=0&&Right_Down_Find!=0)//两个找到就退出
        {
            break;
        }
    }
}
 
/*-------------------------------------------------------------------------------------------------------------------
  @brief     找上面的两个拐点，供十字使用
  @param     搜索的范围起点，终点
  @return    修改两个全局变量
             Left_Up_Find=0;
             Right_Up_Find=0;
  Sample     Find_Up_Point(int start,int end)
  @note      运行完之后查看对应的变量，注意，没找到时对应变量将是0
-------------------------------------------------------------------------------------------------------------------*/
void Find_Up_Point(int start,int end)
{
    Left_Up_Find=0;
    Right_Up_Find=0;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(end<=MT9V03X_H-Search_Stop_Line)
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)//及时最长白列非常长，也要舍弃部分点，防止数组越界
        end=5;
    if(start>=MT9V03X_H-1-5)//下面5行数据不稳定，不能作为边界点来判断，舍弃
        start=MT9V03X_H-1-5;
    for(i=start;i>=end;i--)
    {
        if(Left_Up_Find==0&&//只找第一个符合条件的点
           abs(Left_Line[i]-Left_Line[i-1])<=5&&
           abs(Left_Line[i-1]-Left_Line[i-2])<=5&&
           abs(Left_Line[i-2]-Left_Line[i-3])<=5&&
              (Left_Line[i]-Left_Line[i+2])>=6&&
              (Left_Line[i]-Left_Line[i+3])>=9&&
              (Left_Line[i]-Left_Line[i+4])>=9)
        {
            Left_Up_Find=i;//获取行数即可
        }
        if(Right_Up_Find==0&&//只找第一个符合条件的点
           abs(Right_Line[i]-Right_Line[i-1])<=5&&//下面两行位置差不多
           abs(Right_Line[i-1]-Right_Line[i-2])<=5&&
           abs(Right_Line[i-2]-Right_Line[i-3])<=5&&
              (Right_Line[i]-Right_Line[i+2])<=-5&&
              (Right_Line[i]-Right_Line[i+3])<=-8&&
              (Right_Line[i]-Right_Line[i+4])<=-8)
        {
            Right_Up_Find=i;//获取行数即可
        }
        if(Left_Up_Find!=0&&Right_Up_Find!=0)//下面两个找到就出去
        {
            break;
        }
    }
    if(abs(Right_Up_Find-Left_Up_Find)>=30)//纵向撕裂过大，视为误判
    {
        Right_Up_Find=0;
        Left_Up_Find=0;
    }
}


/*-------------------------------------------------------------------------------------------------------------------
  @brief     左右补线
  @param     补线的起点，终点
  @return    null
  Sample     Left_Add_Line(int x1,int y1,int x2,int y2);
  @note      补的直接是边界，点最好是可信度高的,不要乱补
-------------------------------------------------------------------------------------------------------------------*/
void Left_Add_Line(int x1,int y1,int x2,int y2)//左补线,补的是边界
{
    int max,a1,a2;
    int hx;
    if(x1>=MT9V03X_W-1)//起始点位置校正，排除数组越界的可能
       x1=MT9V03X_W-1;
    else if(x1<=0)
        x1=0;
     if(y1>=MT9V03X_H-1)
        y1=MT9V03X_H-1;
     else if(y1<=0)
        y1=0;
     if(x2>=MT9V03X_W-1)
        x2=MT9V03X_W-1;
     else if(x2<=0)
             x2=0;
     if(y2>=MT9V03X_H-1)
        y2=MT9V03X_H-1;
     else if(y2<=0)
             y2=0;
    a1=y1;
    a2=y2;

    if(a1>a2)
    {
        max=a1;
        a1=a2;
        a2=max;
				max=x1;
				x1=x2;
				x2=max;
    }
    for(i=a1;i<=a2;i++)//根据斜率补线即可
    {
        hx=(i-y1)*(x2-x1)/(y2-y1)+x1;
        if(hx>=MT9V03X_W)
            hx=MT9V03X_W;
        else if(hx<=0)
            hx=0;
        Left_Line[i]=hx;
    }
}

void Right_Add_Line(int x3,int y3,int x4,int y4)
{
		int MAX,a3,a4;
    int Hx;
    if(x3>=MT9V03X_W-1)//起始点位置校正，排除数组越界的可能
       x3=MT9V03X_W-1;
    else if(x3<=0)
        x3=0;
     if(y3>=MT9V03X_H-1)
        y3=MT9V03X_H-1;
     else if(y3<=0)
        y3=0;
     if(x4>=MT9V03X_W-1)
        x4=MT9V03X_W-1;
     else if(x4<=0)
             x4=0;
     if(y4>=MT9V03X_H-1)
        y4=MT9V03X_H-1;
     else if(y4<=0)
             y4=0;
    a3=y3;
    a4=y4;
		if(a3>a4)
    {
        MAX=a3;
        a3=a4;
        a4=MAX;
				MAX=x3;
				x3=x4;
				x4=MAX;
    }
		for(i=a3;i<=a4;i++)//根据斜率补线即可
    {
        Hx=(i-y3)*(x4-x3)/(y4-y3)+x3;
        if(Hx>=MT9V03X_W)
            Hx=MT9V03X_W;
        else if(Hx<=0)
            Hx=0;
        Right_Line[i]=Hx;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     左右边界延长
  @param     延长起始行数，延长到某行
  @return    null
  Sample     Lengthen_Right_Boundry(int start,int end)；
  @note      从起始点向上找3个点，算出斜率，向下延长，直至结束点
-------------------------------------------------------------------------------------------------------------------*/
void Lengthen_Right_Boundry(int start,int end)
{
    if(start>=MT9V03X_H-1)//起始点位置校正，排除数组越界的可能
        start=MT9V03X_H-1;
    else if(start<=0)
        start=0;
    if(end>=MT9V03X_H-1)
        end=MT9V03X_H-1;
    else if(end<=0)
        end=0;
 
    if(start<=5 && start <= end)//因为需要在开始点向上找3个点，对于起始点过于靠上，不能做延长，只能直接连线
    {
        Right_Add_Line(Right_Line[start],start,Right_Line[end],end);
    }
    else
    {
        k=(float)(Right_Line[start]-Right_Line[start-4])/5.0;//这里的k是1/斜率
        if(start<=end)
        {
            for(i=start;i<=end;i++)
            {
                Right_Line[i]=(int)(i-start)*k+Right_Line[start];//(x=(y-y1)*k+x1),点斜式变形
                if(Right_Line[i]>=MT9V03X_W-1)
                {
                    Right_Line[i]=MT9V03X_W-1;
                }
                else if(Right_Line[i]<=0)
                {
                    Right_Line[i]=0;
                }
            }
        }
        else
        {
            for(i=end;i<=start;i++)
            {
                Right_Line[i]=(int)(i-start)*k+Right_Line[start];//(x=(y-y1)*k+x1),点斜式变形
                if(Right_Line[i]>=MT9V03X_W-1)
                {
                    Right_Line[i]=MT9V03X_W-1;
                }
                else if(Right_Line[i]<=0)
                {
                    Right_Line[i]=0;
                }
            }
        }
        
    }
}


void Lengthen_Left_Boundry(int start,int end)
{
		if(start>=MT9V03X_H-1)//起始点位置校正，排除数组越界的可能
        start=MT9V03X_H-1;
    else if(start<=0)
        start=0;
    if(end>=MT9V03X_H-1)
        end=MT9V03X_H-1;
    else if(end<=0)
        end=0;
		 if(start<=5 && start <= end)//因为需要在开始点向上找3个点，对于起始点过于靠上，不能做延长，只能直接连线
    {
        Left_Add_Line(Left_Line[start],start,Left_Line[end],end);
    }
		else
    {
        k=(float)(Left_Line[start]-Left_Line[start-4])/5.0;//这里的k是1/斜率
        if(start<=end)
        {
            for(i=start;i<=end;i++)
            {
                Left_Line[i]=(int)(i-start)*k+Left_Line[start];//(x=(y-y1)*k+x1),点斜式变形
                if(Left_Line[i]>=MT9V03X_W-1)
                {
                    Left_Line[i]=MT9V03X_W-1;
                }
                else if(Left_Line[i]<=0)
                {
                    Left_Line[i]=0;
                }
            }
        }
        else
        {
            for(i=end;i<=start;i++)
            {
                Left_Line[i]=(int)(i-start)*k+Left_Line[start];//(x=(y-y1)*k+x1),点斜式变形
                if(Left_Line[i]>=MT9V03X_W-1)
                {
                    Left_Line[i]=MT9V03X_W-1;
                }
                else if(Left_Line[i]<=0)
                {
                    Left_Line[i]=0;
                }
            }
        }
        
    }
}

//十字检查

void Cross_Detect()
{
    int down_search_start=0;//下角点搜索开始行
    Cross_Flag=0;
		Left_Up_Find=0;
		Right_Up_Find=0;
		Both_Lost_Time=Left_Lost_Time>=Right_Lost_Time?Right_Lost_Time:Left_Lost_Time;
        if(Both_Lost_Time>=20)//十字必定有双边丢线，在有双边丢线的情况下再开始找角点
        {
            Find_Up_Point( MT9V03X_H, 30 );
            if(Left_Up_Find==0&&Right_Up_Find==0)//只要没有同时找到两个上点，直接结束
            {
                return;
            }
        }
        if(Left_Up_Find!=0&&Right_Up_Find!=0)//找到两个上点，就认为找到十字了
        {
            Cross_Flag=1;//确定对应标志位，便于各元素互斥掉
            down_search_start=Left_Up_Find>Right_Up_Find?Left_Up_Find:Right_Up_Find;//用两个上拐点坐标靠下者作为下点的搜索上限
            Find_Down_Point(MT9V03X_H-5,down_search_start+30);//在上拐点下2行作为下角点的截止行
            if(Left_Down_Find<=Left_Up_Find)
            {
                Left_Down_Find=0;//下点不可能比上点还靠上
            }
            if(Right_Down_Find<=Right_Up_Find)
            {
                Right_Down_Find=0;//下点不可能比上点还靠上
            }
            if(Left_Down_Find!=0&&Right_Down_Find!=0)
            {//四个点都在，无脑连线，这种情况显然很少
                Left_Add_Line (Left_Line [Left_Up_Find ],Left_Up_Find ,Left_Line [Left_Down_Find ] ,Left_Down_Find);
                Right_Add_Line(Right_Line[Right_Up_Find],Right_Up_Find,Right_Line[Right_Down_Find],Right_Down_Find);
            }
            else if(Left_Down_Find==0&&Right_Down_Find!=0)//11//这里使用的是斜率补线
            {//三个点                                     //01
                Lengthen_Left_Boundry(Left_Up_Find-1,MT9V03X_H-1);
                Right_Add_Line(Right_Line[Right_Up_Find],Right_Up_Find,Right_Line[Right_Down_Find],Right_Down_Find);
            }
            else if(Left_Down_Find!=0&&Right_Down_Find==0)//11
            {//三个点                                      //10
                Left_Add_Line (Left_Line [Left_Up_Find ],Left_Up_Find ,Left_Line [Left_Down_Find ] ,Left_Down_Find);
                Lengthen_Right_Boundry(Right_Up_Find-1,MT9V03X_H-1);
            }
            else if(Left_Down_Find==0&&Right_Down_Find==0)//11
            {//就俩上点                                    //00
                Lengthen_Left_Boundry (Left_Up_Find-1,MT9V03X_H-1);
                Lengthen_Right_Boundry(Right_Up_Find-1,MT9V03X_H-1);
            }
        }
        else
        {
            Cross_Flag=0;
        }
}		


//以下部分用于环岛

/*-------------------------------------------------------------------------------------------------------------------
  @brief     单调性突变检测
  @param     起始点，终止行
  @return    点所在的行数，找不到返回0
  Sample     Find_Right_Up_Point(int start,int end);
  @note      前5后5它最大（最小），那他就是角点
-------------------------------------------------------------------------------------------------------------------*/
int Monotonicity_Change_Right(int start,int end)//单调性改变，返回值是单调性改变点所在的行数
{
		monotonicity_change_line=0;
    if(Right_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有单调性判断的意义
        return monotonicity_change_line;
    if(start>=MT9V03X_H-1-5)//数组越界保护
        start=MT9V03X_H-1-5;
     if(end<=5)
        end=5;
    if(start<=end)
        return monotonicity_change_line;
    for(i=start;i>=end;i--)//会读取前5后5数据，所以前面对输入范围有要求
    {
        if(Right_Line[i]==Right_Line[i+5]&&Right_Line[i]==Right_Line[i-5]&&
        Right_Line[i]==Right_Line[i+4]&&Right_Line[i]==Right_Line[i-4]&&
        Right_Line[i]==Right_Line[i+3]&&Right_Line[i]==Right_Line[i-3]&&
        Right_Line[i]==Right_Line[i+2]&&Right_Line[i]==Right_Line[i-2]&&
        Right_Line[i]==Right_Line[i+1]&&Right_Line[i]==Right_Line[i-1])
        {//一堆数据一样，显然不能作为单调转折点
            continue;
        }
        else if(Right_Line[i] <Right_Line[i+5]&&Right_Line[i] <Right_Line[i-5]&&
        Right_Line[i] <Right_Line[i+4]&&Right_Line[i] <Right_Line[i-4]&&
        Right_Line[i]<=Right_Line[i+3]&&Right_Line[i]<=Right_Line[i-3]&&
        Right_Line[i]<=Right_Line[i+2]&&Right_Line[i]<=Right_Line[i-2]&&
        Right_Line[i]<=Right_Line[i+1]&&Right_Line[i]<=Right_Line[i-1]&&
				Right_Line[i]-Right_Line[i-4]>=-20)
        {//就很暴力，这个数据是在前5，后5中最大的，那就是单调突变点
            monotonicity_change_line=i;
            break;
        }
    }
    return monotonicity_change_line;
}

int Monotonicity_Change_Left(int start,int end)//单调性改变，返回值是单调性改变点所在的行数
{
		monotonicity_change_line=0;
    if(Left_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有单调性判断的意义
        return monotonicity_change_line;
    if(start>=MT9V03X_H-1-5)//数组越界保护
        start=MT9V03X_H-1-5;
     if(end<=5)
        end=5;
    if(start<=end)
        return monotonicity_change_line;
    for(i=start;i>=end;i--)//会读取前5后5数据，所以前面对输入范围有要求
    {
        if(Left_Line[i]==Left_Line[i+5]&&Left_Line[i]==Left_Line[i-5]&&
        Left_Line[i]==Left_Line[i+4]&&Left_Line[i]==Left_Line[i-4]&&
        Left_Line[i]==Left_Line[i+3]&&Left_Line[i]==Left_Line[i-3]&&
        Left_Line[i]==Left_Line[i+2]&&Left_Line[i]==Left_Line[i-2]&&
        Left_Line[i]==Left_Line[i+1]&&Left_Line[i]==Left_Line[i-1])
        {//一堆数据一样，显然不能作为单调转折点
            continue;
        }
        else if(Left_Line[i] >Left_Line[i+5]&&Left_Line[i] >Left_Line[i-5]&&
        Left_Line[i] >Left_Line[i+4]&&Left_Line[i] >Left_Line[i-4]&&
        Left_Line[i]>=Left_Line[i+3]&&Left_Line[i]>=Left_Line[i-3]&&
        Left_Line[i]>=Left_Line[i+2]&&Left_Line[i]>=Left_Line[i-2]&&
        Left_Line[i]>=Left_Line[i+1]&&Left_Line[i]>=Left_Line[i-1]&&
				Left_Line[i]-Left_Line[i-4]<=20)
        {//就很暴力，这个数据是在前5，后5中最大的，那就是单调突变点
            monotonicity_change_line=i;
            break;
        }
    }
    return monotonicity_change_line;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     左右下角点检测
  @param     起始点，终止点
  @return    返回角点所在的行数，找不到返回0
  Sample     Find_Right_Down_Point(int start,int end);
  @note      角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int Find_Right_Down_Point(int start,int end)//找四个角点，返回值是角点所在的行数
{
    right_down_line=0;
    if(Right_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
        return right_down_line;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-1-5)//下面5行数据不稳定，不能作为边界点来判断，舍弃
        start=MT9V03X_H-1-5;
    if(end<=MT9V03X_H-Search_Stop_Line)
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)
       end=5;
    for(i=start;i>=end;i--)
    {
        if(right_down_line==0&&//只找第一个符合条件的点
           abs(Right_Line[i]-Right_Line[i+1])<=5&&//角点的阈值可以更改
           abs(Right_Line[i+1]-Right_Line[i+2])<=5&&  
           abs(Right_Line[i+2]-Right_Line[i+3])<=5&&
              (Right_Line[i]-Right_Line[i-2])<=-5&&
              (Right_Line[i]-Right_Line[i-3])<=-10&&
              (Right_Line[i]-Right_Line[i-4])<=-10)
        {
            right_down_line=i;//获取行数即可
            break;
        }
    }
    return right_down_line;
}

int Find_Left_Down_Point(int start,int end)//找四个角点，返回值是角点所在的行数
{
    left_down_line=0;
    if(Left_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
        return left_down_line;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-1-5)//下面5行数据不稳定，不能作为边界点来判断，舍弃
        start=MT9V03X_H-1-5;
    if(end<=MT9V03X_H-Search_Stop_Line)
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)
       end=5;
    for(i=start;i>=end;i--)
    {
        if(left_down_line==0&&//只找第一个符合条件的点
           abs(Left_Line[i]-Left_Line[i+1])<=5&&//角点的阈值可以更改
           abs(Left_Line[i+1]-Left_Line[i+2])<=5&&  
           abs(Left_Line[i+2]-Left_Line[i+3])<=5&&
              (Left_Line[i]-Left_Line[i-2])>=7&&
              (Left_Line[i]-Left_Line[i-3])>=10&&
              (Left_Line[i]-Left_Line[i-4])>=12)
        {
            left_down_line=i;//获取行数即可
            break;
        }
    }
    return left_down_line;
}


/*-------------------------------------------------------------------------------------------------------------------
  @brief     左右赛道连续性检测
  @param     起始点，终止点
  @return    连续返回0，不连续返回断线出行数
  Sample     continuity_change_flag=Continuity_Change_Right(int start,int end)
  @note      连续性的阈值设置为5，可更改
-------------------------------------------------------------------------------------------------------------------*/
int Continuity_Change_Right(int start,int end)
{
		
    continuity_change_flag_R=0;
    if(Right_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没必要判断了
       return 1;
    if(start>=MT9V03X_H-5)//数组越界保护
        start=MT9V03X_H-5;
    if(end<=5)
       end=5;
    if(start<end)//都是从下往上计算的，反了就互换一下
    {
       t=start;
       start=end;
       end=t;
    }
 
    for(i=start;i>=end;i--)
    {
        if(abs(Right_Line[i]-Right_Line[i-1])>=5)//连续性阈值是5，可更改
       {
            continuity_change_flag_R=i;
            break;
       }
    }
    return continuity_change_flag_R;
}

int Continuity_Change_Left(int start,int end)
{
    continuity_change_flag_L=0;
    if(Left_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没必要判断了
       return 1;
    if(start>=MT9V03X_H-5)//数组越界保护
        start=MT9V03X_H-5;
    if(end<=5)
       end=5;
    if(start<end)//都是从下往上计算的，反了就互换一下
    {
       t=start;
       start=end;
       end=t;
    }
 
    for(i=start;i>=end;i--)
    {
        if(abs(Left_Line[i]-Left_Line[i-1])>=5)//连续性阈值是5，可更改
       {
            continuity_change_flag_L=i;
            break;
       }
    }
    return continuity_change_flag_L;
}


int main (void)
{
		roundflag=1;
		otsuflag=1;
		COUNT=0;
		FLAG2=0;
		FLAG=0;                                                                     //停车标志位
		menuflag1=0;                                                                //用于控制发车
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
    encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B);                     // 初始化编码器模块与引脚 正交解码编码器模式
		pit_ms_init(PIT, 5);                                                        // 编码器中断

		PID_Init(&pid1,40.0f,0.8f,20.0f,170.0f);
		PID_SetOutputLimits(&pid1,-5000.0f,5000.0f);                                 // 初始化pid参数
		PID_Init_line(&pidline,44.0f,0.0f,0.00f,99.0f);
		Kr=0.03;                                                                    //陀螺仪控制项
		PID_SetOutputLimits_line(&pidline,-3000.0f,3000.0f);
		pit_ms_init(TIM8_PIT, 5);                                                    //pid中断
		
		
    while(1)
    {
		//printf("\r\nMPU6050 gyro data: x=%5d, y=%5d, z=%5d\r\n", mpu6050_gyro_x, mpu6050_gyro_y, mpu6050_gyro_z);
		
		if(menuflag1==0)
		{
			show_process(NULL);				//菜单启动
		}
				
		
				
				if(menuflag1==1)
				{
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
				
				if(mt9v03x_finish_flag)
				{
					image_threshold=otsuThreshold(mt9v03x_image[0],MT9V03X_W, MT9V03X_H);
					//ips200_show_gray_image          (0, 0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, image_threshold);
					//ips200_displayimage03x(mt9v03x_image[0], 188, 120);
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
				for (H = MT9V03X_H - 1; H >= 20; H--)
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
		
		//if(Search_Stop_Line<2)                                                     //识别到的最长白列过短视为出界
		//{
		//		FLAG=1;
		//}


		Cross_Detect();                                                            //十字检测及其处理
		
		if(Find_Left_Down_Point(120,40))
		{
				if(Monotonicity_Change_Left(80,10))
				{
						if(Left_Lost_Time>=20)
						{
							if(Right_Lost_Time<5&&FLAG3==0)
							{
								FLAG3=1;
								//COUNT++;
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
		
		
		//count=0;
		//t=0;
		//斑马线
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
			//for(T=0;T<=100;T++)
			//{
					//ips200_draw_point(Right_Line[T], T, RGB565_RED);
					//ips200_draw_point(Left_Line[T], T, RGB565_RED);
					//ips200_draw_point(Mid_Line[T], T, RGB565_BLUE);
			//}
			
			
	  //数组清零
		Longest_White_Column_Left[0] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
		Longest_White_Column_Left[1] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
		Longest_White_Column_Right[0] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
		Longest_White_Column_Right[1] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
			for(T=0;T<=MT9V03X_W;T++)
			{
					White_Column[T]=0;
			}
		Left_Lost_Time=0;
		Right_Lost_Time=0;
			
			
		}
}
				
				
void pit_handler (void)
{
    encoder_data_R = -encoder_get_count(ENCODER_1);                              // 获取编码器计数
    encoder_data_L = encoder_get_count(ENCODER_2);
		
		mpu6050_get_gyro();                                                           // 获取陀螺仪数据
		
		if(FLAG3==2)
		{
			x=mpu6050_gyro_transition(mpu6050_gyro_x);
			sumx=sumx+x*0.005+0.02;
		}
		
		
			// 获取编码器计数
		putinL=encoder_data_L;
		putinR=encoder_data_R;
		
		
		
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
		
		
		//printf("ENCODER  \t%f.\r\n", putinline);

		//printf("turnR  \t%f .\r\n", turnR);
		//printf("turnL  \t%f .\r\n", turnL);
		
		//printf("%d,",encoder_data_R);
		//printf("%d,",encoder_data_L);	
		//printf("%d\n",160);
		//printf("%d\n,",sum);

		
    encoder_clear_count(ENCODER_1);                                             // 清空编码器计数
		encoder_clear_count(ENCODER_2);                                             // 清空编码器计数
}
 
void pit_handler1 (void)
{
    
		putinline=(Mid_Line[50]+Mid_Line[51]+Mid_Line[52]+Mid_Line[53])/4;
		putoutline = PID_Compute_line(&pidline, putinline);                                          //转向环输出
		if(putinline>=99)                                                           //右转
		{
				turnL=-putoutline;
				turnR=putoutline;
				
		}
		if(putinline<99)                                                           //左转
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
		
		putoutL = PID_Compute(&pid1, putinL+putinR);
		
		if(FLAG)                                     //停车
		{
				putoutL=0;
				putoutR=0;
				turnL=0;
				turnR=0;
		}
}
