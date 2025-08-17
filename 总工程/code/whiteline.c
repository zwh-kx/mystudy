#include "zf_common_headfile.h"
#include "whiteline.h"
#include "element.h"

uint8    W;                                                                      //宽
uint8    H;                                                                      //高
uint8 down_search_start;                                                       //搜索截至行
uint8 White_Column[MT9V03X_W];
uint8 Longest_White_Column_Left[2] ={0};                                      //0是长度，1是下标
uint8 Longest_White_Column_Right[2] ={0};
uint8 Mid_Line [MT9V03X_H];                                                   //中线数组
uint8 image_deal[MT9V03X_H][MT9V03X_W];
uint8 Left_Lost_Flag[MT9V03X_H];																							//左右边界丢线标志位
uint8 Right_Lost_Flag[MT9V03X_H];
uint8 left_border;
uint8 right_border;

void findline(void)
{
	//统计白线长度
		for (W =1; W<=188; W++)
		{
				for (H = MT9V03X_H - 1; H >= 0; H--)
				{
						if(image_deal[H][W] == 0)
						{
							break;
						}
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
}