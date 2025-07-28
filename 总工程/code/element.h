#ifndef _element_h_
#define _element_h_

extern uint8 Right_Down_Find;
extern uint8 Left_Down_Find;
extern uint8 Search_Stop_Line;
extern int right_down_line;
extern int left_down_line;
extern int i,t;
extern uint8 Left_Line [MT9V03X_H];
extern uint8 Right_Line [MT9V03X_H];
extern uint8 Right_Up_Find;
extern uint8 Left_Up_Find;
extern float k;
extern uint8 Cross_Flag;
extern uint8 Left_Lost_Time;
extern uint8 Right_Lost_Time;
extern uint8 Both_Lost_Time;
extern int monotonicity_change_line;
extern int continuity_change_flag_R;
extern int continuity_change_flag_L;

//寻点
void Find_Down_Point(int start,int end);
void Find_Up_Point(int start,int end);
int Find_Right_Down_Point(int start,int end);
int Find_Left_Down_Point(int start,int end);
int Monotonicity_Change_Right(int start,int end);
int Monotonicity_Change_Left(int start,int end);
//补线
void Left_Add_Line(int x1,int y1,int x2,int y2);
void Right_Add_Line(int x3,int y3,int x4,int y4);
void Lengthen_Right_Boundry(int start,int end);
void Lengthen_Left_Boundry(int start,int end);
//十字处理
void Cross_Detect();
//连续性
int Continuity_Change_Right(int start,int end);
int Continuity_Change_Left(int start,int end);


#endif