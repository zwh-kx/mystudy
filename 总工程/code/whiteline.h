#ifndef _whiteline_h_
#define _whiteline_h_

extern uint8    H;                                                                      //高
extern uint8    W;                                                                      //宽
extern uint8 Mid_Line [MT9V03X_H];
extern uint8 Longest_White_Column_Left[2];                                      //0是长度，1是下标
extern uint8 Longest_White_Column_Right[2];
extern uint8 White_Column[MT9V03X_W];

void findline();

#endif