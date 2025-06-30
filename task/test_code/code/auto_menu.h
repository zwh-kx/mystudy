#ifndef _AUTO_MENU_H
#define _AUTO_MENU_H
#include "zf_common_headfile.h"

//#define DEBUG_PRINTF
#define _pid  struct controller

//#define MENU_USE_RTT

#ifdef DEBUG_PRINTF
	#define	debug_printf(format, ...) printf(format, ##__VA_ARGS__)
#else
	#define	debug_printf(format, ...)
#endif

#define MAP_INIT_BEGIN		(0x01<<0)
#define MAP_INIT_END		(0x01<<1)


#define SON_NUM				(6)  //�Ƽ�Ϊ2��������



#define STR_LEN_MAX			12
#define MOUSE_DIS			10
#define MOUSE_LOOK			">"

#define BEGIN_BGCOLOR		BLACK

#define PID_RING_LEN		600


#define IND1 								(P_dad_head->down->m_index[0])
#define IND2 								(P_dad_head->down->enter->down->m_index[1])
#define SON_END_UNIT 						(P_dad_head->down->enter->down)
#define SON_BEGIN_UNIT 						(P_dad_head->down->enter)
#define DAD_INDEX(_ind1,_ind2)				(*(p_index_xy_dad+_ind1+DAD_NUM*_ind2))
#define SON_INDEX(_ind1,_ind2)				(*(p_index_xy_son+_ind1+SON_NUM*_ind2))
#define IS_PRESS							(button1||button2||button3||button4)
#define CLEAR_PID_DRAW()					ips200_clear_hhh(0,(SON_NUM)*16,SCREEN_W-1,SCREEN_H-1,IPS200_BGCOLOR)
#define IS_OK 								(button2==1)&&(!first_in_page_flag)
 
//Ŀǰֻ֧��4�ֳ������͵Ĳ���������������Ҫ���������
typedef enum TYPE_VALUE{TYPE_FLOAT=1,TYPE_DOUBLE,TYPE_INT,TYPE_UINT16,TYPE_UINT32}type_value;

typedef enum UNIT_TYPE{USE_FUN=1,NORMAL_PAR,PID_PAR}unit_type;

//ʹ��tft��ips200(ǰ��ΪTFT��ƣ���������ips�Ͽ�����)
#define TFT						0
#define IPS						1
#define SCREEN_TYPE		        IPS

#if SCREEN_TYPE==TFT
	#define 		screen_init				lcd_init
	#define 		clear							lcd_clear
	#define 		showfloat					lcd_showfloat
	#define 		showimage032			lcd_displayimage032
	#define 		showstr						lcd_showstr
	#define 		showint32					lcd_showint32
	#define 		showuint16				lcd_showuint16
	#define 		showuint16				lcd_showuint16
	#define		  DIS_X							TFT_X_MAX/2
	#define		  DIS_Y							1
	#define		  SCREEN_W					TFT_X_MAX
	#define		  SCREEN_H					TFT_Y_MAX
#else
	#define 		IPS200_W				240
    #define         IPS200_H                320
	#define 		screen_init				ips200_init
	#define 		clear					ips200_clear
	#define 		clear_hhh				ips200_clear_hhh
	#define 		showfloat				ips200_show_float
	#define 		showimage032			ips200_displayimage032
	#define 		showstr     			ips200_show_string
	#define 		showint32				ips200_show_int
	#define 		showuint32				ips200_show_uint
	#define 		showuint16		        ips200_show_uint
	#define 		drawpoint				ips200_draw_point
	#define		    DIS_X					IPS200_W/2
	#define		    DIS_Y					16
	#define		    SCREEN_W				IPS200_W
	#define		    SCREEN_H				IPS200_H
#endif

//  @brief      �˵�Ԫ�ؽṹ��
//  @param      bool use_fun;											�Ƿ�ʹ�ù��ܺ���ָ�루��ֻ���޸Ĳ�������Ϊfalse��
//  @param      void* p_par;											ָ����޸Ĳ�����ָ��
//  @param      type par_type;										���޸Ĳ�����ָ������
//  @param      float delta;											�޸Ĵ�С
//  @param      uint8 num;						 						��ʾ������λ
//  @param      uint8 point_num;									��ʾ��С��λ
typedef struct PARAM_SET{
	void* 				p_par;
	float 				delta;
	type_value 		    par_type;
	uint8 				num;
	uint8				point_num;
}param_set;

typedef struct MENU_UNIT
{
	param_set* par_set;//
	struct MENU_UNIT* up;//��
	struct MENU_UNIT* down;//��
	struct MENU_UNIT* enter;//����
	struct MENU_UNIT* back;//����
	void (*current_operation)();
	char  name[STR_LEN_MAX];
	uint8 m_index[2];
	unit_type 		type_t;
}menu_unit;

#define USE_STATIC_MENU

#ifdef USE_STATIC_MENU
#define MEM_SIZE    100
#endif

extern void menu_init(void);

void show_process(void *parameter);

void fun_dad(uint8 index_dad,uint8 useless);
void fun_son(uint8 index_son1,uint8 index_son2);
void dad_name_init();

//�պ���
void NULL_FUN();
//���еĲ������þ�����
void UNIT_SET();
//���еĺ������þ�����
void FUN_INIT();

//�˴���Ӵ�����
void Flash_Read();
void Flash_Write();
void Flash_Erase();


#endif

