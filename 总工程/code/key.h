#ifndef _key_h
#define _key_h

#define KEY_1               E2                                                       // ���������ϰ�����Ӧ����
int  key1_can(void);

#define KEY_2               E5                                                       // ���������ϰ�����Ӧ����
int  key2_can(void);

#define KEY_3               E3                                                       // ���������ϰ�����Ӧ����
int  key3_can(void);

#define KEY_4               E4                                                       // ���������ϰ�����Ӧ����
int  key4_can(void);

#define LONG_PRESS_TIME	    30	//ԼΪ30*20*2=600ms
extern void key_into(void);
extern void button_entry(void *parameter);

#endif








