#include "zf_common_headfile.h"
#include "key.h"
#include "auto_menu.h"
uint8 key1_state = 0;																// ��������״̬
uint8 key1_state_last = 0;															// ��һ�ΰ�������״̬
uint8 key1_flag=0;                                //   1���£��ɿ�0
uint8 key1_count=0;

uint8 key2_state = 0;																// ��������״̬
uint8 key2_state_last = 0;															// ��һ�ΰ�������״̬
uint8 key2_flag=0;                                //   1���£��ɿ�0
uint8 key2_count=0;

uint8 key3_state = 0;																// ��������״̬
uint8 key3_state_last = 0;															// ��һ�ΰ�������״̬
uint8 key3_flag=0;                                //   1���£��ɿ�0
uint8 key3_count=0;

uint8 key4_state = 0;																// ��������״̬
uint8 key4_state_last = 0;															// ��һ�ΰ�������״̬
uint8 key4_flag=0;                                //   1���£��ɿ�0
uint8 key4_count=0;

#ifdef  MENU_USE_RTT
//�����ź���
rt_sem_t key1_sem;
rt_sem_t key2_sem;
rt_sem_t key3_sem;
rt_sem_t key4_sem;
//���������ź���
rt_sem_t button_feedback_sem;
#endif

int  key1_can(void)
{
	      key1_state_last = key1_state;
		    key1_state = gpio_get_level(KEY_1);
				if(key1_state && !key1_state_last)   
				{					
					key1_flag = 1;
		      return  1;
				}
	else 
		return 0;
}

int  key2_can(void)
{
	      key2_state_last = key2_state;
		    key2_state = gpio_get_level(KEY_2);
				if(key2_state && !key2_state_last)   
				{					
					key2_flag = 1;
		      return  1;
				}
	else 
		return 0;
}


int  key3_can(void)
{
	      key3_state_last = key3_state;
		    key3_state = gpio_get_level(KEY_3);
				if(key3_state && !key3_state_last)   
				{					
					key3_flag = 1;
		      return  1;
				}
	else 
		return 0;
}

int  key4_can(void)
{
	      key4_state_last = key4_state;
		    key4_state = gpio_get_level(KEY_4);
				if(key4_state && !key4_state_last)   
				{					
					key4_flag = 1;
		      return  1;
				}
	else 
		return 0;
}

void button_entry(void *parameter)
{
	static uint8 long_press_cnt=0;
	static uint8 long_press_button=0;
    #ifdef  MENU_USE_RTT
	while(1)
	{
    #endif
		//���水��״̬
		key1_state_last = key1_state;
		key2_state_last = key2_state;
		key3_state_last = key3_state;
		key4_state_last = key4_state;
		
		//��ȡ��ǰ����״̬
		key1_state = gpio_get_level(KEY_1);
		key2_state = gpio_get_level(KEY_2);
		key3_state = gpio_get_level(KEY_3);
		key4_state = gpio_get_level(KEY_4);
//		key4_state = 1;
		
//		printf("key:%d %d %d %d\n",key1_state,key2_state,key3_state,key4_state);

		
		//�������
		if(!key1_state)
		{
			long_press_cnt++;
			long_press_cnt=long_press_cnt>LONG_PRESS_TIME?LONG_PRESS_TIME:long_press_cnt;
			if(long_press_cnt>=LONG_PRESS_TIME)long_press_button=1;
		}
		else if(!key2_state)
		{
			long_press_cnt++;
			long_press_cnt=long_press_cnt>LONG_PRESS_TIME?LONG_PRESS_TIME:long_press_cnt;
			if(long_press_cnt>=LONG_PRESS_TIME)long_press_button=2;
		}
		else if(!key3_state)
		{
			long_press_cnt++;
			long_press_cnt=long_press_cnt>LONG_PRESS_TIME?LONG_PRESS_TIME:long_press_cnt;
			if(long_press_cnt>=LONG_PRESS_TIME)long_press_button=3;
		}
		else if(!key4_state)
		{
			long_press_cnt++;
			long_press_cnt=long_press_cnt>LONG_PRESS_TIME?LONG_PRESS_TIME:long_press_cnt;
			if(long_press_cnt>=LONG_PRESS_TIME)long_press_button=4;
		}
		else
		{
			long_press_cnt=0;
			long_press_button=0;
		}
			
		//��⵽��������֮�󲢷ſ� �ͷ�һ���ź���
		extern uint8 button1,button2,button3,button4;
		button1 = 0;button2 = 0;button3 = 0;button4 = 0;
		if((key1_state && !key1_state_last)||long_press_button==1)    
		{
            #ifdef  MENU_USE_RTT
		    rt_sem_release(key1_sem);
			rt_sem_take(button_feedback_sem,RT_WAITING_FOREVER);
            #else
            button1 = 1;
            #endif
		}
		if((key2_state && !key2_state_last)||long_press_button==2)    
		{
            #ifdef  MENU_USE_RTT
			rt_sem_release(key2_sem);
			rt_sem_take(button_feedback_sem,RT_WAITING_FOREVER);
            #else
            button2 = 1;
            #endif
		}
		if((key3_state && !key3_state_last)||long_press_button==3)    
		{
            #ifdef  MENU_USE_RTT
			rt_sem_release(key3_sem);
			rt_sem_take(button_feedback_sem,RT_WAITING_FOREVER);
            #else
            button3 = 1;
            #endif
        }
		if((key4_state && !key4_state_last)||long_press_button==4)    
		{
            #ifdef  MENU_USE_RTT
			rt_sem_release(key4_sem);
			rt_sem_take(button_feedback_sem,RT_WAITING_FOREVER);
            #else
            button4 = 1;
            #endif
		}
        #ifdef  MENU_USE_RTT
		rt_thread_mdelay(20);
        #endif
    #ifdef  MENU_USE_RTT
	}
    #endif
}

void key_into()
{
	
    gpio_init(KEY_1, GPI, GPIO_LOW, GPI_PULL_UP);                                   // ��ʼ��ΪGPIO�������� Ĭ�������ߵ�ƽ
    gpio_init(KEY_2, GPI, GPIO_LOW, GPI_PULL_UP);                                   // ��ʼ��ΪGPIO�������� Ĭ�������ߵ�ƽ
    gpio_init(KEY_3, GPI, GPIO_LOW, GPI_PULL_UP);                                   // ��ʼ��ΪGPIO�������� Ĭ�������ߵ�ƽ
    gpio_init(KEY_4, GPI, GPIO_LOW, GPI_PULL_UP);                                   // ��ʼ��ΪGPIO�������� Ĭ�������ߵ�ƽ
    #ifdef  MENU_USE_RTT
	rt_thread_t tid;
	key1_sem = rt_sem_create("key1", 0, RT_IPC_FLAG_FIFO);  //�����������ź��������������¾��ͷ��ź���������Ҫʹ�ð����ĵط���ȡ�ź�������
	key2_sem = rt_sem_create("key2", 0, RT_IPC_FLAG_FIFO);  
	key3_sem = rt_sem_create("key3", 0, RT_IPC_FLAG_FIFO);  
	key4_sem = rt_sem_create("key4", 0, RT_IPC_FLAG_FIFO);  
 	
	button_feedback_sem = rt_sem_create("button_feedback",1,RT_IPC_FLAG_FIFO);
	
	tid = rt_thread_create("button",button_entry,RT_NULL,512,12,2);
	//���������߳�
	if(RT_NULL != tid)
	{
			rt_thread_startup(tid);
	}	
    #endif
}

