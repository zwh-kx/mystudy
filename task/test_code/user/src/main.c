#include "zf_common_headfile.h"

#define LED1                    (H2 )
#define LED2                    (B13)

#define SWITCH1                 (D3 )
#define SWITCH2                 (D4 )

uint32 count_time=1000;
int32 encoder1;
int32 encoder2;

#include "key.h"
#include "auto_menu.h"

int main (void)
{
	clock_init(SYSTEM_CLOCK_120M);                                              // 初始化芯片时钟 工作频率为 120MHz
    debug_init();                                                               // 初始化默认 Debug UART
	
	//菜单初始化
    menu_init();

    while(1)
    {
		//运行菜单
        show_process(NULL);
        system_delay_ms(20);
    }
}
	

