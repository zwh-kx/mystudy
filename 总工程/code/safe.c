#include "zf_common_headfile.h"
#include "element.h"

uint8    FLAG=0;  
int16 encoder_data_L = 0;
int16 encoder_data_R = 0;

void safe(void)
{
	if(Search_Stop_Line<3)                                                     //识别到的最长白列过短视为出界
		{
				FLAG=1;
		}
		if(encoder_data_L>=1500)                      //堵转保护
		{
				FLAG=1;
		}
		if(encoder_data_L<=-1500)                      
		{
				FLAG=1;
		}
		if(encoder_data_R>=1500)                      
		{
				FLAG=1;
		}
		if(encoder_data_R<=-1500)                      
		{
				FLAG=1;
		}
}