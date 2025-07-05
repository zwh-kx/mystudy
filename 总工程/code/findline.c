#include "zf_common_headfile.h"


    uint8    image_deal[120][188]={0};

		int leftline[120];  //赛道左边界
		int rightline[120];  //赛道右边界
		int centerline[120]; //赛道中间线
 


		int line=0;
		int list=0;
void image_scan()
		{
		for(line=120;line>=40;line--)
		{
		for(list=93;list<188;list++)
		{
    if((image_deal[line][list-2]==1)&&(image_deal[line][list-1]==0) 
       &&(image_deal[line][list]==0))
     {
       rightline[line]=list;
       	break;
     }
   }
 
   for(list=93;list>1;list--)
   {
    if((image_deal[line][list]==0)&&(image_deal[line][list+1]==0)
        &&(image_deal[line][list+2]==1)) 
     {
       leftline[line]=list;
      break;
     }
   }
	centerline[line]=(rightline[line]+leftline[line])/2;
  }
}
		