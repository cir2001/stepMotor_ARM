#include <stm32f10x.h>
#include "delay.h"

void delay_ms(u32 nms)
{	 		  	  
	while(nms--)   
	delay_us(1000);
}   
//延时nus
//nus为要延时的us数.		    								   
void delay_us(u32 nus)
{		
	u8 j;
	while(nus--)
	for(j=0;j<10;j++);
}

