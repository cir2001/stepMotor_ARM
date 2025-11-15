#include "sys.h" 
//?????????
//NVIC_VectTab:??
//Offset:???			 
void MY_NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset)	 
{ 	   	 
	SCB->VTOR = NVIC_VectTab|(Offset & (u32)0x1FFFFF80);//??NVIC?????????
	//?????????CODE????RAM?
}
//??NVIC??
//NVIC_Group:NVIC?? 0~4 ??5? 		   
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group)	 
{ 
	u32 temp,temp1;	  
	temp1=(~NVIC_Group)&0x07;//????
	temp1<<=8;
	temp=SCB->AIRCR;  //???????
	temp&=0X0000F8FF; //??????
	temp|=0X05FA0000; //????
	temp|=temp1;	   
	SCB->AIRCR=temp;  //????	    	  				   
}
//设置NVIC 
//NVIC_PreemptionPriority:抢占优先级
//NVIC_SubPriority       :响应优先级
//NVIC_Channel           :中断编号
//NVIC_Group             :中断分组 0~4
//注意优先级不能超过设定的组的范围!否则会有意想不到的错误
//组划分:
//组0:0位抢占优先级,4位响应优先级
//组1:1位抢占优先级,3位响应优先级
//组2:2位抢占优先级,2位响应优先级
//组3:3位抢占优先级,1位响应优先级
//组4:4位抢占优先级,0位响应优先级
//NVIC_SubPriority和NVIC_PreemptionPriority的原则是,数值越小,越优先	   
void MY_NVIC_Init(u8 NVIC_PreemptionPriority,u8 NVIC_SubPriority,u8 NVIC_Channel,u8 NVIC_Group)	 
{ 
	u32 temp;	
	MY_NVIC_PriorityGroupConfig(NVIC_Group);//????
	temp=NVIC_PreemptionPriority<<(4-NVIC_Group);	  
	temp|=NVIC_SubPriority&(0x0f>>NVIC_Group);
	temp&=0xf;								//????  
	NVIC->ISER[NVIC_Channel/32]|=(1<<NVIC_Channel%32);//?????(?????,?????OK) 
	NVIC->IP[NVIC_Channel]|=temp<<4;		//?????????????   	    	  				   
} 
//外部中断配置函数
//只针对GPIOA~G;不包括PVD,RTC和USB唤醒这三个
//参数:
//GPIOx:0~6,代表GPIOA~G
//BITx:需要使能的位;
//TRIM:触发模式,1,下升沿;2,上降沿;3，任意电平触发
//该函数一次只能配置1个IO口,多个IO口,需多次调用
//该函数会自动开启对应中断,以及屏蔽线   	    
void Ex_NVIC_Config(u8 GPIOx,u8 BITx,u8 TRIM) 
{
	u8 EXTADDR;
	u8 EXTOFFSET;
	EXTADDR=BITx/4;//得到中断寄存器组的编号
	EXTOFFSET=(BITx%4)*4; 
	RCC->APB2ENR|=0x01;//使能io复用时钟			 
	AFIO->EXTICR[EXTADDR]&=~(0x000F<<EXTOFFSET);//清除原来设置！！！
	AFIO->EXTICR[EXTADDR]|=GPIOx<<EXTOFFSET;//EXTI.BITx映射到GPIOx.BITx 
	//自动设置
	EXTI->IMR|=1<<BITx;//  开启line BITx上的中断
	//EXTI->EMR|=1<<BITx;//不屏蔽line BITx上的事件 (如果不屏蔽这句,在硬件上是可以的,但是在软件仿真的时候无法进入中断!)
 	if(TRIM&0x01)EXTI->FTSR|=1<<BITx;//line BITx上事件下降沿触发
	if(TRIM&0x02)EXTI->RTSR|=1<<BITx;//line BITx上事件上升降沿触发
} 	  
//?????????????!???????????.		    
//??????????		  
void MYRCC_DeInit(void)
{	
 	RCC->APB1RSTR = 0x00000000;//????			 
	RCC->APB2RSTR = 0x00000000; 
	  
  	RCC->AHBENR = 0x00000014;  //???????SRAM????.????.	  
  	RCC->APB2ENR = 0x00000000; //??????.			   
  	RCC->APB1ENR = 0x00000000;   
	RCC->CR |= 0x00000001;     //????????HSION	 															 
	RCC->CFGR &= 0xF8FF0000;   //??SW[1:0],HPRE[3:0],PPRE1[2:0],PPRE2[2:0],ADCPRE[1:0],MCO[2:0]					 
	RCC->CR &= 0xFEF6FFFF;     //??HSEON,CSSON,PLLON
	RCC->CR &= 0xFFFBFFFF;     //??HSEBYP	   	  
	RCC->CFGR &= 0xFF80FFFF;   //??PLLSRC, PLLXTPRE, PLLMUL[3:0] and USBPRE 
	RCC->CIR = 0x00000000;     //??????		 
	//?????				  
#ifdef  VECT_TAB_RAM
	MY_NVIC_SetVectorTable(0x20000000, 0x0);
#else   
	MY_NVIC_SetVectorTable(0x08000000,0x0);
#endif
}
//THUMB?????????
//??????????????WFI  
void WFI_SET(void)
{
	__ASM volatile("wfi");		  
}
//??????
void INTX_DISABLE(void)
{		  
	__ASM volatile("cpsid i");
}
//??????
void INTX_ENABLE(void)
{
	__ASM volatile("cpsie i");		  
}
//??????
//addr:????
__asm void MSR_MSP(u32 addr) 
{
    MSR MSP, r0 			//set Main Stack value
    BX r14
}

//??????	  
void Sys_Standby(void)
{
	SCB->SCR|=1<<2;//??SLEEPDEEP? (SYS->CTRL)	   
  	RCC->APB1ENR|=1<<28;     //??????	    
 	PWR->CSR|=1<<8;          //??WKUP????
	PWR->CR|=1<<2;           //??Wake-up ??
	PWR->CR|=1<<1;           //PDDS??		  
	WFI_SET();				 //??WFI??		 
}	     
//?????   
void Sys_Soft_Reset(void)
{   
	SCB->AIRCR =0X05FA0000|(u32)0x04;	  
} 		 
//JTAG????,????JTAG???
//mode:jtag,swd????;00,???;01,??SWD;10,???;	   
//#define JTAG_SWD_DISABLE   0X02
//#define SWD_ENABLE         0X01
//#define JTAG_SWD_ENABLE    0X00		  
void JTAG_Set(u8 mode)
{
	u32 temp;
	temp=mode;
	temp<<=25;
	RCC->APB2ENR|=1<<0;     //??????	   
	AFIO->MAPR&=0XF8FFFFFF; //??MAPR?[26:24]
	AFIO->MAPR|=temp;       //??jtag??
} 
//?????????
//pll:??????,?2??,????16		 
void Stm32_Clock_Init(u8 PLL)
{
	unsigned char temp=0;   
	MYRCC_DeInit();		  //????????
 	RCC->CR|=0x00010000;  //????????HSEON
	while(!(RCC->CR>>17));//????????
	RCC->CFGR=0X00000400; //APB1=DIV2;APB2=DIV1;AHB=DIV1;
	PLL-=2;				  //??2???(????2???,??0??2)
	RCC->CFGR|=PLL<<18;   //??PLL? 2~16
	RCC->CFGR|=1<<16;	  //PLLSRC ON 
	FLASH->ACR|=0x32;	  //FLASH 2?????
	RCC->CR|=0x01000000;  //PLLON
	while(!(RCC->CR>>25));//??PLL??
	RCC->CFGR|=0x00000002;//PLL??????	 
	while(temp!=0x02)     //??PLL??????????
	{   
		temp=RCC->CFGR>>2;
		temp&=0x03;
	}    
}		    





