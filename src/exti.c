
/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
//驱动
#include "serial.h"
/* Demo program include files. */
#include "config.h" //数据类型等配置文件
#include "public.h" //全局变量定义文件

#include "exti.h" 

#include "user_api.h" //用户接口函数
#include "stm32f10x.h"
#include <stdlib.h>
#include <math.h>
#include "led.h"
#include "adc.h"
#include "timer1.h"
#include "encoder_meas.h"

void exti_rcc_init(void)
{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);		
}

void exti_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /*Configure PE10 as mcu_start pin, PE11 as mcu_rd_clk pin, PE13 as mcu_n_rst pin */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
	
		GPIO_ResetBits(GPIOE, GPIO_Pin_10);//mcu_start
		GPIO_SetBits(GPIOE, GPIO_Pin_11);//mcu_rd_clk
		GPIO_ResetBits(GPIOE, GPIO_Pin_13);//mcu_n_rst

	   /*Configure PE15 and PE9 and PE8 as data select pin*/
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
		
		GPIO_ResetBits(GPIOE, GPIO_Pin_8);
		GPIO_ResetBits(GPIOE, GPIO_Pin_9);
		GPIO_ResetBits(GPIOE, GPIO_Pin_15);
	
	  /*Configure PE14 as mcu_end pin*/
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* Configure PE12 as sync pin */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);	

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 
	                               | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);	
}

void exti_init(void)
{
		EXTI_InitTypeDef  EXTI_InitStructure;  
		NVIC_InitTypeDef  NVIC_InitStructure;
	
		exti_rcc_init();
		exti_gpio_init();
		   
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource12);     /*Config EXTI12*/      
		EXTI_InitStructure.EXTI_Line = EXTI_Line12;      
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;     
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;     // for Z voltage sample
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;     
		EXTI_Init(&EXTI_InitStructure);	
	          
		NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;      
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;     
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;     
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     
		NVIC_Init(&NVIC_InitStructure); 
		
}

void get_edge_num_per_channel(void)
{
	u8 ch_num;
	
	GPIO_ResetBits(GPIOE, GPIO_Pin_15);
	GPIO_ResetBits(GPIOE, GPIO_Pin_9);
	GPIO_ResetBits(GPIOE, GPIO_Pin_8);	
	ch_num = (u8)(GPIO_ReadInputData(GPIOE)&0x00ff);
	sys_para.ch_num = ch_num;
	
	if(ch_num<=1 || ch_num>=(TEST_CHANNEL_MAX+1))
		return;//like to （exit sub）
	
	GPIO_ResetBits(GPIOE, GPIO_Pin_15);
	GPIO_ResetBits(GPIOE, GPIO_Pin_9);
	GPIO_SetBits(GPIOE, GPIO_Pin_8);
	channel_data[ch_num-2].edge_num_per_cycle.uint8._0 = (u8)(GPIO_ReadInputData(GPIOE)&0x00ff);
	
	GPIO_ResetBits(GPIOE, GPIO_Pin_15);
	GPIO_SetBits(GPIOE, GPIO_Pin_9);
	GPIO_ResetBits(GPIOE, GPIO_Pin_8);	
	channel_data[ch_num-2].edge_num_per_cycle.uint8._1 = (u8)(GPIO_ReadInputData(GPIOE)&0x00ff);	


	GPIO_ResetBits(GPIOE, GPIO_Pin_15);
	GPIO_SetBits(GPIOE, GPIO_Pin_9);
	GPIO_SetBits(GPIOE, GPIO_Pin_8);	
	channel_data[ch_num-2].clk_num_per_cycle.uint8._0 = (u8)(GPIO_ReadInputData(GPIOE)&0x00ff);	

	GPIO_SetBits(GPIOE, GPIO_Pin_15);
	GPIO_ResetBits(GPIOE, GPIO_Pin_9);
	GPIO_ResetBits(GPIOE, GPIO_Pin_8);	
	channel_data[ch_num-2].clk_num_per_cycle.uint8._1 = (u8)(GPIO_ReadInputData(GPIOE)&0x00ff);		
	
	GPIO_SetBits(GPIOE, GPIO_Pin_15);
	GPIO_ResetBits(GPIOE, GPIO_Pin_9);
	GPIO_SetBits(GPIOE, GPIO_Pin_8);	
	channel_data[ch_num-2].clk_num_per_cycle.uint8._2 = (u8)(GPIO_ReadInputData(GPIOE)&0x00ff);	
	
	GPIO_SetBits(GPIOE, GPIO_Pin_15);
	GPIO_SetBits(GPIOE, GPIO_Pin_9);
	GPIO_ResetBits(GPIOE, GPIO_Pin_8);	
	channel_data[ch_num-2].clk_num_per_cycle.uint8._3 = (u8)(GPIO_ReadInputData(GPIOE)&0x00ff);	
	
}	
	
void EXTI15_10_IRQHandler(void) 
{       	
	if(EXTI_GetITStatus(EXTI_Line12)!= RESET)      
	{            
		EXTI_ClearITPendingBit(EXTI_Line12); 	
		if(sys_para.status.bits.sample_en == 1)
		{
 			get_edge_num_per_channel();
			if(sys_para.ch_num == 1)//((sys_para.ch_num >= 1) && (sys_para.ch_num <= WAVE_CHANNEL_CNT))
			{
				adc1_reinit();
				timer1_reinit(937-1);//937*0.25us=234.25us(234.375us)
			}
			else if((sys_para.ch_num > WAVE_CHANNEL_CNT) && (sys_para.ch_num < ADC_CHANNEL_MAX))
			{
				ADC_SoftwareStartConvCmd(ADC1,ENABLE);
// 				adc1_reinit();
// 				timer1_reinit(32-1);//32*0.25us=8us
			}
		}
	} 
} 