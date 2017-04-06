
#include "config.h"
#include "user_api.h"
#include "public.h" //全局变量定义文件
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Library includes. */
#include "stm32f10x_conf.h"

#include "led.h"
#include "serial.h"
#include "delay.h"

void timer1_rcc_init(void)
{
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM1,ENABLE );
}

void timer1_init_power_on( uint16_t time_period )
{
	TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
	TIM_OCInitTypeDef         TIM_OCInitStructure;
	
	TIM_DeInit(TIM1);
	timer1_rcc_init();

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = time_period;
	TIM_TimeBaseStructure.TIM_Prescaler = 17;//17~0.25us//71~1us//
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//TIM_OCMode_Toggle;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	TIM_Cmd(TIM1, ENABLE);
	
}

void timer1_reinit( uint16_t time_period )
{
	TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
	TIM_OCInitTypeDef         TIM_OCInitStructure;
	
// 	TIM_DeInit(TIM1);
// 	timer1_rcc_init();
	TIM_SetCounter(TIM1, time_period-2);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = time_period;
	TIM_TimeBaseStructure.TIM_Prescaler = 17;//17~0.25us//71~1us//
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//TIM_OCMode_Toggle;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	TIM_Cmd(TIM1, ENABLE);
	
}







