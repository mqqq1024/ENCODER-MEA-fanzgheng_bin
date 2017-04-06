
#include "main.h"
#include "config.h"
#include "user_api.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Library includes. */
#include "stm32f10x_conf.h"

#include "led.h"

void led_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
 
    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
 
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

//     GPIO_SetBits(GPIOB, GPIO_Pin_12);
// 		GPIO_SetBits(GPIOB, GPIO_Pin_13);
}

void toggle_run_led(void)
{
  if(0==GPIO_ReadOutputDataBit(RUN_LED_PORT, RUN_LED_PIN))
	{
    set_run_led_off();
	}
  else
	{
    set_run_led_on();
	} 
}


// if(0==GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_13))
// {
//  GPIO_SetBits(GPIOB, GPIO_Pin_13);
// }
// else
// {
//  GPIO_ResetBits(GPIOB, GPIO_Pin_13);
// } 	

