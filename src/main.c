
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "config.h"
//#include "platform_config.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "main.h"

#include "user_api.h"
#include "serial.h"
#include "led.h"
#include "adc.h"
#include "timer1.h"
#include "encoder_meas.h"
#include "uart_com.h"
#include "exti.h"

volatile SYS_PARA sys_para;
uint16 zero_cross_vol[16]=
{
	0x07FE,//0
	0x07FE,//1
	0x080A,//2
	0x02DC,//3
	0x02CF,//4
	0x02C6,//5
	0x02E0,//6
	0x02D9,//7
	0x02D5,//8
	0x02DF,//9
	0x02DE,//10
	0x02D4,//11
	0x02DD,//12
	0x02D5,//13
	0x02D5,//14
	0x02D9 //15
};

#define TEST 0
#if TEST==1
void start_test_task( unsigned portBASE_TYPE uxPriority );
#endif


void NVIC_PriorityGroupConfiguration(void);
void module_init(void);
void iwdg_init(void);

int main(void)
{
		//NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x4000);
    /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
    */        
    /* System Priority Group Configuration */
    NVIC_PriorityGroupConfiguration();

    module_init();   

#if TEST==1
    start_test_task(TEST_PRIORITY);
#else
		start_get_vol_task( GET_VOL_PRIORITY );
 		start_cpld_test_task( CPLD_TEST_PRIORITY );
		start_uart_com_task( UART_COM_PRIORITY );		

#endif   
    /* Now all the tasks have been started - start the scheduler.

		NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
		The processor MUST be in supervisor mode when vTaskStartScheduler is 
		called.  The demo applications included in the FreeRTOS.org download switch
		to supervisor mode prior to main being called.  If you are not using one of
		these demo application projects then ensure Supervisor mode is used here. */
		vTaskStartScheduler();
    
    return 0;
}

/**
  * @brief  init module 
  * @param  None
  * @retval None
  */
void module_init(void)
{
  uint8 i;
	
	led_init();
	exti_init();
#if WATCH_DOG==1
	iwdg_init();
#endif
	adc1_init();
	timer1_init_power_on(10000-1);//timer1 trigger interval is 10000*0.25us=2500us
	usart_init(1, 38400);
	vSemaphoreCreateBinary( read_cpld );
	sys_para.status.uint8 = 0;
	
	for(i=0;i<16;i++)
	{
		channel_data[i].vol_zero_cross.uint16 = zero_cross_vol[i];
	}
}


/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void NVIC_PriorityGroupConfiguration(void)
{
     NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 ); /* group 4*/
}

/**
  * @brief  idle task hook.
  * @param  None
  * @retval None
  */
void vApplicationIdleHook( void )
{
    portTickType tick;
    static portTickType prev_tick = 0;
    
#if WATCH_DOG==1
    IWDG_ReloadCounter();
#endif    
    tick = xTaskGetTickCount();
    
    if(get_elapsed_ticks(prev_tick, tick )>1000/portTICK_RATE_MS)
    {
//       toggle_run_led();
       prev_tick = xTaskGetTickCount();
    }
}

#if TEST==1

static portTASK_FUNCTION( test_task, pvParameters )
{
    while(1)
    {    		 
        vTaskDelay(500/portTICK_RATE_MS);
//			set_run_led_on_core();
        vTaskDelay(500/portTICK_RATE_MS);
//			set_run_led_off_core();
    }    
}

void start_test_task( unsigned portBASE_TYPE uxPriority )
{
    
	/* Spawn the task. */
	xTaskCreate( test_task, ( const signed portCHAR * const ) "TEST", TEST_STACK_SIZE, NULL, uxPriority, ( xTaskHandle * ) NULL );
}


#endif


void iwdg_init(void)
{
    /* 写入0x5555,用于允许狗狗寄存器写入功能 */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

    /* 狗狗时钟分频,40K/256=156HZ(6.4ms)*/
    IWDG_SetPrescaler(IWDG_Prescaler_256);

    /* 喂狗时间 10s/6.4MS=1562 .注意不能大于0xfff*/
    IWDG_SetReload(1562);

    /* 喂狗*/
    IWDG_ReloadCounter();

    /* 使能狗狗*/
    IWDG_Enable();
}

