
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


void Delay_mS( u32 nmS )
{
	volatile u32 i, j;			//"volatile" is essential for these two variables to avoid being optized by compiler
	for(i=0;i<nmS;i++)
		for(j=0;j<5980;j++)		//5538 for "volatile u32 nmS"
		;	
}

void Delay_1uS( void )		 //1.432us-0.293us=	1.139us
{
	volatile u16 i;			//"volatile" is essential for these two variables to avoid being optized by compiler
		for(i=0;i<4;i++)		//
		;	
}

void Delay_3uS( void )
{
	volatile u16 i;			//"volatile" is essential for these two variables to avoid being optized by compiler
		for(i=0;i<12;i++)		//
		;	
}
void Delay_4uS( void )
{
	volatile u16 i;			//"volatile" is essential for these two variables to avoid being optized by compiler
		for(i=0;i<17;i++)		//
		;	
}
void Delay_5uS( void )
{
	volatile u16 i;			//"volatile" is essential for these two variables to avoid being optized by compiler
		for(i=0;i<22;i++)		//
		;	
}
void Delay_7uS( void )
{
	volatile u16 i;			//"volatile" is essential for these two variables to avoid being optized by compiler
		for(i=0;i<30;i++)		//
		;	
}
void Delay_10uS( u16 nuS )
{
	volatile u16 i, j;			//"volatile" is essential for these two variables to avoid being optized by compiler
	for(i=0;i<nuS;i++)
		for(j=0;j<41;j++)		//
		;	
}