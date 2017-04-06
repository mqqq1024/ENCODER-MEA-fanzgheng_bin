
#ifndef _TIMER1_H__
#define _TIMER1_H__

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stm32f10x_conf.h"
#include "config.h"


void timer1_reinit( uint16_t time_period );
void timer1_init_power_on( uint16_t time_period );

#endif


