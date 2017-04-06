
#ifndef __UART_COM_H__
#define __UART_COM_H__

#include "config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

void start_uart_com_task( unsigned portBASE_TYPE uxPriority );

#endif
