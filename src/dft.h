
#ifndef __DFT_H__
#define __DFT_H__

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "config.h"
void dft_f(short *buf, short *out);

#endif


