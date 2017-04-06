
#ifndef __AD_H__
#define __AD_H__

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "config.h"

void adc1_init( void );
void start_get_vol_task( unsigned portBASE_TYPE uxPriority );
void adc1_reinit(void);
void calc_uvw_vol_rms( void );
void get_hall_vol_from_wave(uint16* adc_buf);

#define ADC1_DR_Address 			(u32)0x4001244c //ADC×ª»»Êý¾Ý¼Ä´æÆ÷
#define ADC_CHANNEL_MAX   		16
#define WAVE_CHANNEL_CNT 	    9
#define WAVE_LEN_MAX 					1000
#define DMA_BUF_MAX   				256
#define ZERO_CROSS_SAMPLE_NUM 20

extern volatile uint16 channel_wave[9][WAVE_LEN_MAX];

#endif


