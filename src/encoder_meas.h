
#ifndef __ENCODER_MEAS_H__
#define __ENCODER_MEAS_H__

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "stm32f10x.h"
#include "config.h"

#define MCU_END GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_14)
#define MCU_START_H	GPIO_SetBits(GPIOE, GPIO_Pin_10)
#define MCU_START_L	GPIO_ResetBits(GPIOE, GPIO_Pin_10)
#define MCU_RD_CLK_H	GPIO_SetBits(GPIOE, GPIO_Pin_11)//œ‡ŒªΩ«
#define MCU_RD_CLK_L	GPIO_ResetBits(GPIOE, GPIO_Pin_11)
#define MCU_N_RST_H	GPIO_SetBits(GPIOE, GPIO_Pin_13)
#define MCU_N_RST_L	GPIO_ResetBits(GPIOE, GPIO_Pin_13)

#define TEST_CHANNEL_MAX 26

/*
 * list all test results of each channel
 * 
 */
struct channel_msg
{
	double k;
	short b;
	tMEM16 vol_zero_cross_tmp;
	tMEM16 vol_zero_cross;
	tMEM16 vol_rms;
	tMEM16 vol_high_max;
	tMEM16 vol_high_min;
	tMEM16 vol_high_mean;
	tMEM16 vol_low_max;
	tMEM16 vol_low_min;
	tMEM16 vol_low_mean;
	
	tMEM16 pulse_cnt;
	uint16 edge_cnt_readout;
	tMEM16 edge_num_per_cycle;
	tMEM32 clk_num_per_cycle;
	uint16 calculated_period_num;
	
	tMEM32 period_max;//raw data
	tMEM32 period_min;
	tMEM32 period_mean;
	uint32 period_sum;
	
	tMEM32 frep_max;//0.01hz
	tMEM32 frep_min;
	tMEM32 frep_mean;
	uint64_t frep_sum;
	
	tMEM16 duty_max;//0.01%
	tMEM16 duty_min;
	tMEM16 duty_mean;
	uint32 duty_sum;
	
	tMEM32 phase_diff_max;//raw data
	tMEM32 phase_diff_min;
	tMEM32 phase_diff_mean;
	uint32 phase_diff_sum;

	tMEM32 z_phase_diff;//raw data
	tMEM32 z_width;//raw data
	
	uint16 wave_length;
	uint16 zeropos[20];
	uint16 zerocnt;	
	
}__attribute__ ((packed));

union u32_bytes
{
	uint8 _byte[4];
	uint32 _u32;	
	
}__attribute__ ((packed));

union sys_status 
{
	struct __attribute__ ((packed))
	{
		uint8 sys_state:4; 
		uint8 sample_en:1; 
		uint8 get_zero_cross_flag:1; 
		uint8 wave_sample_end:1;
		uint8 :1; 
	} __attribute__ ((packed)) bits;
	uint8 uint8;
}__attribute__ ((packed));

typedef struct __attribute__ ((packed))
{
	union sys_status status;
	uint8 ch_num;
} __attribute__ ((packed)) SYS_PARA;


extern xSemaphoreHandle read_cpld;
extern volatile SYS_PARA sys_para;
extern volatile struct channel_msg channel_data[];

void start_cpld_test_task( unsigned portBASE_TYPE uxPriority );
void send_debug_data(void);

#endif

