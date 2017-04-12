
/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "serial.h"
/* Demo program include files. */
#include "config.h" //
#include "public.h" //

#include "user_api.h" //
#include "stm32f10x.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "led.h"
#include "adc.h"
#include "encoder_meas.h"
#include "stm32f10x_conf.h"
#include "dft.h"

#define ch1_addr_base 0x00000
#define ch2_addr_base 0x01000
#define ch3_addr_base 0x02000
#define ch4_addr_base 0x03000
#define ch5_addr_base 0x04000
#define ch6_addr_base 0x05000
#define ch7_addr_base 0x06000
#define ch8_addr_base 0x07000
#define ch9_addr_base 0x08000
#define ch10_addr_base 0x09000
#define ch11_addr_base 0x19000
#define ch12_addr_base 0x29000
#define ch13_addr_base 0x39000
#define ch14_addr_base 0x49000
#define ch15_addr_base 0x4a000
#define ch16_addr_base 0x4b000
#define ch17_addr_base 0x4c000
#define ch18_addr_base 0x4d000
#define ch19_addr_base 0x4e000
#define ch20_addr_base 0x4f000
#define ch21_addr_base 0x50000
#define ch22_addr_base 0x51000
#define ch23_addr_base 0x52000
#define ch24_addr_base 0x53000
#define ch25_addr_base 0x54000


uint32 ch_base_addr[]={
	ch1_addr_base,
	ch2_addr_base,
	ch3_addr_base,
	ch4_addr_base,
	ch5_addr_base,
	ch6_addr_base,
	ch7_addr_base,
	ch8_addr_base,
	ch9_addr_base,
	ch10_addr_base,
	ch11_addr_base,
	ch12_addr_base,
	ch13_addr_base,
	ch14_addr_base,
	ch15_addr_base,
	ch16_addr_base,
	ch17_addr_base,
	ch18_addr_base,
	ch19_addr_base,
	ch20_addr_base,
	ch21_addr_base,
	ch22_addr_base,
	ch23_addr_base,
	ch24_addr_base,
	ch25_addr_base
};

xSemaphoreHandle read_cpld;
volatile struct channel_msg channel_data[TEST_CHANNEL_MAX];
short dft_buf[3][512];
short dft_output[3][512];
extern u16 adc_tmp[6][257];
uint32 data_buf[TEST_CHANNEL_MAX][30];
volatile uint32 A_buf[2100];
volatile uint32 B_buf[2100];
tMEM16 uvwz_edge_phase_buf[5][30];

void calc_cpld_data(uint32 data, uint16 ch_num)
{
	static uint32 tmp0, tmp1, tmp2, tmp3;
	uint32 frep, period, phase_diff;
	uint16 duty;	

// 	if(channel_data[ch_num].edge_num_per_cycle.uint16  <= 3)
// 	{
// 		if(channel_data[ch_num].edge_cnt_readout == 1)
// 		{
// 			tmp0 = data;
// 			channel_data[ch_num].z_width.uint32 = tmp0&0x7fffffff;
// 		}
// 		else if(channel_data[ch_num].edge_cnt_readout == 2)//include differential signal
// 		{
// 			if((tmp0&0x7fffffff) <= 0x0F)
// 			{
// 				channel_data[ch_num].z_width.uint32 = (data&0x7fffffff) - (tmp0&0x7fffffff);
// 			}
// 		}
// 	}
// 	else
// 	{
		if(channel_data[ch_num].edge_cnt_readout == 1)
		{
			tmp0 = data;
			if((tmp0 & 0x80000000) == 0 && (tmp0 & 0x7fffffff) != 1)
			{
				channel_data[ch_num].z_phase_diff.uint32 = tmp0 & 0x7fffffff;
			}
		}
		else if(channel_data[ch_num].edge_cnt_readout == 2)
		{
			tmp1 = data;
			if((tmp1 & 0x80000000) == 0)
			{
				channel_data[ch_num].z_phase_diff.uint32 = tmp1 & 0x7fffffff;
			}
		}
		else if(channel_data[ch_num].edge_cnt_readout == 3)
		{
			tmp2 = data;
			if((tmp2 & 0x80000000)==0 && (tmp0 & 0x7fffffff) == 1)
			{
				channel_data[ch_num].z_phase_diff.uint32 = tmp2 & 0x7fffffff;
			}
		}
		else if((channel_data[ch_num].edge_cnt_readout % 2 == 0) && (channel_data[ch_num].edge_cnt_readout >= 4))
		{
			tmp3 = data;
			period = tmp3 - tmp1;					
			frep = (uint32)((double)(40 * 1000000 * 1.0 / period) * 100); //0.01hz
			if(tmp3 & 0x80000000)
			{
				duty = (u16)(((double)(((tmp3 & 0x7fffffff) - tmp2) * 1.0)/(double)period * 1.0) * 10000);//0.01%
				phase_diff = (tmp3 & 0x7fffffff) - tmp2;		
			}
			else
			{
				duty = (u16)(((double)(((tmp2 & 0x7fffffff) - tmp1) * 1.0)/(double)period * 1.0) *10000);//0.01%
				phase_diff = (tmp2 & 0x7fffffff) - tmp1;//phase_diff？？
			}

			if(period > channel_data[ch_num].period_max.uint32)
				channel_data[ch_num].period_max.uint32 = period; // period = tmp3 - tmp1;	周波比大小周期计算
			if(period < channel_data[ch_num].period_min.uint32)
				channel_data[ch_num].period_min.uint32 = period;				
			channel_data[ch_num].period_sum = channel_data[ch_num].period_sum  + period;			

			if(frep > channel_data[ch_num].frep_max.uint32)
				channel_data[ch_num].frep_max.uint32 = frep; 
			if(frep < channel_data[ch_num].frep_min.uint32)
				channel_data[ch_num].frep_min.uint32 = frep;				
			channel_data[ch_num].frep_sum = channel_data[ch_num].frep_sum  + frep;
			
			if(duty > channel_data[ch_num].duty_max.uint16)
				channel_data[ch_num].duty_max.uint16 = duty;
			if(duty < channel_data[ch_num].duty_min.uint16)
				channel_data[ch_num].duty_min.uint16 = duty;				
			channel_data[ch_num].duty_sum = channel_data[ch_num].duty_sum + duty;
			
			if(phase_diff > channel_data[ch_num].phase_diff_max.uint32)
				channel_data[ch_num].phase_diff_max.uint32 = phase_diff; // ？
			if(phase_diff < channel_data[ch_num].phase_diff_min.uint32)
				channel_data[ch_num].phase_diff_min.uint32 = phase_diff;				
			channel_data[ch_num].phase_diff_sum = channel_data[ch_num].phase_diff_sum + phase_diff;

			channel_data[ch_num].calculated_period_num++;	//mean 平均数
			channel_data[ch_num].period_mean.uint32 = (uint32)(channel_data[ch_num].period_sum / channel_data[ch_num].calculated_period_num);
			channel_data[ch_num].duty_mean.uint16 = (uint16)(channel_data[ch_num].duty_sum / channel_data[ch_num].calculated_period_num);
			channel_data[ch_num].frep_mean.uint32 = (uint32)(channel_data[ch_num].frep_sum / channel_data[ch_num].calculated_period_num);
			channel_data[ch_num].phase_diff_mean.uint32 = (uint32)(channel_data[ch_num].phase_diff_sum / channel_data[ch_num].calculated_period_num);	
	
		}
		else if((channel_data[ch_num].edge_cnt_readout % 2 == 1) && (channel_data[ch_num].edge_cnt_readout >= 4))
		{
			tmp1 = tmp3;
			tmp2 = data;	
		}						
//	}				
}

void cpld_data_process( void )
{
	uint16 ch_num;
	uint32 i, j, addr_cnt=0;	//addr_cnt 字节计数变量
	union u32_bytes cpld_data;

	for(i=0;i<0x20000;i++)
	{
		for(j=0;j<4;j++)
		{
			MCU_RD_CLK_L;
			cpld_data._byte[j] = GPIO_ReadInputData(GPIOE) & 0x00ff;
			MCU_RD_CLK_H;	
			addr_cnt++;		
		}	

		for(ch_num = 0; ch_num < TEST_CHANNEL_MAX; ch_num++)//check which channel the current data belongs to
		{
			if((addr_cnt - 4) >= ch_base_addr[ch_num] 
				&& (addr_cnt - 4) < (ch_base_addr[ch_num ] + channel_data[ch_num].edge_num_per_cycle.uint16 * 4))//基地址 + 偏移地址数
			{
				channel_data[ch_num].edge_cnt_readout++;
				calc_cpld_data(cpld_data._u32, ch_num);
				
				if(channel_data[ch_num].edge_cnt_readout < 30)
				{
					data_buf[ch_num][channel_data[ch_num].edge_cnt_readout - 1] = cpld_data._u32;
				}
				if(ch_num == 9)
				{
					A_buf[channel_data[ch_num].edge_cnt_readout - 1] = cpld_data._u32;
				}
				if(ch_num == 11)
				{
					B_buf[channel_data[ch_num].edge_cnt_readout - 1] = cpld_data._u32;
				}
				break;
			}				
		}		
	}	
}

void calc_dft(void)
{
	uint16 i, j;
	for(i=0;i<3;i++)
	{
//		memcpy((uint8 *)dft_buf[i], (uint8 *)channel_wave[i], 512*2);
		for(j=0;j<512;j++)
		{
			dft_buf[i][j] = (short)channel_wave[i][j]-(short)channel_data[i].vol_zero_cross.uint16;
		}
		dft_f(dft_buf[i], dft_output[i]);
	}
}
void reset_channel_data( void ) 
{
  uint8 i;
	sys_para.status.bits.wave_sample_end = 0;
	memset((void *)A_buf, 0, sizeof(A_buf));
	memset((void *)B_buf, 0, sizeof(B_buf));
	memset((void *)data_buf, 0, sizeof(data_buf));
	memset((void *)uvwz_edge_phase_buf, 0, sizeof(uvwz_edge_phase_buf));
	
	for(i = 0; i < TEST_CHANNEL_MAX; i++)
	{
		channel_data[i].vol_rms.uint16 = 0;//rms 均方根
		channel_data[i].vol_high_max.uint16 = 0;
		channel_data[i].vol_high_min.uint16 = 0;
		channel_data[i].vol_high_mean.uint16 = 0;
		channel_data[i].vol_low_max.uint16 = 0;
		channel_data[i].vol_low_min.uint16 = 0;
		channel_data[i].vol_low_mean.uint16 = 0;
		channel_data[i].pulse_cnt.uint16 = 0;//pulse脉冲
		channel_data[i].edge_cnt_readout = 0;	
		channel_data[i].edge_num_per_cycle.uint16 = 0;	
		channel_data[i].clk_num_per_cycle.uint32 = 0;	
		channel_data[i].calculated_period_num = 0;
		
		channel_data[i].z_phase_diff.uint32 = 0;
		channel_data[i].z_width.uint32 = 0;
		channel_data[i].wave_length = 0;
		channel_data[i].zerocnt = 0;
							
		channel_data[i].period_max.uint32 = 0;
		channel_data[i].period_mean.uint32 = 0;
		channel_data[i].period_min.uint32 = 0xffffffff;	
		channel_data[i].period_sum = 0;
		
		channel_data[i].duty_max.uint16 = 0;
		channel_data[i].duty_mean.uint16 = 0;
		channel_data[i].duty_min.uint16 = 0xffff;
		channel_data[i].duty_sum = 0;
		
		channel_data[i].frep_max.uint32 = 0;
		channel_data[i].frep_mean.uint32 = 0;
		channel_data[i].frep_min.uint32 = 0xffffffff;	
		channel_data[i].frep_sum = 0;		
		
		channel_data[i].phase_diff_max.uint32 = 0;
		channel_data[i].phase_diff_mean.uint32 = 0;
		channel_data[i].phase_diff_min.uint32 = 0xffffffff;	
		channel_data[i].phase_diff_sum = 0;	
	}		
}

void start_cpld_test( void )
{
	MCU_RD_CLK_H;
	MCU_START_L;
	MCU_N_RST_L;
	MCU_N_RST_H;
	MCU_START_H;
	MCU_START_L;	
	sys_para.status.bits.sample_en = 1;	
}

void send_debug_data(void)
{	
	u16 i;

// 		usart_send_frame(1, (uint8*)&A_buf[0], 250*4);		
// 		vTaskDelay(1500/portTICK_RATE_MS);
// 		usart_send_frame(1, (uint8*)&A_buf[250], 250*4);		
// 		vTaskDelay(1500/portTICK_RATE_MS);
// 		usart_send_frame(1, (uint8*)&A_buf[500], 250*4);		
// 		vTaskDelay(1500/portTICK_RATE_MS);
// 		usart_send_frame(1, (uint8*)&A_buf[750], 250*4);		
// 		vTaskDelay(1500/portTICK_RATE_MS);
// 		usart_send_frame(1, (uint8*)&A_buf[1000], 250*4);		
// 		vTaskDelay(1500/portTICK_RATE_MS);
// 		usart_send_frame(1, (uint8*)&A_buf[1250], 250*4);		
// 		vTaskDelay(1500/portTICK_RATE_MS);
// 		usart_send_frame(1, (uint8*)&A_buf[1500], 250*4);		
// 		vTaskDelay(1500/portTICK_RATE_MS);
// 		usart_send_frame(1, (uint8*)&A_buf[1750], 250*4);		
// 		vTaskDelay(1500/portTICK_RATE_MS);
// 		usart_send_frame(1, (uint8*)&A_buf[2000], 100*4);		
// 		vTaskDelay(1500/portTICK_RATE_MS);

// 		
// 		usart_send_frame(1, (uint8*)&B_buf[0], 250*4);		
// 		vTaskDelay(1500/portTICK_RATE_MS);
// 		usart_send_frame(1, (uint8*)&B_buf[250], 250*4);		
// 		vTaskDelay(1500/portTICK_RATE_MS);
// 		usart_send_frame(1, (uint8*)&B_buf[500], 250*4);		
// 		vTaskDelay(1500/portTICK_RATE_MS);
// 		usart_send_frame(1, (uint8*)&B_buf[750], 250*4);		
// 		vTaskDelay(1500/portTICK_RATE_MS);
// 		usart_send_frame(1, (uint8*)&B_buf[1000], 250*4);		
// 		vTaskDelay(1500/portTICK_RATE_MS);
// 		usart_send_frame(1, (uint8*)&B_buf[1250], 250*4);		
// 		vTaskDelay(1500/portTICK_RATE_MS);
// 		usart_send_frame(1, (uint8*)&B_buf[1500], 250*4);		
// 		vTaskDelay(1500/portTICK_RATE_MS);
// 		usart_send_frame(1, (uint8*)&B_buf[1750], 250*4);		
// 		vTaskDelay(1500/portTICK_RATE_MS);
// 		usart_send_frame(1, (uint8*)&B_buf[2000], 100*4);		
// 		vTaskDelay(1500/portTICK_RATE_MS);
		
	for(i=0;i<TEST_CHANNEL_MAX;i++)
	{
		usart_send_frame(1, (uint8*)&channel_data[i].edge_num_per_cycle.uint16, 2);	
		vTaskDelay(10/portTICK_RATE_MS);					
		usart_send_frame(1, (uint8*)&channel_data[i].clk_num_per_cycle.uint32, 4);	
		vTaskDelay(10/portTICK_RATE_MS);

		usart_send_frame(1, (uint8*)&channel_data[i].vol_rms.uint16, 2);	
		vTaskDelay(10/portTICK_RATE_MS);	
		usart_send_frame(1, (uint8*)&channel_data[i].vol_zero_cross_tmp.uint16, 2);	
		vTaskDelay(10/portTICK_RATE_MS);	
		usart_send_frame(1, (uint8*)&channel_data[i].zerocnt, 2);	
		vTaskDelay(10/portTICK_RATE_MS);	
		usart_send_frame(1, (uint8*)&channel_data[i].zeropos[channel_data[i].zerocnt - 1], 2);	
		vTaskDelay(10/portTICK_RATE_MS);	
		usart_send_frame(1, (uint8*)&channel_data[i].z_phase_diff.uint32, 4);	
		vTaskDelay(10/portTICK_RATE_MS);
		
		usart_send_frame(1, (uint8*)&channel_data[i].period_max.uint32, 4);	
		vTaskDelay(10/portTICK_RATE_MS);		
		usart_send_frame(1, (uint8*)&channel_data[i].period_min.uint32, 4);	
		vTaskDelay(10/portTICK_RATE_MS);		
		usart_send_frame(1, (uint8*)&channel_data[i].period_mean.uint32, 4);	
		vTaskDelay(300/portTICK_RATE_MS);		
		
	}		
		
	for(i=0;i<TEST_CHANNEL_MAX;i++)
	{
		usart_send_frame(1, (uint8*)data_buf[i], 100);		
		vTaskDelay(400/portTICK_RATE_MS);
	}
	for(i=0;i<9;i++)
	{
		usart_send_frame(1, (uint8*)channel_wave[i], 200);		
		vTaskDelay(400/portTICK_RATE_MS);						
	}
	for(i=0;i<3;i++)
	{
		usart_send_frame(1, (uint8*)channel_data[i].zeropos, 40);		
		vTaskDelay(400/portTICK_RATE_MS);						
	}
// 	for(i=0;i<3;i++)
// 	{
// 		usart_send_frame(1, (uint8*)dft_output[i], 512*2);		
// 		vTaskDelay(500/portTICK_RATE_MS);						
// 	}
// 	for(i=0;i<6;i++)
// 	{
// 		usart_send_frame(1, (uint8*)adc_tmp[i], 512);		
// 		vTaskDelay(500/portTICK_RATE_MS);						
// 	}
}

void calc_uvw_edge_phase(void)
{
	uint8 i;
	
	for(i=0;i<20;i++)
	{
		if((data_buf[3][0]&0x7fffffff) <= 0x0F)
		{
			uvwz_edge_phase_buf[0][i].uint16 = (u16)(((double)((data_buf[3][i+1]&0x7fffffff)*1.0)/(double)channel_data[3].clk_num_per_cycle.uint32*1.0)*360*100);
		}
		else
		{
			uvwz_edge_phase_buf[0][i].uint16 = (u16)(((double)((data_buf[3][i]&0x7fffffff)*1.0)/(double)channel_data[3].clk_num_per_cycle.uint32*1.0)*360*100);
		}			
	}

	for(i=0;i<20;i++)
	{
		if((data_buf[5][0]&0x7fffffff) <= 0x0F)
		{
			uvwz_edge_phase_buf[1][i].uint16 = (u16)(((double)((data_buf[5][i+1]&0x7fffffff)*1.0)/(double)channel_data[5].clk_num_per_cycle.uint32*1.0)*360*100);
		}
		else
		{
			uvwz_edge_phase_buf[1][i].uint16 = (u16)(((double)((data_buf[5][i]&0x7fffffff)*1.0)/(double)channel_data[5].clk_num_per_cycle.uint32*1.0)*360*100);			
		}
	}

	for(i=0;i<20;i++)
	{
		if((data_buf[7][0]&0x7fffffff) <= 0x0F)
		{
			uvwz_edge_phase_buf[2][i].uint16 = (u16)(((double)((data_buf[7][i+1]&0x7fffffff)*1.0)/(double)channel_data[7].clk_num_per_cycle.uint32*1.0)*360*100);
		}
		else
		{
			uvwz_edge_phase_buf[2][i].uint16 = (u16)(((double)((data_buf[7][i]&0x7fffffff)*1.0)/(double)channel_data[7].clk_num_per_cycle.uint32*1.0)*360*100);			
		}
	}	
	for(i=0;i<20;i++)
	{
		if((data_buf[8][0]&0x7fffffff) <= 0x0F)
		{
			uvwz_edge_phase_buf[3][i].uint16 = (u16)(((double)((data_buf[8][i+1]&0x7fffffff)*1.0)/(double)channel_data[8].clk_num_per_cycle.uint32*1.0)*360*100);
		}
		else
		{
			uvwz_edge_phase_buf[3][i].uint16 = (u16)(((double)((data_buf[8][i]&0x7fffffff)*1.0)/(double)channel_data[8].clk_num_per_cycle.uint32*1.0)*360*100);			
		}
	}	
	// qj Z相位角计算
	for(i=0;i<20;i++)
	{
		if((data_buf[13][0]&0x7fffffff) <= 0x0F)
		{
			uvwz_edge_phase_buf[5][i].uint16 = (u16)(((double)((data_buf[13][i+1]&0x7fffffff)*1.0)/(double)channel_data[13].clk_num_per_cycle.uint32*1.0)*360*100);
		}
		else
		{
			uvwz_edge_phase_buf[5][i].uint16 = (u16)(((double)((data_buf[13][i]&0x7fffffff)*1.0)/(double)channel_data[13].clk_num_per_cycle.uint32*1.0)*360*100);			
		}
	}	
	if((data_buf[3][0]&0x7fffffff) <= 0x0F)
	{
		channel_data[3].edge_num_per_cycle.uint16--;
	}
	if((data_buf[5][0]&0x7fffffff) <= 0x0F)
	{
		channel_data[5].edge_num_per_cycle.uint16--;
	}
	if((data_buf[7][0]&0x7fffffff) <= 0x0F)
	{
		channel_data[7].edge_num_per_cycle.uint16--;
	}
	
	if((data_buf[8][0]&0x7fffffff) <= 0x0F && channel_data[8].edge_num_per_cycle.uint16<=3)
	{
		channel_data[8].edge_num_per_cycle.uint16=1;
	}
	// qj Z边沿计算
	if((data_buf[13][0] & 0x7fffffff) <= 0x0F)
	{
		channel_data[13].edge_num_per_cycle.uint16 --;
		channel_data[13].edge_num_per_cycle.uint16 = channel_data[13].edge_num_per_cycle.uint16 / 2;
	}
}


static portTASK_FUNCTION( cpld_test_task, pvParameters )
{
	uint16 timeout;
	
	while(1)
    {    		 
			xSemaphoreTake( read_cpld, portMAX_DELAY );		
			reset_channel_data();
			start_cpld_test();
			sys_para.status.bits.sys_state = STATE_TEST_ONGOING;
			for(timeout=0;timeout<500;timeout++)
			{
				vTaskDelay(10/portTICK_RATE_MS);
				if( MCU_END == 1 )
				{
					sys_para.status.bits.sample_en = 0;
					cpld_data_process();
 					calc_uvw_vol_rms();	//计算方差 判断缺相？
//					calc_dft();
					calc_uvw_edge_phase();	//计算 边沿数和相位角
					sys_para.status.bits.sys_state = STATE_TEST_OVER;
					break;
				}
			}	
			if(timeout >= 500)
			{
				sys_para.status.bits.sys_state = STATE_TEST_FAIL;				
			}
    }    
}

void start_cpld_test_task( unsigned portBASE_TYPE uxPriority )
{
    
	/* Spawn the task. */
	xTaskCreate(cpld_test_task, ( const signed portCHAR * const ) "cpld_test", CPLD_TEST_STACK_SIZE, NULL, uxPriority, ( xTaskHandle * ) NULL );
}


