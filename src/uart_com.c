
/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "config.h"
#include "public.h"

/* Library includes. */
#include "stm32f10x_conf.h"
#include "serial.h"
#include "encoder_meas.h"
#include "adc.h"
#include <string.h>
#include "dft.h"

#define CMD_START    0x00
#define CMD_END      0x01
#define CMD_RD_STATE 0x02
#define CMD_RD_CH    0x03
#define CMD_RD_WAVE  0x04
#define CMD_RD_PHASE 0x05
#define CMD_RD_DFT   0x06
#define CMD_GET_ZERO 0x07
#define CMD_DEBUG    0x08
#define CMD_ORG      0x09
#define CMD_RD_EDGE_PHASE  0x0a

uint8 rx_buf[100];
uint8 tx_buf[1000];

extern short dft_output[3][512];
extern tMEM16 uvwz_edge_phase_buf[5][30];

uint8 validate_check_rx_data(uint8* rx_buf)
{
	uint8 i, check_sum=0;
	uint8 byte_cnt;
	
	byte_cnt = *(rx_buf+1);
	for(i=0;i<(byte_cnt-3);i++)
	{
		check_sum +=rx_buf[i+1];
	}
	
	if((check_sum == rx_buf[byte_cnt-2])&&(rx_buf[byte_cnt-1] == 0x7D))
	{
		if((*(rx_buf+3) ==  CMD_START) && (*(rx_buf+1) == 0x06))
			return 1;
		else if((*(rx_buf+3) ==  CMD_END) && (*(rx_buf+1) == 0x06))
			return 1;
		else if((*(rx_buf+3) ==  CMD_RD_STATE) && (*(rx_buf+1) == 0x06))
			return 1;
		else if((*(rx_buf+3) ==  CMD_RD_CH) && (*(rx_buf+1) == 0x07))
			return 1;
		else if((*(rx_buf+3) ==  CMD_RD_WAVE) && (*(rx_buf+1) == 0x07))
			return 1;
		else if((*(rx_buf+3) ==  CMD_RD_PHASE) && (*(rx_buf+1) == 0x06))
			return 1;
		else if((*(rx_buf+3) ==  CMD_RD_DFT) && (*(rx_buf+1) == 0x07))
			return 1;
		else if((*(rx_buf+3) ==  CMD_GET_ZERO) && (*(rx_buf+1) == 0x06))
			return 1;
		else if((*(rx_buf+3) ==  CMD_DEBUG) && (*(rx_buf+1) == 0x06))
			return 1;
		else if((*(rx_buf+3) ==  CMD_ORG) && (*(rx_buf+1) == 0x06))
			return 1;		
		else if((*(rx_buf+3) ==  CMD_RD_EDGE_PHASE) && (*(rx_buf+1) == 0x07))
			return 1;	
		else
			return 0;		
	}
	else
		return 0;
}

uint8 get_check_sum(uint8* buf, uint32 cnt)
{
	uint32 i;
	uint8 check_sum=0;
	
	for(i=0;i<cnt;i++)
	{
		check_sum += buf[i];
	}
	return check_sum;
}

void read_channel_data(uint8* rx_buf, uint8* tx_buf, uint32* len)
{
	uint8 ch_num;	
//	tMEM16 period_ratio;

	ch_num = *(rx_buf + 4);	
	if(ch_num >= 15)
	return;
	
	channel_data[ch_num].pulse_cnt.uint16 = channel_data[ch_num].edge_num_per_cycle.uint16 / 2;	
//	period_ratio.uint16 = (u16)(((double)((channel_data[ch_num].period_max.uint32 - channel_data[ch_num].period_min.uint32)*1.0) / (channel_data[ch_num].period_mean.uint32*1.0))*10000);
	
	*tx_buf = 0x7B;
	*(tx_buf+1) = 63;
	*(tx_buf+2) = *(rx_buf+2);
	*(tx_buf+3) = *(rx_buf+3);
	*(tx_buf+4) = *(rx_buf+4);
	*(tx_buf+5) = channel_data[ch_num].pulse_cnt.uint8._0;
	*(tx_buf+6) = channel_data[ch_num].pulse_cnt.uint8._1;
	*(tx_buf+7) = channel_data[ch_num].vol_zero_cross.uint8._0;
	*(tx_buf+8) = channel_data[ch_num].vol_zero_cross.uint8._1;
	*(tx_buf+9) = channel_data[ch_num].vol_rms.uint8._0;
	*(tx_buf+10) = channel_data[ch_num].vol_rms.uint8._1;		
	*(tx_buf+11) = channel_data[ch_num].vol_high_max.uint8._0;
	*(tx_buf+12) = channel_data[ch_num].vol_high_max.uint8._1;
	*(tx_buf+13) = channel_data[ch_num].vol_high_min.uint8._0;
	*(tx_buf+14) = channel_data[ch_num].vol_high_min.uint8._1;
	*(tx_buf+15) = channel_data[ch_num].vol_high_mean.uint8._0;
	*(tx_buf+16) = channel_data[ch_num].vol_high_mean.uint8._1;
	*(tx_buf+17) = channel_data[ch_num].vol_low_max.uint8._0;
	*(tx_buf+18) = channel_data[ch_num].vol_low_max.uint8._1;
	*(tx_buf+19) = channel_data[ch_num].vol_low_min.uint8._0;
	*(tx_buf+20) = channel_data[ch_num].vol_low_min.uint8._1;
	*(tx_buf+21) = channel_data[ch_num].vol_low_mean.uint8._0;
	*(tx_buf+22) = channel_data[ch_num].vol_low_mean.uint8._1;
	*(tx_buf+23) = channel_data[ch_num].clk_num_per_cycle.uint8._0;
	*(tx_buf+24) = channel_data[ch_num].clk_num_per_cycle.uint8._1;
	*(tx_buf+25) = channel_data[ch_num].clk_num_per_cycle.uint8._2;
	*(tx_buf+26) = channel_data[ch_num].clk_num_per_cycle.uint8._3;	
	
// 	if((channel_data[ch_num].vol_high_max.uint16 < 0x400)&&(ch_num>=3))
// 	{
// 		*(tx_buf+27) = 0;
// 		*(tx_buf+28) = 0;
// 		*(tx_buf+29) = 0;
// 		*(tx_buf+30) = 0;
// 		*(tx_buf+31) = 0;
// 		*(tx_buf+32) = 0;
// 		*(tx_buf+33) = 0;
// 		*(tx_buf+34) = 0;
// 		*(tx_buf+35) = 0;
// 		*(tx_buf+36) = 0;
// 		*(tx_buf+37) = 0;
// 		*(tx_buf+38) = 0;
// 		*(tx_buf+39) = 0;
// 		*(tx_buf+40) = 0;
// 		*(tx_buf+41) = 0;
// 		*(tx_buf+42) = 0;
// 		*(tx_buf+43) = 0;
// 		*(tx_buf+44) = 0;
// 		*(tx_buf+45) = 0;
// 		*(tx_buf+46) = 0;
// 		*(tx_buf+47) = 0;
// 		*(tx_buf+48) = 0;
// 		*(tx_buf+49) = 0;
// 		*(tx_buf+50) = 0;
// 		*(tx_buf+51) = 0;
// 		*(tx_buf+52) = 0;
// 		*(tx_buf+53) = 0;
// 		*(tx_buf+54) = 0;
// 		*(tx_buf+55) = 0;
// 		*(tx_buf+56) = 0;	
// 	}
// 	else
// 	{
		*(tx_buf+27) = channel_data[ch_num].period_max.uint8._0;
		*(tx_buf+28) = channel_data[ch_num].period_max.uint8._1;
		*(tx_buf+29) = channel_data[ch_num].period_max.uint8._2;
		*(tx_buf+30) = channel_data[ch_num].period_max.uint8._3;
		*(tx_buf+31) = channel_data[ch_num].period_min.uint8._0;
		*(tx_buf+32) = channel_data[ch_num].period_min.uint8._1;
		*(tx_buf+33) = channel_data[ch_num].period_min.uint8._2;
		*(tx_buf+34) = channel_data[ch_num].period_min.uint8._3;
		*(tx_buf+35) = channel_data[ch_num].period_mean.uint8._0;
		*(tx_buf+36) = channel_data[ch_num].period_mean.uint8._1;
		*(tx_buf+37) = channel_data[ch_num].period_mean.uint8._2;
		*(tx_buf+38) = channel_data[ch_num].period_mean.uint8._3;
		*(tx_buf+39) = channel_data[ch_num].frep_max.uint8._0;
		*(tx_buf+40) = channel_data[ch_num].frep_max.uint8._1;
		*(tx_buf+41) = channel_data[ch_num].frep_max.uint8._2;
		*(tx_buf+42) = channel_data[ch_num].frep_max.uint8._3;
		*(tx_buf+43) = channel_data[ch_num].frep_min.uint8._0;
		*(tx_buf+44) = channel_data[ch_num].frep_min.uint8._1;
		*(tx_buf+45) = channel_data[ch_num].frep_min.uint8._2;
		*(tx_buf+46) = channel_data[ch_num].frep_min.uint8._3;
		*(tx_buf+47) = channel_data[ch_num].frep_mean.uint8._0;
		*(tx_buf+48) = channel_data[ch_num].frep_mean.uint8._1;
		*(tx_buf+49) = channel_data[ch_num].frep_mean.uint8._2;
		*(tx_buf+50) = channel_data[ch_num].frep_mean.uint8._3;
		*(tx_buf+51) = channel_data[ch_num].duty_max.uint8._0;
		*(tx_buf+52) = channel_data[ch_num].duty_max.uint8._1;
		*(tx_buf+53) = channel_data[ch_num].duty_min.uint8._0;
		*(tx_buf+54) = channel_data[ch_num].duty_min.uint8._1;
		*(tx_buf+55) = channel_data[ch_num].duty_mean.uint8._0;
		*(tx_buf+56) = channel_data[ch_num].duty_mean.uint8._1;
// 	}
	*(tx_buf+57) = channel_data[ch_num].z_width.uint8._0;
	*(tx_buf+58) = channel_data[ch_num].z_width.uint8._1;
	*(tx_buf+59) = channel_data[ch_num].z_width.uint8._2;
	*(tx_buf+60) = channel_data[ch_num].z_width.uint8._3;
	
	*(tx_buf+61) = get_check_sum((tx_buf+1),60);
	*(tx_buf+62) = 0x7D;	
	*len = *(tx_buf+1);
	
	usart_send_frame(1, tx_buf, *len);
}

/*****************************************************************
u:U-V
v:V-W
w:W-U
*******************************************************************/
void read_phase(uint8* rx_buf, uint8* tx_buf, uint32* len)
{
	tMEM16 uv_vw_phase_max, uv_vw_phase_min, uv_vw_phase_mean;
	tMEM16 uv_wu_phase_max, uv_wu_phase_min, uv_wu_phase_mean;
	tMEM16 vw_wu_phase_max, vw_wu_phase_min, vw_wu_phase_mean;
	
	tMEM16 h1_h2_phase_max, h1_h2_phase_min, h1_h2_phase_mean;
	tMEM16 h1_h3_phase_max, h1_h3_phase_min, h1_h3_phase_mean;
	tMEM16 h2_h3_phase_max, h2_h3_phase_min, h2_h3_phase_mean;
	
	tMEM16 uv_h1_phase_max, uv_h1_phase_min, uv_h1_phase_mean;
	tMEM16 vw_h2_phase_max, vw_h2_phase_min, vw_h2_phase_mean;
	tMEM16 wu_h3_phase_max, wu_h3_phase_min, wu_h3_phase_mean;
	
	tMEM16 a_b_phase_max, a_b_phase_min, a_b_phase_mean;
	tMEM16 z_uv_phase, z_vw_phase, z_wu_phase;
	tMEM16 z_h1_phase, z_h2_phase, z_h3_phase;
	tMEM16 z_a_phase, z_b_phase;
	tMEM16 z_bianwei_phase;

	uv_vw_phase_max.uint16 = (u16)(((double)(channel_data[15].phase_diff_max.uint32*1.0)/(double)channel_data[0].period_mean.uint32*1.0)*360*100);
	uv_vw_phase_min.uint16 = (u16)(((double)(channel_data[15].phase_diff_min.uint32*1.0)/(double)channel_data[0].period_mean.uint32*1.0)*360*100);
	uv_vw_phase_mean.uint16 = (u16)(((double)(channel_data[15].phase_diff_max.uint32*1.0)/(double)channel_data[0].period_mean.uint32*1.0)*360*100);
	uv_wu_phase_max.uint16 = (u16)(((double)(channel_data[16].phase_diff_max.uint32*1.0)/(double)channel_data[0].period_mean.uint32*1.0)*360*100);
	uv_wu_phase_min.uint16 = (u16)(((double)(channel_data[16].phase_diff_min.uint32*1.0)/(double)channel_data[0].period_mean.uint32*1.0)*360*100);
	uv_wu_phase_mean.uint16 = (u16)(((double)(channel_data[16].phase_diff_max.uint32*1.0)/(double)channel_data[0].period_mean.uint32*1.0)*360*100);
	vw_wu_phase_max.uint16 = (u16)(((double)(channel_data[17].phase_diff_max.uint32*1.0)/(double)channel_data[1].period_mean.uint32*1.0)*360*100);
	vw_wu_phase_min.uint16 = (u16)(((double)(channel_data[17].phase_diff_min.uint32*1.0)/(double)channel_data[1].period_mean.uint32*1.0)*360*100);
	vw_wu_phase_mean.uint16 = (u16)(((double)(channel_data[17].phase_diff_max.uint32*1.0)/(double)channel_data[1].period_mean.uint32*1.0)*360*100);

	h1_h2_phase_max.uint16 = (u16)(((double)(channel_data[18].phase_diff_max.uint32*1.0)/(double)channel_data[3].period_mean.uint32*1.0)*360*100);
	h1_h2_phase_min.uint16 = (u16)(((double)(channel_data[18].phase_diff_min.uint32*1.0)/(double)channel_data[3].period_mean.uint32*1.0)*360*100);
	h1_h2_phase_mean.uint16 = (u16)(((double)(channel_data[18].phase_diff_max.uint32*1.0)/(double)channel_data[3].period_mean.uint32*1.0)*360*100);
	h1_h3_phase_max.uint16 = (u16)(((double)(channel_data[19].phase_diff_max.uint32*1.0)/(double)channel_data[3].period_mean.uint32*1.0)*360*100);
	h1_h3_phase_min.uint16 = (u16)(((double)(channel_data[19].phase_diff_min.uint32*1.0)/(double)channel_data[3].period_mean.uint32*1.0)*360*100);
	h1_h3_phase_mean.uint16 = (u16)(((double)(channel_data[19].phase_diff_max.uint32*1.0)/(double)channel_data[3].period_mean.uint32*1.0)*360*100);
	h2_h3_phase_max.uint16 = (u16)(((double)(channel_data[20].phase_diff_max.uint32*1.0)/(double)channel_data[5].period_mean.uint32*1.0)*360*100);
	h2_h3_phase_min.uint16 = (u16)(((double)(channel_data[20].phase_diff_min.uint32*1.0)/(double)channel_data[5].period_mean.uint32*1.0)*360*100);
	h2_h3_phase_mean.uint16 = (u16)(((double)(channel_data[20].phase_diff_max.uint32*1.0)/(double)channel_data[5].period_mean.uint32*1.0)*360*100);

	uv_h1_phase_max.uint16 = (u16)(((double)(channel_data[21].phase_diff_max.uint32*1.0)/(double)channel_data[3].period_mean.uint32*1.0)*360*100);
	uv_h1_phase_min.uint16 = (u16)(((double)(channel_data[21].phase_diff_min.uint32*1.0)/(double)channel_data[3].period_mean.uint32*1.0)*360*100);
	uv_h1_phase_mean.uint16 = (u16)(((double)(channel_data[21].phase_diff_max.uint32*1.0)/(double)channel_data[3].period_mean.uint32*1.0)*360*100);
	vw_h2_phase_max.uint16 = (u16)(((double)(channel_data[22].phase_diff_max.uint32*1.0)/(double)channel_data[5].period_mean.uint32*1.0)*360*100);
	vw_h2_phase_min.uint16 = (u16)(((double)(channel_data[22].phase_diff_min.uint32*1.0)/(double)channel_data[5].period_mean.uint32*1.0)*360*100);
	vw_h2_phase_mean.uint16 = (u16)(((double)(channel_data[22].phase_diff_max.uint32*1.0)/(double)channel_data[5].period_mean.uint32*1.0)*360*100);
	wu_h3_phase_max.uint16 = (u16)(((double)(channel_data[23].phase_diff_max.uint32*1.0)/(double)channel_data[7].period_mean.uint32*1.0)*360*100);
	wu_h3_phase_min.uint16 = (u16)(((double)(channel_data[23].phase_diff_min.uint32*1.0)/(double)channel_data[7].period_mean.uint32*1.0)*360*100);
	wu_h3_phase_mean.uint16 = (u16)(((double)(channel_data[23].phase_diff_max.uint32*1.0)/(double)channel_data[7].period_mean.uint32*1.0)*360*100);

	a_b_phase_max.uint16 = (u16)(((double)(channel_data[24].phase_diff_max.uint32*1.0)/(double)channel_data[9].period_mean.uint32*1.0)*360*100);
	a_b_phase_min.uint16 = (u16)(((double)(channel_data[24].phase_diff_min.uint32*1.0)/(double)channel_data[9].period_mean.uint32*1.0)*360*100);
	a_b_phase_mean.uint16 = (u16)(((double)(channel_data[24].phase_diff_max.uint32*1.0)/(double)channel_data[9].period_mean.uint32*1.0)*360*100);

//	z_uv_phase.uint16 = (u16)(((double)(channel_data[0].z_phase_diff.uint32*1.0)/(double)channel_data[0].period_mean.uint32*1.0)*360*100);
// 	z_vw_phase.uint16 = (u16)(((double)(channel_data[1].z_phase_diff.uint32*1.0)/(double)channel_data[1].period_mean.uint32*1.0)*360*100);
// 	z_wu_phase.uint16 = (u16)(((double)(channel_data[2].z_phase_diff.uint32*1.0)/(double)channel_data[2].period_mean.uint32*1.0)*360*100);
	z_uv_phase.uint16 = (u16)(((double)(channel_data[0].z_phase_diff.uint32*1.0)/(double)channel_data[0].clk_num_per_cycle.uint32*1.0)*360*100);
	z_vw_phase.uint16 = (u16)(((double)(channel_data[1].z_phase_diff.uint32*1.0)/(double)channel_data[1].clk_num_per_cycle.uint32*1.0)*360*100);
	z_wu_phase.uint16 = (u16)(((double)(channel_data[2].z_phase_diff.uint32*1.0)/(double)channel_data[2].clk_num_per_cycle.uint32*1.0)*360*100);

	z_h1_phase.uint16 = (u16)(((double)(channel_data[3].z_phase_diff.uint32*1.0)/(double)channel_data[3].period_mean.uint32*1.0)*360*100);
	z_h2_phase.uint16 = (u16)(((double)(channel_data[5].z_phase_diff.uint32*1.0)/(double)channel_data[5].period_mean.uint32*1.0)*360*100);
	z_h3_phase.uint16 = (u16)(((double)(channel_data[7].z_phase_diff.uint32*1.0)/(double)channel_data[7].period_mean.uint32*1.0)*360*100);
	
	z_a_phase.uint16 = (u16)(((double)(channel_data[9].z_phase_diff.uint32*1.0)/(double)channel_data[9].period_mean.uint32*1.0)*360*100);
	z_b_phase.uint16 = (u16)(((double)(channel_data[11].z_phase_diff.uint32*1.0)/(double)channel_data[11].period_mean.uint32*1.0)*360*100);
	
	z_bianwei_phase.uint16 = (u16)(((double)(channel_data[14].z_phase_diff.uint32*1.0)/(double)channel_data[14].clk_num_per_cycle.uint32*1.0)*360*100);

	*tx_buf = 0x7B;
	*(tx_buf+1) = 84;
	*(tx_buf+2) = *(rx_buf+2);
	*(tx_buf+3) = *(rx_buf+3);
	*(tx_buf+4) = uv_vw_phase_max.uint8._0;
	*(tx_buf+5) = uv_vw_phase_max.uint8._1;
	*(tx_buf+6) = uv_vw_phase_min.uint8._0;
	*(tx_buf+7) = uv_vw_phase_min.uint8._1;
	*(tx_buf+8) = uv_vw_phase_mean.uint8._0;
	*(tx_buf+9) = uv_vw_phase_mean.uint8._1;
	*(tx_buf+10) = uv_wu_phase_max.uint8._0;
	*(tx_buf+11) = uv_wu_phase_max.uint8._1;
	*(tx_buf+12) = uv_wu_phase_min.uint8._0;
	*(tx_buf+13) = uv_wu_phase_min.uint8._1;
	*(tx_buf+14) = uv_wu_phase_mean.uint8._0;
	*(tx_buf+15) = uv_wu_phase_mean.uint8._1;
	*(tx_buf+16) = vw_wu_phase_max.uint8._0;
	*(tx_buf+17) = vw_wu_phase_max.uint8._1;
	*(tx_buf+18) = vw_wu_phase_min.uint8._0;
	*(tx_buf+19) = vw_wu_phase_min.uint8._1;
	*(tx_buf+20) = vw_wu_phase_mean.uint8._0;
	*(tx_buf+21) = vw_wu_phase_mean.uint8._1;	
	
	*(tx_buf+22) = h1_h2_phase_max.uint8._0;
	*(tx_buf+23) = h1_h2_phase_max.uint8._1;	
	*(tx_buf+24) = h1_h2_phase_min.uint8._0;
	*(tx_buf+25) = h1_h2_phase_min.uint8._1;
	*(tx_buf+26) = h1_h2_phase_mean.uint8._0;
	*(tx_buf+27) = h1_h2_phase_mean.uint8._1;
	*(tx_buf+28) = h1_h3_phase_max.uint8._0;
	*(tx_buf+29) = h1_h3_phase_max.uint8._1;	
	*(tx_buf+30) = h1_h3_phase_min.uint8._0;
	*(tx_buf+31) = h1_h3_phase_min.uint8._1;
	*(tx_buf+32) = h1_h3_phase_mean.uint8._0;
	*(tx_buf+33) = h1_h3_phase_mean.uint8._1;
	*(tx_buf+34) = h2_h3_phase_max.uint8._0;
	*(tx_buf+35) = h2_h3_phase_max.uint8._1;	
	*(tx_buf+36) = h2_h3_phase_min.uint8._0;
	*(tx_buf+37) = h2_h3_phase_min.uint8._1;
	*(tx_buf+38) = h2_h3_phase_mean.uint8._0;
	*(tx_buf+39) = h2_h3_phase_mean.uint8._1;	

	*(tx_buf+40) = uv_h1_phase_max.uint8._0;
	*(tx_buf+41) = uv_h1_phase_max.uint8._1;	
	*(tx_buf+42) = uv_h1_phase_min.uint8._0;
	*(tx_buf+43) = uv_h1_phase_min.uint8._1;
	*(tx_buf+44) = uv_h1_phase_mean.uint8._0;
	*(tx_buf+45) = uv_h1_phase_mean.uint8._1;
	*(tx_buf+46) = vw_h2_phase_max.uint8._0;
	*(tx_buf+47) = vw_h2_phase_max.uint8._1;	
	*(tx_buf+48) = vw_h2_phase_min.uint8._0;
	*(tx_buf+49) = vw_h2_phase_min.uint8._1;
	*(tx_buf+50) = vw_h2_phase_mean.uint8._0;
	*(tx_buf+51) = vw_h2_phase_mean.uint8._1;	
	*(tx_buf+52) = wu_h3_phase_max.uint8._0;
	*(tx_buf+53) = wu_h3_phase_max.uint8._1;	
	*(tx_buf+54) = wu_h3_phase_min.uint8._0;
	*(tx_buf+55) = wu_h3_phase_min.uint8._1;
	*(tx_buf+56) = wu_h3_phase_mean.uint8._0;
	*(tx_buf+57) = wu_h3_phase_mean.uint8._1;
		
// 	if(channel_data[3].vol_high_max.uint16 < 0x400)
// 	{	
// 		*(tx_buf+22) = 0;
// 		*(tx_buf+23) = 0;	
// 		*(tx_buf+24) = 0;
// 		*(tx_buf+25) = 0;
// 		*(tx_buf+26) = 0;
// 		*(tx_buf+27) = 0;
// 		*(tx_buf+28) = 0;
// 		*(tx_buf+29) = 0;	
// 		*(tx_buf+30) = 0;
// 		*(tx_buf+31) = 0;
// 		*(tx_buf+32) = 0;
// 		*(tx_buf+33) = 0;

// 		*(tx_buf+40) = 0;
// 		*(tx_buf+41) = 0;	
// 		*(tx_buf+42) = 0;
// 		*(tx_buf+43) = 0;
// 		*(tx_buf+44) = 0;
// 		*(tx_buf+45) = 0;	
// 	}

// 	if(channel_data[5].vol_high_max.uint16 < 0x400)
// 	{	
// 		*(tx_buf+22) = 0;
// 		*(tx_buf+23) = 0;	
// 		*(tx_buf+24) = 0;
// 		*(tx_buf+25) = 0;
// 		*(tx_buf+26) = 0;
// 		*(tx_buf+27) = 0;
// 		*(tx_buf+34) = 0;
// 		*(tx_buf+35) = 0;	
// 		*(tx_buf+36) = 0;
// 		*(tx_buf+37) = 0;
// 		*(tx_buf+38) = 0;
// 		*(tx_buf+39) = 0;	

// 		*(tx_buf+46) = 0;
// 		*(tx_buf+47) = 0;	
// 		*(tx_buf+48) = 0;
// 		*(tx_buf+49) = 0;
// 		*(tx_buf+50) = 0;
// 		*(tx_buf+51) = 0;	
// 	}

// 	if(channel_data[7].vol_high_max.uint16 < 0x400)
// 	{	
// 		*(tx_buf+28) = 0;
// 		*(tx_buf+29) = 0;	
// 		*(tx_buf+30) = 0;
// 		*(tx_buf+31) = 0;
// 		*(tx_buf+32) = 0;
// 		*(tx_buf+33) = 0;
// 		*(tx_buf+34) = 0;
// 		*(tx_buf+35) = 0;	
// 		*(tx_buf+36) = 0;
// 		*(tx_buf+37) = 0;
// 		*(tx_buf+38) = 0;
// 		*(tx_buf+39) = 0;	

// 		*(tx_buf+52) = 0;
// 		*(tx_buf+53) = 0;	
// 		*(tx_buf+54) = 0;
// 		*(tx_buf+55) = 0;
// 		*(tx_buf+56) = 0;
// 		*(tx_buf+57) = 0;
// 	}
	
	*(tx_buf+58) = a_b_phase_max.uint8._0;
	*(tx_buf+59) = a_b_phase_max.uint8._1;	
	*(tx_buf+60) = a_b_phase_min.uint8._0;
	*(tx_buf+61) = a_b_phase_min.uint8._1;
	*(tx_buf+62) = a_b_phase_mean.uint8._0;
	*(tx_buf+63) = a_b_phase_mean.uint8._1;

// 	if((channel_data[9].vol_high_max.uint16 < 0x400)||(channel_data[11].vol_high_max.uint16 < 0x400))
// 	{	
// 		*(tx_buf+58) = 0;
// 		*(tx_buf+59) = 0;	
// 		*(tx_buf+60) = 0;
// 		*(tx_buf+61) = 0;
// 		*(tx_buf+62) = 0;
// 		*(tx_buf+63) = 0;				
// 	}	

	*(tx_buf+64) = z_uv_phase.uint8._0;
	*(tx_buf+65) = z_uv_phase.uint8._1;	
	*(tx_buf+66) = z_vw_phase.uint8._0;
	*(tx_buf+67) = z_vw_phase.uint8._1;
	*(tx_buf+68) = z_wu_phase.uint8._0;
	*(tx_buf+69) = z_wu_phase.uint8._1;

	*(tx_buf+70) = z_h1_phase.uint8._0;
	*(tx_buf+71) = z_h1_phase.uint8._1;	
	*(tx_buf+72) = z_h2_phase.uint8._0;
	*(tx_buf+73) = z_h2_phase.uint8._1;
	*(tx_buf+74) = z_h3_phase.uint8._0;
	*(tx_buf+75) = z_h3_phase.uint8._1;
	
	*(tx_buf+76) = z_a_phase.uint8._0;
	*(tx_buf+77) = z_a_phase.uint8._1;
	*(tx_buf+78) = z_b_phase.uint8._0;
	*(tx_buf+79) = z_b_phase.uint8._1;

	*(tx_buf+80) = z_bianwei_phase.uint8._0;
	*(tx_buf+81) = z_bianwei_phase.uint8._1;
	
	*(tx_buf+82) = get_check_sum((tx_buf+1),81);
	*(tx_buf+83) = 0x7D;	
	*len = *(tx_buf+1);

	usart_send_frame(1, tx_buf, *len);
}

void read_sys_status(uint8* rx_buf, uint8* tx_buf, uint32* len)
{
	*tx_buf = 0x7B;
	*(tx_buf+1) = 7;
	*(tx_buf+2) = *(rx_buf+2);
	*(tx_buf+3) = *(rx_buf+3);
	*(tx_buf+4) = sys_para.status.uint8;
	*(tx_buf+5) = get_check_sum((tx_buf+1),4);
	*(tx_buf+6) = 0x7D;	
	*len = *(tx_buf+1);
	
	usart_send_frame(1, tx_buf, *len);	
}

void send_ack(uint8* rx_buf, uint8* tx_buf, uint32* len)
{
	*tx_buf = 0x7B;
	*(tx_buf+1) = 6;
	*(tx_buf+2) = *(rx_buf+2);
	*(tx_buf+3) = *(rx_buf+3);
	*(tx_buf+4) = get_check_sum((tx_buf+1),3);
	*(tx_buf+5) = 0x7D;	
	*len = *(tx_buf+1);
	
	usart_send_frame(1, tx_buf, *len);	
}

void read_wave(uint8* rx_buf, uint8* tx_buf, uint32* len)
{
	uint8 ch_num;	
	uint16 i;
		
	ch_num = *(rx_buf + 4);
	if(ch_num>=9)
		return;

	*tx_buf = 0x7B;
	*(tx_buf+1) = 200+7;
	*(tx_buf+2) = *(rx_buf+2);
	*(tx_buf+3) = *(rx_buf+3);
	*(tx_buf+4) = *(rx_buf+4);	
	
//	memcpy((uint8*)(tx_buf+6), (uint8*)channel_wave[ch_num], 2*800);
	for(i=0;i<200;i++)
	{
		*(tx_buf+5+i) = (uint8)(channel_wave[ch_num][i]/16);
	}
	*(tx_buf+200+5) = get_check_sum((tx_buf+1),204);
	*(tx_buf+200+6) = 0x7D;
	
	usart_send_frame(1, tx_buf, 207);
}

void read_uvw_dft(uint8* rx_buf, uint8* tx_buf, uint32* len)
{
	uint8 ch_num, first_harmonic_num=0;	
	uint16 i, dft_tmp=0;
		
	ch_num = *(rx_buf + 4);
	if(ch_num>=3)
		return;

	for(i=0;i<512;i++)
	{
		if(dft_output[ch_num][i] > dft_tmp)
		{
			dft_tmp = dft_output[ch_num][i];
			first_harmonic_num = i;
		}
	}
	
	*tx_buf = 0x7B;
	*(tx_buf+1) = 48;
	*(tx_buf+2) = *(rx_buf+2);
	*(tx_buf+3) = *(rx_buf+3);
	*(tx_buf+4) = *(rx_buf+4);	
	*(tx_buf+5) = first_harmonic_num;	

	for(i=0;i<20;i++)//20th harmonic wave
	{
		*(tx_buf+6+2*i) = (uint8)dft_output[ch_num][first_harmonic_num*(i+1)];
		*(tx_buf+6+2*i+1) = (uint8)(dft_output[ch_num][first_harmonic_num*(i+1)]>>8);
	}
	
	*(tx_buf+40+6) = get_check_sum((tx_buf+1),45);
	*(tx_buf+40+7) = 0x7D;
	
	usart_send_frame(1, tx_buf, 48);
}


void save_zero_cross_vol(uint8* rx_buf, uint8* tx_buf, uint32* len)
{
	uint16 i;
	
	for(i=0;i<16;i++)
	{	
		channel_data[i].vol_zero_cross.uint16 = channel_data[i].vol_zero_cross_tmp.uint16;
	}
	send_ack(rx_buf, tx_buf, len);
}


void find_origin(uint8* rx_buf, uint8* tx_buf, uint32* len)
{
	uint16 timeout=0;
	
	for(timeout=0;timeout<2000;timeout++)
	{
		if(1 == GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12))
		{
			break;	
		}
		vTaskDelay(5/portTICK_RATE_MS);	
	}
	if(timeout>=2000)
		return;
	
	*tx_buf = 0x7B;
	*(tx_buf+1) = 6;
	*(tx_buf+2) = *(rx_buf+2);
	*(tx_buf+3) = *(rx_buf+3);
	*(tx_buf+4) = get_check_sum((tx_buf+1),*(tx_buf+1)-3);
	*(tx_buf+5) = 0x7D;	
	*len = *(tx_buf+1);	
	usart_send_frame(1, tx_buf, *len);		
}

void read_uvw_edge_phase(uint8* rx_buf, uint8* tx_buf, uint32* len)
{
	
	*tx_buf = 0x7B;
	*(tx_buf+1) = 7+2+40;
	*(tx_buf+2) = *(rx_buf+2);
	*(tx_buf+3) = *(rx_buf+3);
	*(tx_buf+4) = *(rx_buf+4);
// 	if(*(rx_buf+4) <= 3)  //qj 发送Z数据 （边沿数　、相位角 ）
	if((*(rx_buf+4) <= 2) || (*(rx_buf+4) = 5))
	{
		*(tx_buf+5) = channel_data[(*(rx_buf+4))*2+3].edge_num_per_cycle.uint8._0;
		*(tx_buf+6) = channel_data[(*(rx_buf+4))*2+3].edge_num_per_cycle.uint8._1;
		memcpy((void *)(tx_buf+7), (void *)uvwz_edge_phase_buf[*(rx_buf+4)], 40);
	}
	else
	{
		*(tx_buf+5) = channel_data[8].edge_num_per_cycle.uint8._0;
		*(tx_buf+6) = channel_data[8].edge_num_per_cycle.uint8._1;
		memcpy((void *)(tx_buf+7), (void *)uvwz_edge_phase_buf[*(rx_buf+4)], 40);				
	}
	*(tx_buf+47) = get_check_sum((tx_buf+1),*(tx_buf+1)-3);
	*(tx_buf+48) = 0x7D;	
	*len = *(tx_buf+1);	
	usart_send_frame(1, tx_buf, *len);		
}

void process_ack_frame(uint8* rx_buf, uint8* tx_buf)
{
	uint32 len;
	
	switch(rx_buf[3])
	{
		case CMD_START: 
							send_ack(rx_buf, tx_buf, &len);
							vTaskDelay(10/portTICK_RATE_MS);
							xSemaphoreGive( read_cpld );						
							break;
		case CMD_END: 
							send_ack(rx_buf, tx_buf, &len);
							break;
		case CMD_RD_STATE: 
							read_sys_status(rx_buf, tx_buf, &len);
							break;		
		case CMD_RD_CH:
							read_channel_data(rx_buf, tx_buf, &len);
							break;
		case CMD_RD_WAVE: 
							read_wave(rx_buf, tx_buf, &len);
							break;
		case CMD_RD_PHASE: 
							read_phase(rx_buf, tx_buf, &len);
							break;
		case CMD_RD_DFT: 
							read_uvw_dft(rx_buf, tx_buf, &len);
							break;
		case CMD_GET_ZERO: 
							save_zero_cross_vol(rx_buf, tx_buf, &len);
							break;
		case CMD_DEBUG: 
							send_debug_data();
							break;
		case CMD_ORG: 
							find_origin(rx_buf, tx_buf, &len);
							break;		
		case CMD_RD_EDGE_PHASE: 
							read_uvw_edge_phase(rx_buf, tx_buf, &len);
							break;	
		default: break;		
	}
}

/*-----------------------------------------------------------------------------------*/

/**
 * 下位机通讯任务.
 */
/*-----------------------------------------------------------------------------------*/
static portTASK_FUNCTION( uart_com_task, pvParameters )
{
	/* The parameters are not used. */
	( void ) pvParameters; 
	
	while(1)
	{	
		if(1 == usart_recv_frame(1, rx_buf))
		{
			if(1 == validate_check_rx_data(rx_buf))
			{
				process_ack_frame(rx_buf, tx_buf);				
			}
		}
	}
}

/*-----------------------------------------------------------------------------------*/
/**
 * 启动下位通讯任务函数.
 *
 * 
 *
 */
/*-----------------------------------------------------------------------------------*/
void start_uart_com_task( unsigned portBASE_TYPE uxPriority )
{
    
	/* Spawn the task. */
	xTaskCreate( uart_com_task, ( const signed portCHAR * const ) "UART_COM_TASK", UART_COM_STACK_SIZE, NULL, uxPriority, ( xTaskHandle * ) NULL );
}
//end of file
