
#include "config.h"
#include "user_api.h"
#include "public.h" //全局变量定义文件
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Library includes. */
#include "stm32f10x_conf.h"

#include "led.h"
#include "adc.h"
#include <math.h>
#include <string.h>
#include "serial.h"
#include "encoder_meas.h"
#include "delay.h"
#include "timer1.h"

xSemaphoreHandle ad_end;
volatile static u16 dma_buf[DMA_BUF_MAX];  
volatile uint16 channel_wave[9][WAVE_LEN_MAX];
u16 adc_tmp[6][257];

uint8_t adc_channel_select[]={
	ADC_Channel_0,
	ADC_Channel_1,
	ADC_Channel_2,
	ADC_Channel_3,
	ADC_Channel_4,
	ADC_Channel_5,
	ADC_Channel_6,
	ADC_Channel_7,
	ADC_Channel_8,
	ADC_Channel_9,
	ADC_Channel_10,
	ADC_Channel_11,
	ADC_Channel_12,
	ADC_Channel_13,
	ADC_Channel_14,
	ADC_Channel_15
};

uint8_t adc_sample_time[]={
	ADC_SampleTime_239Cycles5,
	ADC_SampleTime_239Cycles5,
	ADC_SampleTime_239Cycles5,
	ADC_SampleTime_239Cycles5,
	ADC_SampleTime_239Cycles5,
	ADC_SampleTime_239Cycles5,
	ADC_SampleTime_239Cycles5,
	ADC_SampleTime_239Cycles5,
	ADC_SampleTime_239Cycles5,
	ADC_SampleTime_71Cycles5,
	ADC_SampleTime_71Cycles5,
	ADC_SampleTime_71Cycles5,
	ADC_SampleTime_71Cycles5,
	ADC_SampleTime_71Cycles5,
	ADC_SampleTime_71Cycles5,
	ADC_SampleTime_71Cycles5
};

void adc1_rcc_init(void)
{
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE );
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC1,ENABLE );
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_DMA1,ENABLE );
}

void adc1_gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;   //模拟量输入

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;
	GPIO_Init(GPIOC,&GPIO_InitStructure);	
}

void adc1_dma_init(void)
{
	NVIC_InitTypeDef  NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr=ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr=(u32)&dma_buf;	
	DMA_InitStructure.DMA_DIR=DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize=ADC_CHANNEL_MAX;
	DMA_InitStructure.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc=DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode=DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority=DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M=DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1,&DMA_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;      
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE); 
	DMA_Cmd(DMA1_Channel1,ENABLE);

}

void adc1_init( void )
{
	ADC_InitTypeDef ADC_InitStructure;
	
	adc1_rcc_init();
	adc1_gpio_init();
	adc1_dma_init();

	RCC_ADCCLKConfig(RCC_PCLK2_Div6); //ADC时钟为   PCLK2/6=12MHz
	ADC_DeInit(ADC1);
	ADC_InitStructure.ADC_Mode=ADC_Mode_Independent;  //独立工作模式
	ADC_InitStructure.ADC_ScanConvMode=ENABLE;		  //SCAN OPEN
	ADC_InitStructure.ADC_ContinuousConvMode=DISABLE;  
	ADC_InitStructure.ADC_ExternalTrigConv=ADC_ExternalTrigConv_T1_CC1;//关外部触发启动
	ADC_InitStructure.ADC_DataAlign=ADC_DataAlign_Right;	 //转换数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel=ADC_CHANNEL_MAX;	   //设置进行规则转换的通道数，每次采集16个通道的过零点数据
	ADC_Init(ADC1,&ADC_InitStructure);

	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_1,2,ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_2,3,ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_3,4,ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_4,5,ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_5,6,ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_6,7,ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_7,8,ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_8,9,ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_9,10,ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_10,11,ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_11,12,ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_12,13,ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_13,14,ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_14,15,ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_15,16,ADC_SampleTime_239Cycles5);
	
	ADC_DMACmd(ADC1,ENABLE);
	ADC_ExternalTrigConvCmd(ADC1, ENABLE);//////////////////////
	ADC_Cmd(ADC1,ENABLE);
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
//	ADC_SoftwareStartConvCmd(ADC1,ENABLE); //开启转换 //使能或者失能指定的ADC的软件转换启动功能

	vSemaphoreCreateBinary( ad_end );
}


void adc1_dma_reinit(void)
{
	NVIC_InitTypeDef  NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	DMA_DeInit(DMA1_Channel1);
//	DMA_Cmd(DMA1_Channel1,DISABLE);
	DMA_InitStructure.DMA_PeripheralBaseAddr=ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr=(u32)&dma_buf;	
	DMA_InitStructure.DMA_DIR=DMA_DIR_PeripheralSRC;
	if(sys_para.ch_num < WAVE_CHANNEL_CNT)
	{
		DMA_InitStructure.DMA_BufferSize=WAVE_CHANNEL_CNT;
	}
	else
	{
		DMA_InitStructure.DMA_BufferSize=DMA_BUF_MAX;
	}
	DMA_InitStructure.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc=DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord;
 	DMA_InitStructure.DMA_Mode=DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority=DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M=DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1,&DMA_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;      
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE); 	
	DMA_Cmd(DMA1_Channel1,ENABLE);
}

void adc1_reinit(void)
{
	uint16 i;
	ADC_InitTypeDef ADC_InitStructure;

	if((sys_para.ch_num >= ADC_CHANNEL_MAX)||(sys_para.ch_num == 0))
		return;

	adc1_dma_reinit();
	ADC_DeInit(ADC1);
	ADC_InitStructure.ADC_Mode=ADC_Mode_Independent;  //独立工作模式
	ADC_InitStructure.ADC_ScanConvMode=ENABLE;	
	if(sys_para.ch_num < WAVE_CHANNEL_CNT)
	{		
		ADC_InitStructure.ADC_ContinuousConvMode=DISABLE;  
		ADC_InitStructure.ADC_ExternalTrigConv=ADC_ExternalTrigConv_T1_CC1;
	}
	else
	{
		ADC_InitStructure.ADC_ContinuousConvMode=ENABLE;  
		ADC_InitStructure.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;		
	}
	ADC_InitStructure.ADC_DataAlign=ADC_DataAlign_Right;	 //转换数据右对齐
	if(sys_para.ch_num < WAVE_CHANNEL_CNT)
	{
		ADC_InitStructure.ADC_NbrOfChannel = WAVE_CHANNEL_CNT;	   //设置进行规则转换的通道数
	}
	else
	{
		ADC_InitStructure.ADC_NbrOfChannel=1;
	}
	ADC_Init(ADC1,&ADC_InitStructure);
	
	if(sys_para.ch_num < WAVE_CHANNEL_CNT)
	{
		for(i=1;i<=WAVE_CHANNEL_CNT;i++)
		{
			ADC_RegularChannelConfig(ADC1,adc_channel_select[i-1],i,adc_sample_time[i-1]);//7us for 71.5 cycles and 21us for 239.5 cycles
		}
	}
	else
	{
		ADC_RegularChannelConfig(ADC1,adc_channel_select[sys_para.ch_num],1,adc_sample_time[sys_para.ch_num]);//7us for 71.5 cycles and 21us for 239.5 cycles
	}		

	ADC_DMACmd(ADC1,ENABLE);
	if(sys_para.ch_num < WAVE_CHANNEL_CNT)
	{
		ADC_ExternalTrigConvCmd(ADC1, ENABLE);
	}
	ADC_Cmd(ADC1,ENABLE);
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
	//	ADC_SoftwareStartConvCmd(ADC1,ENABLE); //instead of triggered by software, adc1 is triggered by timer1 here 
	
}


void DMA1_Channel1_IRQHandler(void)	 
{  
	long xHigherPriorityTaskWoken = pdFALSE;
		
	if(DMA_GetITStatus(DMA1_IT_TC1))
	{		
		DMA_ClearITPendingBit(DMA1_IT_GL1); //clear all interrupt flag	
		if((sys_para.ch_num >= WAVE_CHANNEL_CNT)&&(sys_para.status.bits.wave_sample_end == 1))
		{
			TIM_Cmd(TIM1, DISABLE);
			adc1_reinit();
		}
		xSemaphoreGiveFromISR( ad_end, &xHigherPriorityTaskWoken );
	}
	
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}


void bubble_sort( uint16* buf, uint16 len)
{
	u16 temp;
	u16 i,j; 
	   
	for(i=0;i<len;i++)
  {
		for(j=1;j<len-i;j++)
		{
			if(buf[j-1]>buf[j])
			{
				temp = buf[j-1];
				buf[j-1] = buf[j];
				buf[j] = temp;
			}
		}
  }
}

void get_zero_cross_vol( void )
{
	uint16 i;
	static uint32 channel_vol[ADC_CHANNEL_MAX+1];
	static uint16 cnt;
	
	cnt++;
	for(i=0;i<ADC_CHANNEL_MAX;i++)
	{
		channel_vol[i] += dma_buf[i];
	}	
	if(cnt == ZERO_CROSS_SAMPLE_NUM)
	{
		sys_para.status.bits.get_zero_cross_flag = 1;	
		cnt = 0;
		TIM_Cmd(TIM1, DISABLE);
		DMA_DeInit(DMA1_Channel1);
		for(i=0;i<=ADC_CHANNEL_MAX;i++)
		{
			channel_data[i].vol_zero_cross_tmp.uint16 = (uint16)(channel_vol[i] / ZERO_CROSS_SAMPLE_NUM);
			channel_vol[i] = 0;
		}
	}
}

/*********************************************************************************************/
void get_channel_vol( void )
{
	uint16 i, cnt;
	uint16 adc_buf[DMA_BUF_MAX+1];
	
	if(sys_para.ch_num <= WAVE_CHANNEL_CNT && sys_para.ch_num >= 1)
	{
		for(i=0;i<WAVE_CHANNEL_CNT;i++)
		{
			cnt = channel_data[i].wave_length;
			if(cnt >= WAVE_LEN_MAX)
			{
				sys_para.status.bits.wave_sample_end = 1;
				return;
			}
 			channel_wave[i][cnt] = dma_buf[i];
 			channel_data[i].wave_length++;		
		}

	}
	else if(sys_para.ch_num <= ADC_CHANNEL_MAX-3 && sys_para.ch_num >= 1)
	{
		memcpy((uint8 *)adc_buf, (uint8 *)dma_buf, DMA_BUF_MAX*2);
		memcpy((uint8 *)adc_tmp[sys_para.ch_num-10], (uint8 *)dma_buf, DMA_BUF_MAX*2);
		bubble_sort( adc_buf, DMA_BUF_MAX);
		channel_data[sys_para.ch_num-1].vol_high_max.uint16 = adc_buf[DMA_BUF_MAX-5];
		channel_data[sys_para.ch_num-1].vol_high_min.uint16 = adc_buf[DMA_BUF_MAX-80];
		channel_data[sys_para.ch_num-1].vol_high_mean.uint16 = adc_buf[DMA_BUF_MAX-40];
		channel_data[sys_para.ch_num-1].vol_low_max.uint16 = adc_buf[80];
		channel_data[sys_para.ch_num-1].vol_low_min.uint16 = adc_buf[5];
		channel_data[sys_para.ch_num-1].vol_low_mean.uint16 = adc_buf[40];
	}	
	else if(sys_para.ch_num < ADC_CHANNEL_MAX && sys_para.ch_num >= 1)
	{
		memcpy((uint8 *)adc_buf, (uint8 *)dma_buf, DMA_BUF_MAX*2);
		memcpy((uint8 *)adc_tmp[sys_para.ch_num-10], (uint8 *)dma_buf, DMA_BUF_MAX*2);
		bubble_sort( adc_buf, DMA_BUF_MAX);
		channel_data[sys_para.ch_num-1].vol_high_mean.uint16 = adc_buf[DMA_BUF_MAX-1];
		channel_data[sys_para.ch_num-1].vol_low_mean.uint16 = adc_buf[0];
	}	
	
	if(sys_para.ch_num == (ADC_CHANNEL_MAX-1))
	{
		get_hall_vol_from_wave(adc_buf);	
	}
}


void get_zero_cross_index( void )
{
	uint16 ch_num, in, zero_cross_vol, index;
	
	/* find zero cross */
	for(ch_num=0;ch_num<3;ch_num++)//there are 3 channels, U and V and W
	{
		zero_cross_vol = channel_data[ch_num].vol_zero_cross.uint16;
		in = channel_wave[ch_num][0] > zero_cross_vol ? 1 : 0;

		for(index = 0; index < channel_data[ch_num].wave_length; index++)
		{
			if(in)
			{
				if(channel_wave[ch_num][index] <= zero_cross_vol)
				{
					channel_data[ch_num].zeropos[channel_data[ch_num].zerocnt] = index;
					channel_data[ch_num].zerocnt ++;
					in = !in;
				}
			}
			else
			{
				if(channel_wave[ch_num][index] > zero_cross_vol)
				{
					channel_data[ch_num].zeropos[channel_data[ch_num].zerocnt] = index;
					channel_data[ch_num].zerocnt ++;
					in = !in;
				}
			}

			if(channel_data[ch_num].zerocnt >= 10)
			{
				break;
			}
		}					
	}	
}

void calc_uvw_vol_rms( void )
{
	uint16 index, ch_num, size, index_start, index_end;
	uint64_t mac = 0;
	int32_t in;
	short zero_cross_vol;
	
	get_zero_cross_index();
	for(ch_num=0;ch_num<3;ch_num++)//there are 3 channels, U and V and W
	{
		zero_cross_vol = (short)channel_data[ch_num].vol_zero_cross.uint16;
		index_start = channel_data[ch_num].zeropos[0];
		index_end = channel_data[ch_num].zeropos[channel_data[ch_num].zerocnt - 1];
		size = index_end - index_start + 1;
		
		for(index=index_start; index<=index_end; index++)
		{
			in = (short)channel_wave[ch_num][index] - zero_cross_vol;
			mac += in * in;							
		}
		channel_data[ch_num].vol_rms.uint16 = (uint16_t)sqrt((double)mac / (double)size);
		mac = 0;
	}	
}

void get_hall_vol_from_wave(uint16* adc_buf)
{
	uint16 i;
	
	for(i=3;i<=8;i++)
	{
		memcpy((uint8 *)adc_buf, (uint8 *)channel_wave[i], DMA_BUF_MAX*2);
		bubble_sort( adc_buf, DMA_BUF_MAX);
		channel_data[i].vol_high_max.uint16 = adc_buf[DMA_BUF_MAX-5];
		channel_data[i].vol_high_min.uint16 = adc_buf[DMA_BUF_MAX-80];
		channel_data[i].vol_high_mean.uint16 = adc_buf[DMA_BUF_MAX-40];
		channel_data[i].vol_low_max.uint16 = adc_buf[80];		
		channel_data[i].vol_low_min.uint16 = adc_buf[5];
		channel_data[i].vol_low_mean.uint16 = adc_buf[40];		
	}	
}


static portTASK_FUNCTION( get_vol_task, pvParameters )
{ 
	while(1)
	{    		 
		xSemaphoreTake( ad_end, portMAX_DELAY );	
			
		if( sys_para.status.bits.get_zero_cross_flag == 0 )//get zero cross voltage for each channel when poweron
		{
			get_zero_cross_vol();				
		}
		else
		{
			get_channel_vol();		
		}
	}    
}

void start_get_vol_task( unsigned portBASE_TYPE uxPriority )
{
    
	/* Spawn the task. */
	xTaskCreate(get_vol_task, ( const signed portCHAR * const ) "get_vol", GET_VOL_STACK_SIZE, NULL, uxPriority, ( xTaskHandle * ) NULL );
}


// if(get_zero_cross_vol_flag == 0)
// {
// 	DMA_Cmd(DMA1_Channel1, DISABLE);	  
// 	DMA_SetCurrDataCounter(DMA1_Channel1, ADC_CHANNEL_MAX);
// 	DMA_Cmd(DMA1_Channel1, ENABLE);	
// }
