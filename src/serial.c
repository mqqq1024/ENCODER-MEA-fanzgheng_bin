#include "main.h"
#include "config.h"
#include "public.h" //全局变量定义文件

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Library includes. */
#include "stm32f10x_conf.h"

/* Driver includes. */
#include "serial.h"
#include "led.h"
#include "stdio.h"

/*-----------------------------------------------------------*/
/******************************************************************************
*** NOTE:  COM1 == USART1, COM2 == USART2, COM3 == USART3
******************************************************************************/
/* Private define ------------------------------------------------------------*/
#define USARTx                   USART1
#define USARTx_GPIO              GPIOA
#define USARTx_CLK               RCC_APB2Periph_USART1
#define USARTx_GPIO_CLK          RCC_APB2Periph_GPIOA
#define USARTx_RxPin             GPIO_Pin_10
#define USARTx_TxPin             GPIO_Pin_9
#define USARTx_IRQChannel        USART1_IRQn

#define USARTy                   USART2
#define USARTy_GPIO              GPIOD
#define USARTy_CLK               RCC_APB1Periph_USART2
#define USARTy_GPIO_CLK          RCC_APB2Periph_GPIOD
#define USARTy_RxPin             GPIO_Pin_6
#define USARTy_TxPin             GPIO_Pin_5
#define USARTy_IRQChannel        USART2_IRQn

#define USARTz                   USART3
#define USARTz_GPIO              GPIOC
#define USARTz_CLK               RCC_APB1Periph_USART3
#define USARTz_GPIO_CLK          RCC_APB2Periph_GPIOC
#define USARTz_RxPin             GPIO_Pin_11
#define USARTz_TxPin             GPIO_Pin_10
#define USARTz_IRQChannel        USART3_IRQn


/* The number of COM ports that can be controlled at the same time. */
#define serNUM_COM_PORTS				( 3 )

/* Queues are used to hold characters that are waiting to be transmitted.  This
constant sets the maximum number of characters that can be contained in such a
queue at any one time. */
#define serTX_QUEUE_LEN					( 2000 )

/* Queues are used to hold characters that have been received but not yet 
processed.  This constant sets the maximum number of characters that can be 
contained in such a queue. */
#define serRX_QUEUE_LEN					( 100 )

/* The maximum amount of time that calls to lSerialPutString() should wait for
there to be space to post each character to the queue of characters waiting
transmission.  NOTE!  This is the time to wait per character - not the time to
wait for the entire string. */
#define serPUT_STRING_CHAR_DELAY		( 5 / portTICK_RATE_MS )

/*-----------------------------------------------------------*/

/* References to the USART peripheral addresses themselves. */
//static USART_TypeDef * const xUARTS[ serNUM_COM_PORTS ] = { ( ( USART_TypeDef * ) USART1_BASE ), ( ( //USART_TypeDef * ) USART2_BASE ), ( ( USART_TypeDef * ) USART3_BASE ) };
static USART_TypeDef * const xUARTS[ serNUM_COM_PORTS ] ={USART1, USART2, USART3};

/* Queues used to hold characters waiting to be transmitted - one queue per port. */
static xQueueHandle xCharsForTx[ serNUM_COM_PORTS ] = { 0 };

/* Queues holding received characters - one queue per port. */
static xQueueHandle xRxedChars[ serNUM_COM_PORTS ] = { 0 };


/*-----------------------------------------------------------*/
void usart1_gpio_init(void);
void usart1_rcc_init(void);
void usart1_init(unsigned long wanted_baud);

void usart2_gpio_init(void);
void usart2_rcc_init(void);
void usart2_init(unsigned long wanted_baud);

void usart3_gpio_init(void);
void usart3_rcc_init(void);
void usart3_init(unsigned long wanted_baud);


/* UART interrupt handlers, as named in the vector table. */
void USART1_IRQHandler( void );
void USART2_IRQHandler( void );
void USART3_IRQHandler( void );

/*
 * support usart1 and usart2 and usart3
 */

portBASE_TYPE usart_init(unsigned long port, unsigned long wanted_baud)
{
    long ret = pdFAIL;
    if( port >= serNUM_COM_PORTS )
        return ret;
    if(port ==0 )
        return ret;
		
		/* Create the queue of chars that are waiting to be sent to COM1. */
		xCharsForTx[ port-1 ] = xQueueCreate( serTX_QUEUE_LEN, sizeof( char ) );
		/* Create the queue used to hold characters received from COM1. */
		xRxedChars[ port-1  ] = xQueueCreate( serRX_QUEUE_LEN, sizeof( char ) );  
		
    if(1==port)
    {        
        usart1_rcc_init();
        usart1_gpio_init();
        usart1_init(wanted_baud);
        ret = pdPASS;

				USART_ITConfig( USART1, USART_IT_RXNE, ENABLE );	  
    }
    if(2==port)
    {       
        usart2_rcc_init();
        usart2_gpio_init();
        usart2_init(wanted_baud);
        ret = pdPASS;

//				USART_ITConfig( USART2, USART_IT_RXNE, ENABLE );	  
    }
    else if(3 == port)
    {        
        usart3_rcc_init();
        usart3_gpio_init();
        usart3_init(wanted_baud);
        ret = pdPASS;
    }
    return ret;

}

/*-----------------------------------------------------------*/
void usart1_init(unsigned long wanted_baud)
{
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    /* USARTy and USARTz configuration -------------------------------------------*/
    /* USARTy and USARTz configured as follow:
        - BaudRate = 2400 baud  
        - Word Length = 9 Bits
        - One Stop Bit
        - Even parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
    */
    USART_InitStructure.USART_BaudRate = wanted_baud;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		//USART_InitStructure.USART_Clock = USART_Clock_Disable;
		//USART_InitStructure.USART_CPOL = USART_CPOL_Low;
		//USART_InitStructure.USART_CPHA = USART_CPHA_2Edge;
		//USART_InitStructure.USART_LastBit = USART_LastBit_Disable;
		USART_Init(USART1, &USART_InitStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = USARTx_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init( &NVIC_InitStructure );        
  
    USART_ITConfig( USARTx, USART_IT_RXNE, ENABLE );
		USART_ITConfig( USARTx, USART_IT_TC, ENABLE );
			       
    //USART_DMACmd( USART1, ( USART_DMAReq_Tx | USART_DMAReq_Rx ), ENABLE );
    
    /* Enable the USART1 */
		USART_Cmd(USART1, ENABLE);
		USART_ClearFlag (USART1, USART_FLAG_TC);

}

/*-----------------------------------------------------------*/
void usart2_init(unsigned long wanted_baud)
{
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    /* USARTy and USARTz configuration -------------------------------------------*/
    /* USARTy and USARTz configured as follow:
        - BaudRate = 2400 baud  
        - Word Length = 9 Bits
        - One Stop Bit
        - Even parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
    */
    USART_InitStructure.USART_BaudRate = wanted_baud;
    USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_Even;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    //USART_InitStructure.USART_Clock = USART_Clock_Disable;
	//USART_InitStructure.USART_CPOL = USART_CPOL_Low;
	//USART_InitStructure.USART_CPHA = USART_CPHA_2Edge;
	//USART_InitStructure.USART_LastBit = USART_LastBit_Disable;
  
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        
    /* Configure USARTy */
		USART_Init(USART2, &USART_InitStructure);

		USART_ITConfig( USARTy, USART_IT_RXNE, DISABLE );
		USART_ITConfig( USART2, USART_IT_TC, DISABLE );
    //USART_ITConfig( USARTy, USART_IT_TXE, ENABLE );
			
		NVIC_InitStructure.NVIC_IRQChannel = USARTy_IRQChannel;
		NVIC_Init( &NVIC_InitStructure );
    
    
    //USART_DMACmd( USART2, ( USART_DMAReq_Tx | USART_DMAReq_Rx ), ENABLE );
    
    /* Enable the USART2 */
		USART_Cmd(USART2, ENABLE);
		USART_ClearFlag (USART2, USART_FLAG_TC);

}


void usart3_init(unsigned long wanted_baud)
{
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    /* USARTy and USARTz configuration -------------------------------------------*/
    /* USARTy and USARTz configured as follow:
        - BaudRate = 9600 baud  
        - Word Length = 9 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
    */
    USART_InitStructure.USART_BaudRate = wanted_baud;
    USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        
    /* Configure USARTz */
    USART_Init(USART3, &USART_InitStructure);
  
    USART_ITConfig( USART3, USART_IT_RXNE, DISABLE );
	//USART_ITConfig( USARTz, USART_IT_TXE, ENABLE );
		USART_ITConfig( USART3, USART_IT_TC, DISABLE );

		NVIC_InitStructure.NVIC_IRQChannel = USARTz_IRQChannel;
		NVIC_Init( &NVIC_InitStructure );
    
    
    //USART_DMACmd( USARTz, ( USART_DMAReq_Tx | USART_DMAReq_Rx ), ENABLE );
    
    /* Enable the USARTz */
		USART_Cmd(USART3, ENABLE);
		USART_ClearFlag (USART3, USART_FLAG_TC);

}

void usart1_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure; 

    /* Configure USARTx Rx as input floating */
    GPIO_InitStructure.GPIO_Pin = USARTx_RxPin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(USARTx_GPIO, &GPIO_InitStructure);
  
    /* Configure USARTx Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = USARTx_TxPin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(USARTx_GPIO, &GPIO_InitStructure);

}
void usart2_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure; 

    /* Configure USARTy Rx as input floating */
    GPIO_InitStructure.GPIO_Pin = USARTy_RxPin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(USARTy_GPIO, &GPIO_InitStructure);
  
    /* Configure USARTy Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = USARTy_TxPin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(USARTy_GPIO, &GPIO_InitStructure);
    
    /* Enable the USART2 Pins Software Remapping */
    GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE); 

}


void usart3_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* Configure USARTz Rx as input floating */
    GPIO_InitStructure.GPIO_Pin = USARTz_RxPin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(USARTz_GPIO, &GPIO_InitStructure);  

    /* Configure USARTz Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = USARTz_TxPin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(USARTz_GPIO, &GPIO_InitStructure);  
    
    /* Enable the USART3 Pins Software Remapping */
    GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);
    
}

void usart1_rcc_init(void)
{
    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(USARTx_GPIO_CLK, ENABLE);
    
    /* Enable USART1 Clock */
    RCC_APB2PeriphClockCmd(USARTx_CLK, ENABLE); 

}
void usart2_rcc_init(void)
{
    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(USARTy_GPIO_CLK, ENABLE);
    
    /* Enable AFIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    
    /* Enable USART2 Clock */
    RCC_APB1PeriphClockCmd(USARTy_CLK, ENABLE); 

}

void usart3_rcc_init(void)
{
    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(USARTz_GPIO_CLK, ENABLE);
    
    /* Enable AFIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    
    /* Enable USART3 Clock */
    RCC_APB1PeriphClockCmd(USARTz_CLK, ENABLE); 

}
/*
*************************************************************************
 * usart_send_frame() - 发送一帧数据
 * @frame: 指向要发送帧的指针.
 * @len: 帧的字节长度.
 *
 *返回1表示成功，0表示失败 。
 *  认为发送队列已经空
 *
 * 与中断服务程序配合，完成对第9位数据位的处理。
 *
 **/
uint32 usart_send_frame(long port, uint8* frame, uint32 len)
{
	uint32 i;
//	uint32 num;
	uint8* p;
//	uint8 rubbish;
	
	if(1==port)
		USART_ITConfig( USART1, USART_IT_RXNE, DISABLE );
	else if(2==port)
		USART_ITConfig( USART2, USART_IT_RXNE, DISABLE );
	else if(3==port)
		USART_ITConfig( USART3, USART_IT_RXNE, DISABLE );
	else
	{}
	
	p=frame;
	
	////////////////////////////////////////////////////////////////////////////
	//清空接收队列，以防接收队列内有数据
// 	num = uxQueueMessagesWaiting (xRxedChars[ port-1 ]);
// 	while(num > 0)
// 	{
// 		xQueueReceive(xRxedChars[ port-1 ], (void*)(&rubbish), 0);
// 		num--;
// 	}
	
  if(1==port)
	{
		USART_ClearFlag (USART1, USART_FLAG_TC);
		/* send the first byte */
		USART_SendData( USART1, (*p) );//( USART2, 0x100|(*p) );
	}
  else if(2==port)
	{
		USART_ClearFlag (USART2, USART_FLAG_TC);
		USART_SendData( USART2, (*p) );//( USART3, 0x100|(*p) );
	}
  else if(3==port)
	{
		USART_ClearFlag (USART3, USART_FLAG_TC);
		USART_SendData( USART3, (*p) );//( USART3, 0x100|(*p) );
	}
	else
	{}
	
	p++;
	
	for(i=0;i<len-1;i++)
	{
		//此时发送缓冲区为空，肯定能发送出去
		xQueueSend( xCharsForTx[ port-1 ], p, 0 );
		p++;
	}
	
	USART_ITConfig( xUARTS[ port-1 ], USART_IT_TC, ENABLE );
	return 1;
}

// /**
//  * usart_recv_frame() - 接收一帧数据
// * @frame:	指向要接收帧缓冲区的指针.
//  * @len:	帧的字节长度.
//  *		
//  *返回1表示成功，0表示失败 。
//  * 
//  **/
//  uint32 usart_recv_frame(long port, uint8* frame, uint32 len)
// {
// 	uint32 i;
// 	uint8* p;
// 	
// 	p=frame;
// 	
// 	if (pdFALSE == xQueueReceive(xRxedChars[ port-1 ], (void*)(p), 1000/portTICK_RATE_MS))
// 			return 0;
// 	p++;
// 	
// 	//接收len-1个数据
// 	for (i = 0; i<(len-1); i++)
// 	{
// 			//接收下一个字符，等待400ms未收到则接收失败
// 			if (pdFALSE == xQueueReceive(xRxedChars[ port-1 ], (void*)(p), 400/portTICK_RATE_MS))
// 					return 0;
// 					
// 			p++;
// 	
// 	}
// 	return 1;

// }
/**
 * usart_recv_frame() - 接收一帧数据
* @frame:	指向要接收帧缓冲区的指针.
 * @len:	帧的字节长度.
 *		
 *返回1表示成功，0表示失败 。
 * 
 **/
 uint32 usart_recv_frame(long port, uint8* frame)//, uint32 len)
{
	uint8 i, len;
	uint8* p;
	
	p=frame;
	
	if (pdFALSE == xQueueReceive(xRxedChars[ port-1 ], (void*)(p), 10/portTICK_RATE_MS))
		return 0;	
	else if(*p != 0x7b)
		return 0;
			
	p++;
	if (pdFALSE == xQueueReceive(xRxedChars[ port-1 ], (void*)(p), 10/portTICK_RATE_MS))
		return 0;
	else
		len = *p;
	
	p++;
	//接收len-2个数据
	for (i=0; i<(len-2); i++)
	{
			//接收下一个字符，等待100ms未收到则接收失败
			if (pdFALSE == xQueueReceive(xRxedChars[ port-1 ], (void*)(p), 10/portTICK_RATE_MS))
					return 0;
					
			p++;	
	}
	return 1;

}

/*-----------------------------------------------------------*/

signed long xSerialGetChar( long lPort, signed char *pcRxedChar, portTickType xBlockTime )
{
	long lReturn = pdFAIL;

	if( lPort < serNUM_COM_PORTS ) 
	{
		if( xQueueReceive( xRxedChars[ lPort ], pcRxedChar, xBlockTime ) == pdPASS )
		{
			lReturn = pdPASS;
		}
	}

	return lReturn;
}
/*-----------------------------------------------------------*/

long lSerialPutString( long lPort, const char * const pcString, unsigned long ulStringLength )
{
long lReturn;
unsigned long ul;

	if( lPort < serNUM_COM_PORTS )
	{
		lReturn = pdPASS;

		for( ul = 0; ul < ulStringLength; ul++ )
		{
			if( xQueueSend( xCharsForTx[ lPort ], &( pcString[ ul ] ), serPUT_STRING_CHAR_DELAY ) != pdPASS )
			{
				/* Cannot fit any more in the queue.  Try turning the Tx on to 
				clear some space. */
				USART_ITConfig( xUARTS[ lPort ], USART_IT_TXE, ENABLE );
				vTaskDelay( serPUT_STRING_CHAR_DELAY );

				/* Go back and try again. */
				continue;
			}
		}

        USART_ITConfig( xUARTS[ lPort ], USART_IT_TXE, ENABLE );
	}
	else
	{
		lReturn = pdFAIL;
	}

	return lReturn;
}
/*-----------------------------------------------------------*/

signed long xSerialPutChar( long lPort, signed char cOutChar, portTickType xBlockTime )
{
long lReturn;

	if( xQueueSend( xCharsForTx[ lPort ], &cOutChar, xBlockTime ) == pdPASS )
	{
		lReturn = pdPASS;
		USART_ITConfig( xUARTS[ lPort ], USART_IT_TXE, ENABLE );
	}
	else
	{
		lReturn = pdFAIL;
	}

	return lReturn;
}

#if 1
void USART1_IRQHandler( void )
{
long xHigherPriorityTaskWoken = pdFALSE;
char cChar;

	if( USART_GetITStatus( USART1, USART_IT_TC ) == SET )
	{
		/* The interrupt was caused by the THR becoming empty.  Are there any
		more characters to transmit? */
		if( xQueueReceiveFromISR( xCharsForTx[ 0 ], &cChar, &xHigherPriorityTaskWoken ) )
		{
			/* A character was retrieved from the buffer so can be sent to the
			THR now. */
			USART_SendData( USART1, cChar );
		}
		else
		{	
			/* change to recv first, so this make RXNE set if recevied 00h*/
			/* the sequence is carefully setted */
			USART_ClearFlag (USART1, USART_FLAG_TC);
			USART_ITConfig( USART1, USART_IT_TC, DISABLE );
			USART_ClearFlag (USART1, USART_FLAG_RXNE);
			USART_ITConfig( USART1, USART_IT_RXNE, ENABLE );
		}		
	}
	
	if( USART_GetITStatus( USART1, USART_IT_RXNE ) == SET )
	{
		cChar = USART_ReceiveData( USART1 );
		xQueueSendFromISR( xRxedChars[ 0 ], &cChar, &xHigherPriorityTaskWoken );
	}	
	
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
#endif

#if 0
void USART2_IRQHandler( void )
{
long xHigherPriorityTaskWoken = pdFALSE;
char cChar;

	if( USART_GetITStatus( USART2, USART_IT_TC ) == SET )
	{
		/* The interrupt was caused by the THR becoming empty.  Are there any
		more characters to transmit? */
		if( xQueueReceiveFromISR( xCharsForTx[ 1 ], &cChar, &xHigherPriorityTaskWoken ) )
		{
			/* A character was retrieved from the buffer so can be sent to the
			THR now. */
			USART_SendData( USART2, cChar );
		}
		else
		{	
            /* change to recv first, so this make RXNE set if recevied 00h*/
            /* the sequence is carefully setted */
			USART_ClearFlag (USART2, USART_FLAG_TC);
			USART_ITConfig( USART2, USART_IT_TC, DISABLE );
			USART_ClearFlag (USART2, USART_FLAG_RXNE);
			USART_ITConfig( USART2, USART_IT_RXNE, ENABLE );
		}		
	}
	
	if( USART_GetITStatus( USART2, USART_IT_RXNE ) == SET )
	{
		cChar = USART_ReceiveData( USART2 );
		xQueueSendFromISR( xRxedChars[ 1 ], &cChar, &xHigherPriorityTaskWoken );
	}	
	
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
#endif

#if 0
void USART2_IRQHandler( void )
{
long xHigherPriorityTaskWoken = pdFALSE;
char cChar;

	if( USART_GetITStatus( USART2, USART_IT_TXE ) == SET )
	{
		/* The interrupt was caused by the THR becoming empty.  Are there any
		more characters to transmit? */
		if( xQueueReceiveFromISR( xCharsForTx[ 1 ], &cChar, &xHigherPriorityTaskWoken ) )
		{
			/* A character was retrieved from the buffer so can be sent to the
			THR now. */
			USART_SendData( USART2, cChar );
		}
		else
		{	
            USART_ITConfig( USART2, USART_IT_TXE, DISABLE );
            set_usart2_rs485_recv();
		}		
	}
	
	if( USART_GetITStatus( USART2, USART_IT_RXNE ) == SET )
	{
		cChar = USART_ReceiveData( USART2 );
		xQueueSendFromISR( xRxedChars[ 1 ], &cChar, &xHigherPriorityTaskWoken );
	}	
	
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
#endif

/*-----------------------------------------------------------*/
#if 0
void USART2_IRQHandler( void )
{
long xHigherPriorityTaskWoken = pdFALSE;
char cChar;

	if( USART_GetITStatus( USART2, USART_IT_TXE ) == SET )
	{
		/* The interrupt was caused by the THR becoming empty.  Are there any
		more characters to transmit? */
		if( xQueueReceiveFromISR( xCharsForTx[ 1 ], &cChar, &xHigherPriorityTaskWoken ) )
		{
			/* A character was retrieved from the buffer so can be sent to the
			THR now. */
			USART_SendData( USART2, cChar );
		}
		else
		{	
            //USART_ITConfig( USART2, USART_IT_TXE, DISABLE );
            USART_ITConfig( USART2, USART_IT_TC, ENABLE );
		}		
	}
    
    if(USART_GetITStatus( USART2, USART_IT_TC ) == SET )
    {
        USART_ClearITPendingBit(USART2, USART_IT_TC);
        USART_ITConfig( USART2, USART_IT_TXE, DISABLE );
        USART_ITConfig( USART2, USART_IT_TC, DISABLE );
        set_usart2_rs485_recv();
    }
	
	if( USART_GetITStatus( USART2, USART_IT_RXNE ) == SET )
	{
		cChar = USART_ReceiveData( USART2 );
		xQueueSendFromISR( xRxedChars[ 1 ], &cChar, &xHigherPriorityTaskWoken );
	}	
	
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

#endif

/*-----------------------------------------------------------*/

#if 0
void USART3_IRQHandler( void )
{
long xHigherPriorityTaskWoken = pdFALSE;
char cChar;

	if( USART_GetITStatus( USART3, USART_IT_TXE ) == SET )
	{
		/* The interrupt was caused by the THR becoming empty.  Are there any
		more characters to transmit? */
		if( xQueueReceiveFromISR( xCharsForTx[ 2 ], &cChar, &xHigherPriorityTaskWoken ) )
		{
			/* A character was retrieved from the buffer so can be sent to the
			THR now. */
			USART_SendData( USART3, cChar );
		}
		else
		{
			USART_ITConfig( USART3, USART_IT_TXE, DISABLE );	
            set_usart3_rs485_recv();
		}		
	}
	
	if( USART_GetITStatus( USART3, USART_IT_RXNE ) == SET )
	{
		cChar = USART_ReceiveData( USART3 );
		xQueueSendFromISR( xRxedChars[ 2 ], &cChar, &xHigherPriorityTaskWoken );
	}	
	
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}



void USART3_IRQHandler( void )
{
long xHigherPriorityTaskWoken = pdFALSE;
char cChar;

	if( USART_GetITStatus( USART3, USART_IT_TC ) == SET )
	{
		/* The interrupt was caused by the THR becoming empty.  Are there any
		more characters to transmit? */
		if( xQueueReceiveFromISR( xCharsForTx[ 2 ], &cChar, &xHigherPriorityTaskWoken ) )
		{
			/* A character was retrieved from the buffer so can be sent to the
			THR now. */
			USART_SendData( USART3, cChar );
		}
		else
		{
			/* change to recv first, so this make RXNE set if recevied 00h*/
            /* the sequence is carefully setted */
			USART_ClearFlag (USART3, USART_FLAG_TC);
			USART_ITConfig( USART3, USART_IT_TC, DISABLE );
			USART_ClearFlag (USART3, USART_FLAG_RXNE);
			USART_ITConfig( USART3, USART_IT_RXNE, ENABLE );
		}		
	}
	
	if( USART_GetITStatus( USART3, USART_IT_RXNE ) == SET )
	{
		cChar = USART_ReceiveData( USART3 );
		xQueueSendFromISR( xRxedChars[ 2 ], &cChar, &xHigherPriorityTaskWoken );
	}	
	
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
#endif


// #ifdef __GNUC__  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf set to 'Yes') calls __io_putchar() */  
// #define PUTCHAR_PROTOTYPE int __io_putchar(int ch) 
// #else  
// #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f) 
// #endif /* __GNUC__ */ 

// PUTCHAR_PROTOTYPE { 
// 	/* Place your implementation of fputc here */ /* e.g. write a character to the USART */ 
// 	USART_SendData(USART1, (uint8_t) ch); /* Loop until the end of transmission */  
// 	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); 
// 	return ch; 
// }