
#ifndef __H_CONFIG
#define __H_CONFIG

#ifndef TRUE
//#define TRUE  1
#endif

#ifndef true
//#define true  1
#endif

#ifndef FALSE
//#define FALSE 0
#endif

#ifndef false
//#define false 0
#endif

#ifndef NULL
#define NULL  0
#endif

#ifndef null
#define null  0
#endif
/** @} */

/////////////////////////////////////////////////////////////////////
//通用数据类型定义
/**
 * \name 数据类型定义
 * @{
 *
 */
/**
 * 便于移植的数据类型.
 *
 * This should normally not be changed.
 */
//typedef enum _BOOL { FALSE = 0, TRUE } BOOL;	// Undefined size
//typedef char BOOL;

typedef unsigned char  uint8;                   /* 无符号8位整型变量                        */
typedef signed   char  int8;                    /* 有符号8位整型变量                        */
typedef unsigned short uint16;                  /* 无符号16位整型变量                       */
typedef signed   short int16;                   /* 有符号16位整型变量                       */
typedef unsigned int   uint32;                  /* 无符号32位整型变量                       */
typedef signed   int   int32;                   /* 有符号32位整型变量                       */
typedef float          fp32;                    /* 单精度浮点数（32位长度）                 */
typedef double         fp64;                    /* 双精度浮点数（64位长度）                 */
typedef uint8          bool;                    /* 布尔变量        */

// datatype definitions macros
/*
typedef unsigned char  u08;
typedef   signed char  s08;
typedef unsigned short u16;
typedef   signed short s16;
typedef unsigned long  u32;
typedef   signed long  s32;
*/

typedef union __attribute__ ((packed)) uMEM08 // 
{                    //  var.uint8 = 0xFF;
  uint8 uint8 ;          // var.bit._7 = 1;

  struct __attribute__ ((packed))
  {
    uint8 _0: 1;
    uint8 _1: 1;
    uint8 _2: 1;
    uint8 _3: 1;
    uint8 _4: 1;
    uint8 _5: 1;
    uint8 _6: 1;
    uint8 _7: 1;
  } __attribute__ ((packed)) bit;

} __attribute__ ((packed)) tMEM08;

typedef union __attribute__ ((packed)) uMEM16 // 
{                    //var.uint16 = 0xAA55;
  uint16 uint16;     //var.uint8._0 = 0xFF;

  struct __attribute__ ((packed))
  {
    uint8 _0; // LSB
    uint8 _1; // MSB
  } __attribute__ ((packed)) uint8;

} __attribute__ ((packed)) tMEM16;

typedef union uMEM32 // 32-Bit 
{
  uint32 uint32; // var.U32 = 0x4000AA55;
  uint32 *pU32; //U32

  float fp; // var.fp = 123.456;
  float *pfp; //float

  struct // var.uint16._0 = 0xABCD;
  {
    uint16 _0; // LSW
    uint16 _1; // MSW
  } uint16;

  uint16 *puint16; // Zeiger auf uint16

  struct // Zugriff auf Byte var.uint8._0 = 0xFF;
  {
    uint8 _0; // LSB
    uint8 _1;
    uint8 _2;
    uint8 _3; // MSB
  } uint8;

  uint8 *puint8; // Zeiger auf uint8

} tMEM32;                      


#define RETRANSMIT_TIMES 5  /* modbus frame retransmit times */

/* task stack size */
#define UART_COM_STACK_SIZE 512 
#define GET_VOL_STACK_SIZE  512
#define CPLD_TEST_STACK_SIZE  512
#define TEST_STACK_SIZE 256

/* task priority */
#define GET_VOL_PRIORITY  (tskIDLE_PRIORITY+4)
#define UART_COM_PRIORITY  (tskIDLE_PRIORITY+3)
#define CPLD_TEST_PRIORITY  (tskIDLE_PRIORITY+2)
#define TEST_PRIORITY  (tskIDLE_PRIORITY+1)

/**************************************/
#define SOURCE_ADDR 0X01 /* Address */

#define WATCH_DOG 1

#endif

