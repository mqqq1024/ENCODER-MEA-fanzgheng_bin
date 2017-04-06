/***************************************************************
用户定义API函数

***************************************************************/

#ifndef __USER_API__
#define __USER_API__

#include "config.h"

////////////////////////////////////////////////////////
//通用宏定义

//取二子节数据的高字节和地址节
#define HSB_OF_16(x) ((uint8)((x >> 8)& 0xff))
#define LSB_OF_16(x) ((uint8)(x & 0xff))
////////////////////////////////////////////////////////

//计算CRC16值
uint16 get_crc16(uint8 * buf, int len);
uint16 get_crc16_standard(uint8 * buf, int len);
//计算CRC107值
uint8 get_crc107(uint8 *buff, uint32 len);
//置第i位为1
void set_1(uint8* p, uint32 index);
//置第i位为0
void set_0(uint8* p, uint32 index);

uint16 get_neg_true_form(int16 neg);

int16 get_complement_code(uint16 x);

uint32 get_elapsed_ticks(uint32 pre_tick, uint32 cur_tick);
//////////////////////////////////////////////////////////

#endif
