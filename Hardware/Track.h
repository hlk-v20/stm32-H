#ifndef __TRACK_H
#define __TRACK_H

#include "stm32f10x.h"

void Track_Init();
uint8_t Read_Left1();
uint8_t Read_Left2();
uint8_t Read_Mid();
uint8_t Read_Right1();
uint8_t Read_Right2();

static __inline uint8_t Track_ReadMask(void)
{
	uint8_t mask = 0;

	mask |= (Read_Left1()  ? 0x10 : 0x00);
	mask |= (Read_Left2()  ? 0x08 : 0x00);
	mask |= (Read_Mid()    ? 0x04 : 0x00);
	mask |= (Read_Right1() ? 0x02 : 0x00);
	mask |= (Read_Right2() ? 0x01 : 0x00);

	return mask;
}

static __inline uint8_t Track_IsBPoint(uint8_t mask)
{
	/* 五路循迹任意一路命中就认为到达B点 */
	if (mask != 0x00)
	{
		return 1;
	}
	return 0;
}

#endif
