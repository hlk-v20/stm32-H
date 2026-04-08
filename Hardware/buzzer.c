#include "stm32f10x.h"                  // Device header
#include "Delay.h" 

void buzzer_Init(void)
{

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_10);
}


void buzzer_on (void )
{
		GPIO_ResetBits(GPIOA,GPIO_Pin_10);
}

void buzzer_off (void )
{
		GPIO_SetBits(GPIOA,GPIO_Pin_10);
}
	
void buzzer_turn (void )
{
	if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_10) == 0)
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_10);
	}
	else
	{
		GPIO_ResetBits(GPIOA, GPIO_Pin_10);
	}
}



