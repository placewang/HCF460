#ifndef __MB_PORT_H
#define __MB_PORT_H
#include "bsp.h"

enum
{    
 HOST1=0,
 HOST2=1, 
 HOSTMAX=2,
};

typedef enum
{
	MB_PARITY_NONE=0X00,	//无奇偶校验，两个停止位
	MB_PARITY_ODD, 			//奇校验
	MB_PARITY_EVEN			//偶校验
}mbParity;

extern M4_USART_TypeDef* uarthost[HOSTMAX];


void mb_port_uartInit(uint32_t baud);

void mb_port_putchar(uint8_t idx,uint8_t ch);

void mb_port_getchar(uint8_t idx,uint8_t *ch);

void mb_port_timerInit(uint32_t baud);

void mb_port_timerEnable(uint8_t idx);

void mb_port_timerDisable(uint8_t idx);

void mb_port_uartInitTest(void);

void mb_port_uartInitByidx(uint8_t idx,uint32_t baud);

#endif

