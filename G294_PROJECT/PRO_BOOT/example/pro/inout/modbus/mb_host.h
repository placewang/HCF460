#ifndef __MB_HOST_H
#define __MB_HOST_H

#include "mb_port.h"
#include "stdint.h"

#define MBH_RTU_MIN_SIZE	4
#define MBH_RTU_MAX_SIZE	255	//最大不超过255
#define MBH_ERR_MAX_TIMES	5
#define MBH_REC_TIMEOUT		10  //单位3.5T
#define MBH_MASTER_NUM       2   //主机数量

typedef enum
{
	MBH_STATE_IDLE=0X00,
	MBH_STATE_TX,
	MBH_STATE_TX_END,
	MBH_STATE_RX,
	MBH_STATE_RX_CHECK,
	MBH_STATE_EXEC,
	MBH_STATE_REC_ERR,		//接收错误状态
	MBH_STATE_TIMES_ERR,	//传输
	
}mb_host_state;

typedef struct
{
	uint8_t state;						//modbus host状态
	uint8_t errTimes;  					//失败次数计数
	uint8_t txLen;     					//需要发送的帧长度
	uint8_t txCounter;					//已发送bytes计数
	uint8_t txBuf[MBH_RTU_MAX_SIZE];	//发送缓冲区
	uint8_t rxCounter;					//接收计数
	uint8_t rxBuf[MBH_RTU_MAX_SIZE];	//接收缓冲区
	uint8_t rxTimeOut;					//接收时的超时计数
	
}MBHOST;

extern MBHOST mbHost[MBH_MASTER_NUM];

void mbh_init(uint32_t baud);

int8_t mbh_send(uint8_t idx,uint8_t add,uint8_t cmd,uint8_t *data,uint8_t data_len);

uint8_t mbh_getState(uint8_t idx);

void mbh_poll(uint8_t idx);

void mbh_timer3T5Isr(uint8_t idx);

void mbh_uartRxIsr(uint8_t idx);

void mbh_uartTxIsr(uint8_t idx);

void mb_port_timercnt_clear(uint8_t idx);

void mbHost_Init(void);



#endif

