#ifndef __BOARD_G294_H
#define __BOARD_G294_H
#include "machconfig.h"
#include "hc32f46x_spi.h"
#include "hc32_ddl.h"
#include "hc32f46x_gpio.h"
#include "hc32f46x_pwc.h"
#include "hc32f46x_timer0.h"
#include "hc32f46x_spi.h"
#include "hc32f46x_can.h"
#include "hc32f46x_interrupts.h"
#include "hc32f46x_timera.h"
#include "hc32f46x_usart.h"

#define  ENABLE  1
#define  DISABLE 0
enum
{
 M_WORK_LED=0,
 M_IN1=1,
 M_IN2=2,
 M_IN3=3,
 M_SPI1_SS1=4,
 M_SPI1_SS2=5, 
 M_SPI2_SS0=6,   
 M_MOTOR1_DIAG0=7,
 M_MOTOR1_DIAG1=8,
 M_MOTOR1_ZERO=9,
 M_MOTER1_ENN=10,
 M_MOTER2_ENN=11,
 M_MOTOR2_ZERO=12,
 M_MOTOR2_DIAG1=13,
 M_MOTOR2_DIAG0=14,
 M_SPI3_SS0=15,
 M_SPI3_SCK=16,
 M_SPI3_MOSI=17,
 M_SPI3_MISO=18,
 M_8803_ALARM=19,
 M_8803_OUT1=20,
 M_8803_OUT2=21,
 M_8803_OUT3=22,
 M_8803_REST=23,
 M_8803_EN=24,
 M__CAN_RX=25,
 M__CAN_TX=26,
 N_USED_MAX=27,    
};

enum
{
 N_SPI_SCK=0,
 N_SPI_NSS=1,
 N_SPI_MOSI=2,     
 N_SPI_MISO=3,    
};

typedef struct
{
 uint8_t   id;
 uint8_t   type;
 uint8_t   hardware[4];
 uint8_t   boot_ver[2];
 uint8_t   app_ver[2];
 uint8_t   uid[12];
 uint16_t  private_canid;
}_MAINBOARD_INFO;


extern _MAINBOARD_INFO boardinfo;
extern uint32_t timcan;

extern const en_port_t g294mcu_port[N_USED_MAX];
extern const en_pin_t g294mcu_pin[N_USED_MAX];

#define  NMAXSPI  3


#define LED0_OFF()                      (PORT_SetBits(mcu_port[IO_LED], mcu_pin[IO_LED]))
#define LED0_ON()                       (PORT_ResetBits(mcu_port[IO_LED], mcu_pin[IO_LED]))
#define TMR_UNIT            (M4_TMR02)
#define TMR_INI_GCMA        (INT_TMR02_GCMA)
#define TMR_INI_GCMB        (INT_TMR02_GCMB)
#define ENABLE_TMR0()       (PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM02, Enable))
#define TIME0_VAL    1000//定义定时器0是us 

#define LED0_TOGGLE                   (PORT_Toggle(g294mcu_port[M_WORK_LED], g294mcu_pin[M_WORK_LED]))
#define SPI3_CLK_H                    PORT_SetBits(g294mcu_port[M_SPI3_SCK], g294mcu_pin[M_SPI3_SCK])
#define SPI3_CLK_L                    PORT_ResetBits(g294mcu_port[M_SPI3_SCK], g294mcu_pin[M_SPI3_SCK])
#define M_SPI3_MOSI_H                 PORT_SetBits(g294mcu_port[M_SPI3_MOSI], g294mcu_pin[M_SPI3_MOSI])
#define M_SPI3_MOSI_L                 PORT_ResetBits(g294mcu_port[M_SPI3_MOSI], g294mcu_pin[M_SPI3_MOSI])

extern void G294GpioInit(void);
extern void NewSpi_Config(int nspi);
extern void Init_CanConfig(void (*callback)());
extern uint8_t SPI1_ReadWriteByte(uint8_t TxData,uint8_t *iserr);
extern void motor_spi_cs(uint8_t nmot,uint8_t level);
extern void Init_Timer0(uint16_t time_us,void (*TimeCallback)());
extern void motor_en(uint8_t nmot,uint8_t en);
extern void ValveSpi_Config(int nspi);
extern void SPI2_3_WriteByte(uint8_t nspi,uint8_t TxData);
extern void spi_cs(uint8_t nspi,uint8_t level);
extern void SPI2_3_WriteLen(uint8_t nspi,uint8_t *TxData,uint8_t len);
extern uint8_t zerosignal(uint8_t nmot);
extern void Init_Flash(void);
extern void EFM_EraseProgram(uint32_t u32Addr,uint32_t dat);
extern uint32_t EFM_ReadByAddr(uint32_t u32Addr);
extern int CanMessageSendReal(int canid,unsigned char *buf);
extern int CanMessageSend(int canid,unsigned char *buf);
extern uint8_t inputsignal(uint8_t s_num);


extern void Drv8803_Enable(uint8_t en);
extern void Drv8803_Reset(void);
extern void Drv8803_Out(uint8_t num,uint8_t sts);
extern uint8_t Drv8803_AlarmCheck(void);

#endif































