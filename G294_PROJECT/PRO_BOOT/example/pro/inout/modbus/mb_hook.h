/**
  ******************************************************************************
  * @file    mb_hook.c
  * @brief   modebus回调函数头文件
  ******************************************************************************
  * @note
  * 
  ******************************************************************************
  */

#ifndef __MB_HOOK_H
#define __MB_HOOK_H

void mbh_hook_rec01(uint8_t add,uint8_t *data,uint8_t datalen);
void mbh_hook_rec02(uint8_t add,uint8_t *data,uint8_t datalen);
void mbh_hook_rec03(uint8_t add,uint8_t *data,uint8_t datalen);
void mbh_hook_rec04(uint8_t add,uint8_t *data,uint8_t datalen);
void mbh_hook_rec05(uint8_t add,uint8_t *data,uint8_t datalen);
void mbh_hook_rec06(uint8_t add,uint8_t *data,uint8_t datalen);
void mbh_hook_rec15(uint8_t add,uint8_t *data,uint8_t datalen);
void mbh_hook_rec16(uint8_t add,uint8_t *data,uint8_t datalen);
void mbh_hook_timesErr(uint8_t add,uint8_t cmd);

#endif

