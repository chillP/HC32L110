
//===================================================================================

#ifndef __HDEE__
#define __HDEE__

//===================================================================================

#include "hc32l110.h"

//===================================================================================

#define HDEE_EeSize             32              //模拟32个EE字节，最大值为120
#define HDEE_BakSize            32              //备份区，不做负载平衡
#define HDEE_FlashSize          512*4           //4个页面用于模拟EE，步进量为4个页面
#define HDEE_FlashLock          0X8000L         //只锁定4个PAGE
#define HDEE_FlashStopAddr      0x8000          //该芯片的Flash最后一个字节的地址+1
#define HDEE_FlashStartAddr     ( HDEE_FlashStopAddr - HDEE_FlashSize )

//===================================================================================

void   HDEE_Ini( void );
uint8_t HDEE_Read( uint8_t EeAddr, uint8_t *pRdBuf , uint8_t RdCnt );
uint8_t HDEE_Write( uint8_t EeAddr, uint8_t *pWrBuf , uint8_t WrCnt );

void HDEE_BakZone_Clear( void );
uint8_t HDEE_BakZone_Read( uint8_t *pRdBuf , uint8_t RdCnt );
void HDEE_BakZone_Write( uint8_t *pWrBuf , uint8_t WrCnt );

//===================================================================================

#endif

//===================================================================================
