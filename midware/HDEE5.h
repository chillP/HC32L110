
//===================================================================================

#ifndef __HDEE__
#define __HDEE__

//===================================================================================

#include "hc32l110.h"

//===================================================================================

#define HDEE_EeSize             32              //ģ��32��EE�ֽڣ����ֵΪ120
#define HDEE_BakSize            32              //����������������ƽ��
#define HDEE_FlashSize          512*4           //4��ҳ������ģ��EE��������Ϊ4��ҳ��
#define HDEE_FlashLock          0X8000L         //ֻ����4��PAGE
#define HDEE_FlashStopAddr      0x8000          //��оƬ��Flash���һ���ֽڵĵ�ַ+1
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
