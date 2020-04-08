//===================================================================================
//
//Demo for EE REV05
//
//===================================================================================

#include "hc32l110.h"
#include "HDEE5.h"

//===================================================================================
//��ʼ��Flash������صļĴ���
void Flash_Operation_Ini( uint8_t Mode )
{
	__disable_irq() ;
	M0P_CLOCK->PERI_CLKEN_f.FLASH = 1;

	M0P_FLASH->BYPASS=0x5a5a;  M0P_FLASH->BYPASS=0xa5a5;  M0P_FLASH->TNVS    = 4 * 8;
	M0P_FLASH->BYPASS=0x5a5a;  M0P_FLASH->BYPASS=0xa5a5;  M0P_FLASH->TPGS    = (uint32_t)(4 * 5.75);
	M0P_FLASH->BYPASS=0x5a5a;  M0P_FLASH->BYPASS=0xa5a5;  M0P_FLASH->TPROG   = (uint32_t)(4 * 6.75);
	M0P_FLASH->BYPASS=0x5a5a;  M0P_FLASH->BYPASS=0xa5a5;  M0P_FLASH->TSERASE = 4 * 4500;
	M0P_FLASH->BYPASS=0x5a5a;  M0P_FLASH->BYPASS=0xa5a5;  M0P_FLASH->TMERASE = 4 * 35000;
	M0P_FLASH->BYPASS=0x5a5a;  M0P_FLASH->BYPASS=0xa5a5;  M0P_FLASH->TPRCV   = 4 * 6;
	M0P_FLASH->BYPASS=0x5a5a;  M0P_FLASH->BYPASS=0xa5a5;  M0P_FLASH->TSRCV   = 4 * 60;
	M0P_FLASH->BYPASS=0x5a5a;  M0P_FLASH->BYPASS=0xa5a5;  M0P_FLASH->TMRCV   = 4 * 250;

	M0P_FLASH->BYPASS=0x5a5a;  M0P_FLASH->BYPASS=0xa5a5;  M0P_FLASH->CR_f.OP = Mode;  //����д��Ƭ��
	M0P_FLASH->BYPASS=0x5a5a;  M0P_FLASH->BYPASS=0xa5a5;  M0P_FLASH->SLOCK = HDEE_FlashLock;  //���4��Page
}

//===================================================================================
// ��ֹ����Flash
void Flash_Operation_DeIni( void )
{
	M0P_FLASH->BYPASS=0x5a5a;  M0P_FLASH->BYPASS=0xa5a5;  M0P_FLASH->CR_f.OP = 0x00;
	M0P_FLASH->BYPASS=0x5a5a;  M0P_FLASH->BYPASS=0xa5a5;  M0P_FLASH->SLOCK = 0x00;

	M0P_CLOCK->PERI_CLKEN_f.FLASH = 0;
	__enable_irq() ;
}

//===================================================================================
//����Addr���ڵ�ҳ��
void Flash_ErasePage( uint32_t Addr )
{
	Flash_Operation_Ini( 2 );  //page erase

	*((uint8_t * )Addr) = 0x00;  //dummy write
	//while( M0P_FLASH->CR_f.BUSY );   //��RAM�����в���Ҫ��ָ��

	Flash_Operation_DeIni();
}

//===================================================================================
//��pWrBuf�е�WrCnt���ֽ�д��Addr��ָ���ĵ�ַ
void Flash_WriteBytes( uint32_t Addr , uint8_t *pWrBuf , uint8_t WrCnt )
{
	Flash_Operation_Ini( 1 );  //write
	while( WrCnt )
	{
		if( (WrCnt>=4) && ( (Addr&0x03)==0x00 ) )
		{
			*((volatile uint32_t * )Addr) = (pWrBuf[3]<<24) | (pWrBuf[2]<<16) | (pWrBuf[1]<<8) | (pWrBuf[0]<<0) ;	
			pWrBuf+=4;
			Addr+=4;
			WrCnt-=4;
		}
		else
		{
			*((volatile uint8_t * )Addr) = *pWrBuf;	
			pWrBuf++;
			Addr++;
			WrCnt--;
		}
	}
	Flash_Operation_DeIni();
}

//===================================================================================
//��Addr��ָ���ĵ�ַ����RdCnt���ֽڣ��浽pRdBuf��
void Flash_ReadBytes( uint32_t Addr , uint8_t *pRdBuf , uint8_t RdCnt )
{
	while( RdCnt )
	{
		*pRdBuf = *((uint8_t * )Addr)  ;
		pRdBuf++;
		Addr++;
		RdCnt--;
	}
}

//===================================================================================
//��EERecord��¼������CRCֵ
void EeRecord_AddCRC( uint8_t *pRecord )
{
	uint8_t i;


	//M0P_SYSCTRL->PERI_CLKEN_f.CRC = 1;
	*((volatile uint32_t * )0x40002020) |= (1<<26);
		
	M0P_CRC->RESULT = 0xffff;

	for( i=0; i<HDEE_EeSize; i++ )
	{
		*((volatile uint8_t *)(&M0P_CRC->DATA)) = *pRecord;
		pRecord++;
	}

	*pRecord = M0P_CRC->RESULT & 0xff;
	pRecord++;
	*pRecord = ( M0P_CRC->RESULT>>8 ) & 0xff;
	
}

//===================================================================================
//���EERECORDβ����CRC�Ƿ���ȷ
//��ȷ����1�����󷵻�0
uint8_t EeRecord_CheckCRC( uint8_t *pRecord )
{
 	uint8_t i;


	M0P_CLOCK->PERI_CLKEN_f.CRC = 1;
	M0P_CRC->RESULT = 0xffff;

	for( i=0; i<HDEE_EeSize+2; i++ )
	{
		*((volatile uint8_t *)(&M0P_CRC->DATA)) = *pRecord;
		pRecord++;
	}

	return( M0P_CRC->RESULT_f.FLAG  );
}

//===================================================================================
//���EERECORD�������Ƿ�ȫΪFF
//ȫΪFF����1�����򷵻�0
uint8_t EeRecord_CheckFF( uint8_t *pRecord )
{
	uint8_t i;

	for( i=0; i<HDEE_EeSize+2; i++ )
	{
		if( *pRecord != 0xff )
		{
			return( 0x00 );
		}
		pRecord++;
	}

	return( 0x01 );
}

//===================================================================================
//��EE�����ָ����ַ��д��һ����¼
void EeRecord_Write( uint32_t RecordAddr, uint8_t *pRecord  )
{
	EeRecord_AddCRC( pRecord );
	Flash_WriteBytes( RecordAddr , pRecord , HDEE_EeSize+2 );
}

//===================================================================================
//��PageAddr���ڵ�ҳ�����ҳ����
//PageFlage: 0xf0����ǰ����ҳ�棬0X00����ҳ�����ݿռ���ʹ����ɣ�0xff����ҳ���ڻ�û������
void EeRecord_PageFlag_Modify( uint32_t PageAddr, uint8_t PageFlage )
{
	uint8_t  tmp8;
	uint32_t PageFlag_Addr;


	PageFlag_Addr = PageAddr | 0x1ff;
	tmp8  = *((uint8_t * )PageFlag_Addr);
	if( tmp8!=PageFlage )  //��鹤��ҳ���¼����Ƿ����
	{
		Flash_WriteBytes(  PageFlag_Addr , &PageFlage , 1 );
	}
}

//===================================================================================
// ����EE�����е����һ����Ч����
// pBuf:���һ����Ч���ݵ�����
// pRecordAddr���һ����Ч���ݵĴ洢��ַ
// �������ݴ����Ƿ��ҵ����һ����Ч��¼
uint8_t EeRecord_FindLastRecord( uint8_t *pBuf, uint32_t *pRecordAddr )
{
	uint8_t  tmp8, RecordFlag;
	uint8_t  RecordBuf[HDEE_EeSize+2];
	uint32_t tmp32, WorkPageAddr_Start, WorkPageAddr_End;


	//step1 �ҵ���ǰ�Ĺ���ҳ��
	RecordFlag = 0x00;
	WorkPageAddr_Start = 0x00;
	WorkPageAddr_End = HDEE_FlashStartAddr | 0x1ff; 
	while( WorkPageAddr_End<HDEE_FlashStopAddr )
	{
		tmp8  = *((uint8_t * )WorkPageAddr_End);
		if( tmp8==0xf0 )
		{
			WorkPageAddr_Start = WorkPageAddr_End & 0xfffffe00;
			break;
		}
		WorkPageAddr_End += 512;
	}

	//step2 �ҵ����һ����Ч��¼
	if( WorkPageAddr_Start!=0x00 ) //֮ǰ���ҵ�����ҳ��
	{
		tmp32 = WorkPageAddr_Start;
		while( tmp32+HDEE_EeSize+HDEE_BakSize+2+2 <= WorkPageAddr_End ) 
		{
			Flash_ReadBytes( tmp32, RecordBuf, HDEE_EeSize + 2 );
			tmp8 = EeRecord_CheckCRC( RecordBuf );
			if( tmp8 )
			{
				RecordFlag = 0x01;
				*pRecordAddr = tmp32;
			}
			tmp32 = tmp32 + HDEE_EeSize + 2;
		}

		if( RecordFlag )
		{
			Flash_ReadBytes( *pRecordAddr, pBuf, HDEE_EeSize );
		}
	}

	return( RecordFlag );

}

//===================================================================================
// ��RefAddr��ַ��ʼ���ҳ���һ����¼�Ĵ洢��ַ
// ��������Ϊ�µĴ洢��ַ
uint32_t EeRecord_FindNewSpace( uint32_t RefAddr )
{
	uint8_t  tmp8;
	uint8_t  RecordBuf[HDEE_EeSize+2];
	uint32_t PageEndAddr;


	PageEndAddr = RefAddr | 0x1ff;
	while( RefAddr+HDEE_EeSize+HDEE_BakSize+2 <= PageEndAddr )  
	{
		Flash_ReadBytes( RefAddr, RecordBuf, HDEE_EeSize+2 );
		tmp8 = EeRecord_CheckFF( RecordBuf );
		if( tmp8 )
		{
			break;
		}
		RefAddr += HDEE_EeSize+2;
	}

	if( RefAddr+HDEE_EeSize+HDEE_EeSize+2+2 > PageEndAddr )    //��ҳû�пռ���
	{
		RefAddr = PageEndAddr+1; //������һҳ
		if( RefAddr>=HDEE_FlashStopAddr )
		{
			RefAddr = HDEE_FlashStartAddr;
		}
		
		Flash_ErasePage( RefAddr );  //��ʽ����һҳ
	}

	return( RefAddr );

}

//===================================================================================
// ׷��һ�����ݼ�¼
// RefAddrΪ��һ����Ч��¼�ĵ�ַ
// pRecord��׷�ӵļ�¼������
void EeRecord_Append( uint32_t RefAddr, uint8_t *pRecord  )
{
	uint32_t RecordAddr;


	RecordAddr = EeRecord_FindNewSpace( RefAddr );  		//�ҵ��洢λ��
	EeRecord_Write( RecordAddr, pRecord );  				//д������
	EeRecord_PageFlag_Modify( RecordAddr, 0xF0 );  			//���ӹ���ҳ����
	RecordAddr &= 0xFFFFFE00;
	RefAddr    &= 0xFFFFFE00;
	if( RecordAddr!=RefAddr )                               //�ѻ�ҳ
	{
		EeRecord_PageFlag_Modify( RefAddr, 0x00 );  	    //�����һ��ҳ��Ĺ������
	}
}

//===================================================================================
// EE�����ݳ�ʼ��
void HDEE_Ini( void )
{
	uint32_t tmp32 , Addr32;
	uint8_t  RecordBuf[HDEE_EeSize+2];
	uint8_t  i;
	
    
	Addr32 = HDEE_FlashStartAddr;     
	while( Addr32<HDEE_FlashStopAddr )  //��ѯ�Ƿ���EE����δ������оƬ
	{
		tmp32 = *((uint32_t * )Addr32);
		if( tmp32!=0xFFFFFFFF )
		{
			break;
		}
		Addr32+=4;
	}

	if( Addr32>=HDEE_FlashStopAddr )    //оƬ�ڻ�û�н�EE����ʼ��
	{
		for( i=0; i<HDEE_EeSize; i++ )  //��ӵ�һ����¼
		{
			RecordBuf[i] = 0x00;
		}
		EeRecord_Append( HDEE_FlashStartAddr , RecordBuf );
	}
}


//===================================================================================
// ��EE����EeAddr������RdCnt���ֽڣ�����pBuf
// �������ݴ����Ƿ�ɹ�����
uint8_t HDEE_Read( uint8_t EeAddr, uint8_t *pRdBuf , uint8_t RdCnt )
{
	uint8_t tmp8;
	uint8_t  RecordBuf[HDEE_EeSize+2];
	uint32_t tmp32;

	tmp8 = EeRecord_FindLastRecord( RecordBuf , &tmp32 );
	if( tmp8 )  //���ҵ����µļ�¼
	{
		if( EeAddr+RdCnt>HDEE_EeSize ) //����ȡ�����ݳ�����¼������
		{
			tmp8=0x00;
		}
		else
		{
			while( RdCnt )
			{
				*pRdBuf = RecordBuf[EeAddr];
				EeAddr++;
				pRdBuf++;
				RdCnt--;
			}
		}
	}

	return( tmp8 );

}

//===================================================================================
// ��pWrBuf�ڵ�WrCnt������д��EE��
// �������ݴ����Ƿ�ɹ�д��
uint8_t HDEE_Write( uint8_t EeAddr, uint8_t *pWrBuf , uint8_t WrCnt )
{
	uint8_t tmp8;
	uint8_t  RecordBuf[HDEE_EeSize+2];
	uint32_t tmp32;

	tmp8 = EeRecord_FindLastRecord( RecordBuf , &tmp32 );
	if( tmp8 )  //���ҵ����µļ�¼
	{
		if( EeAddr+WrCnt>HDEE_EeSize ) //��д������ݳ�����¼������
		{
			tmp8=0x00;
		}
		else
		{
			while( WrCnt )
			{
				RecordBuf[EeAddr] = *pWrBuf;
				EeAddr++;
				pWrBuf++;
				WrCnt--;
			}
			EeRecord_Append( tmp32, RecordBuf );
		}
	}

	return( tmp8 );

}


//===================================================================================
// ����Bak���ĵ�ַ
uint32_t HDEE_BakZone_FindAddr( void )
{
	uint8_t  PageFlag;
	uint32_t PageFlag_Addr;

	//step1 �ҵ�ҳ���ǲ�Ϊ0xF0��ҳ��
	PageFlag_Addr = HDEE_FlashStartAddr | 0x1ff; 
	while( PageFlag_Addr<HDEE_FlashStopAddr )
	{
		PageFlag  = *((uint8_t * )PageFlag_Addr);
		if( PageFlag!=0xF0 )
		{
			break;
		}
		PageFlag_Addr += 512;
	}
	PageFlag_Addr = PageFlag_Addr>HDEE_FlashStopAddr ? HDEE_FlashStartAddr|0x1ff : PageFlag_Addr;
	PageFlag_Addr = PageFlag_Addr - HDEE_BakSize - 2;

	return( PageFlag_Addr );
}



//===================================================================================
// ��pWrBuf�ڵ�WrCnt������д��Bak��
// ��ע�������Ὣϵͳʱ���л�ΪRCH4M
void HDEE_BakZone_Write( uint8_t *pWrBuf , uint8_t WrCnt )
{
	uint8_t  i, *pBuf;
	uint16_t tmp16;
	uint32_t tmp32;

	//step1, �л�ϵͳʱ�ӵ�RCH4M
	M0P_CLOCK->RCH_CR = *((uint16_t *)( 0X00100C08 ) ); //4M
	if( M0P_CLOCK->SYSCTRL0_f.RCH_EN==0 )
	{
		M0P_CLOCK->SYSCTRL2 = 0X5A5A;
		M0P_CLOCK->SYSCTRL2 = 0XA5A5;
		M0P_CLOCK->SYSCTRL0_f.RCH_EN = 1;
	}
	if( M0P_CLOCK->SYSCTRL0_f.CLK_SW4_SEL!=0x00 )
	{
		M0P_CLOCK->SYSCTRL2 = 0X5A5A;
		M0P_CLOCK->SYSCTRL2 = 0XA5A5;
		M0P_CLOCK->SYSCTRL0_f.CLK_SW4_SEL = 0x00;
	}

	//step2, CALC CRC
	M0P_CLOCK->PERI_CLKEN_f.CRC = 1;
	M0P_CRC->RESULT = 0xffff;
	pBuf = pWrBuf;
	for( i=0; i<HDEE_BakSize; i++ )
	{
		*((volatile uint8_t *)(&M0P_CRC->DATA)) = *pBuf;
		pBuf++;
	}
	tmp16 = M0P_CRC->RESULT;
	

	//step3, �ҵ��洢��ַ,������
	tmp32 = HDEE_BakZone_FindAddr();
	Flash_Operation_Ini( 1 );  //write
	for( i=0; i<HDEE_BakSize; i++ )
	{
		
		*((volatile uint8_t * )(tmp32+i)) = pWrBuf[i];	
		//while( M0P_FLASH->CR_f.BUSY );  //��RAM�����в���Ҫ��ָ��
	}
	*((volatile uint8_t * )(tmp32+HDEE_BakSize+0)) = tmp16 & 0xFF;
	*((volatile uint8_t * )(tmp32+HDEE_BakSize+1)) = tmp16>>8;

	Flash_Operation_DeIni();

}

//===================================================================================
// ��Bak������RdCnt���ֽڣ�����pBuf
// ����1:����Bak��������Ч,  0:����Bak��������Ч
uint8_t HDEE_BakZone_Read( uint8_t *pRdBuf , uint8_t RdCnt )
{
	uint8_t  i;
	uint32_t tmp32;


	//step1 , �ҵ��洢��ַ
	tmp32 = HDEE_BakZone_FindAddr();

	//step2, CALC CRC
	M0P_CLOCK->PERI_CLKEN_f.CRC = 1;
	M0P_CRC->RESULT = 0xffff;
	for( i=0; i<HDEE_BakSize+2; i++ )
	{
		*((volatile uint8_t *)(&M0P_CRC->DATA)) = *((volatile uint8_t *)(tmp32+i));
	}

	//setp3, read data
	if( M0P_CRC->RESULT_f.FLAG )
	{
		for( i=0; i<RdCnt; i++ )
		{
			pRdBuf[i] = *((volatile uint8_t *)(tmp32+i));
		}
		return( 1 );
	}

	return( 0 );

}

//===================================================================================
// ���Bak������
// ��ע�������Ὣϵͳʱ���л�ΪRCH4M
	void HDEE_BakZone_Clear( void )
{
	uint32_t tmp32;


	//step1, �л�ϵͳʱ�ӵ�RCH4M
	M0P_CLOCK->RCH_CR = *((uint16_t *)( 0X00100C08 ) ); //4M
	if( M0P_CLOCK->SYSCTRL0_f.RCH_EN==0 )
	{
		M0P_CLOCK->SYSCTRL2 = 0X5A5A;
		M0P_CLOCK->SYSCTRL2 = 0XA5A5;
		M0P_CLOCK->SYSCTRL0_f.RCH_EN = 1;
	}
	if( M0P_CLOCK->SYSCTRL0_f.CLK_SW4_SEL!=0x00 )
	{
		M0P_CLOCK->SYSCTRL2 = 0X5A5A;
		M0P_CLOCK->SYSCTRL2 = 0XA5A5;
		M0P_CLOCK->SYSCTRL0_f.CLK_SW4_SEL = 0x00;
	}

	//step2 , �ҵ��洢��ַ
	tmp32 = HDEE_BakZone_FindAddr();

	//step3, ����Bak����ҳ��
	Flash_ErasePage( tmp32 ); 
}

//===================================================================================









