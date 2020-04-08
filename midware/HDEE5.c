//===================================================================================
//
//Demo for EE REV05
//
//===================================================================================

#include "hc32l110.h"
#include "HDEE5.h"

//===================================================================================
//初始化Flash操作相关的寄存器
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

	M0P_FLASH->BYPASS=0x5a5a;  M0P_FLASH->BYPASS=0xa5a5;  M0P_FLASH->CR_f.OP = Mode;  //擦、写、片擦
	M0P_FLASH->BYPASS=0x5a5a;  M0P_FLASH->BYPASS=0xa5a5;  M0P_FLASH->SLOCK = HDEE_FlashLock;  //最高4个Page
}

//===================================================================================
// 禁止操作Flash
void Flash_Operation_DeIni( void )
{
	M0P_FLASH->BYPASS=0x5a5a;  M0P_FLASH->BYPASS=0xa5a5;  M0P_FLASH->CR_f.OP = 0x00;
	M0P_FLASH->BYPASS=0x5a5a;  M0P_FLASH->BYPASS=0xa5a5;  M0P_FLASH->SLOCK = 0x00;

	M0P_CLOCK->PERI_CLKEN_f.FLASH = 0;
	__enable_irq() ;
}

//===================================================================================
//擦除Addr所在的页面
void Flash_ErasePage( uint32_t Addr )
{
	Flash_Operation_Ini( 2 );  //page erase

	*((uint8_t * )Addr) = 0x00;  //dummy write
	//while( M0P_FLASH->CR_f.BUSY );   //在RAM中运行才需要该指令

	Flash_Operation_DeIni();
}

//===================================================================================
//将pWrBuf中的WrCnt个字节写到Addr所指定的地址
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
//从Addr所指定的地址读出RdCnt个字节，存到pRdBuf中
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
//向EERecord记录中增加CRC值
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
//检查EERECORD尾部的CRC是否正确
//正确返回1，错误返回0
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
//检查EERECORD的内容是否全为FF
//全为FF返回1，否则返回0
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
//向EE区域的指定地址，写入一条记录
void EeRecord_Write( uint32_t RecordAddr, uint8_t *pRecord  )
{
	EeRecord_AddCRC( pRecord );
	Flash_WriteBytes( RecordAddr , pRecord , HDEE_EeSize+2 );
}

//===================================================================================
//在PageAddr所在的页面添加页面标记
//PageFlage: 0xf0代表当前工作页面，0X00代表本页面数据空间已使用完成，0xff代表本页面内还没有数据
void EeRecord_PageFlag_Modify( uint32_t PageAddr, uint8_t PageFlage )
{
	uint8_t  tmp8;
	uint32_t PageFlag_Addr;


	PageFlag_Addr = PageAddr | 0x1ff;
	tmp8  = *((uint8_t * )PageFlag_Addr);
	if( tmp8!=PageFlage )  //检查工作页面记录标记是否存在
	{
		Flash_WriteBytes(  PageFlag_Addr , &PageFlage , 1 );
	}
}

//===================================================================================
// 查找EE区域中的最后一条有效数据
// pBuf:最后一条有效数据的内容
// pRecordAddr最后一条有效数据的存储地址
// 返回数据代表是否找到最后一条有效记录
uint8_t EeRecord_FindLastRecord( uint8_t *pBuf, uint32_t *pRecordAddr )
{
	uint8_t  tmp8, RecordFlag;
	uint8_t  RecordBuf[HDEE_EeSize+2];
	uint32_t tmp32, WorkPageAddr_Start, WorkPageAddr_End;


	//step1 找到当前的工作页面
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

	//step2 找到最后一条有效记录
	if( WorkPageAddr_Start!=0x00 ) //之前已找到工作页面
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
// 从RefAddr地址开始，找出下一条记录的存储地址
// 返回数据为新的存储地址
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

	if( RefAddr+HDEE_EeSize+HDEE_EeSize+2+2 > PageEndAddr )    //本页没有空间了
	{
		RefAddr = PageEndAddr+1; //换到下一页
		if( RefAddr>=HDEE_FlashStopAddr )
		{
			RefAddr = HDEE_FlashStartAddr;
		}
		
		Flash_ErasePage( RefAddr );  //格式化下一页
	}

	return( RefAddr );

}

//===================================================================================
// 追加一条数据记录
// RefAddr为上一条有效记录的地址
// pRecord待追加的记录的内容
void EeRecord_Append( uint32_t RefAddr, uint8_t *pRecord  )
{
	uint32_t RecordAddr;


	RecordAddr = EeRecord_FindNewSpace( RefAddr );  		//找到存储位置
	EeRecord_Write( RecordAddr, pRecord );  				//写入数据
	EeRecord_PageFlag_Modify( RecordAddr, 0xF0 );  			//增加工作页面标记
	RecordAddr &= 0xFFFFFE00;
	RefAddr    &= 0xFFFFFE00;
	if( RecordAddr!=RefAddr )                               //已换页
	{
		EeRecord_PageFlag_Modify( RefAddr, 0x00 );  	    //清除上一个页面的工作标记
	}
}

//===================================================================================
// EE区数据初始化
void HDEE_Ini( void )
{
	uint32_t tmp32 , Addr32;
	uint8_t  RecordBuf[HDEE_EeSize+2];
	uint8_t  i;
	
    
	Addr32 = HDEE_FlashStartAddr;     
	while( Addr32<HDEE_FlashStopAddr )  //查询是否是EE区尚未开启的芯片
	{
		tmp32 = *((uint32_t * )Addr32);
		if( tmp32!=0xFFFFFFFF )
		{
			break;
		}
		Addr32+=4;
	}

	if( Addr32>=HDEE_FlashStopAddr )    //芯片内还没有将EE区初始化
	{
		for( i=0; i<HDEE_EeSize; i++ )  //添加第一条记录
		{
			RecordBuf[i] = 0x00;
		}
		EeRecord_Append( HDEE_FlashStartAddr , RecordBuf );
	}
}


//===================================================================================
// 从EE区的EeAddr处读出RdCnt个字节，存入pBuf
// 返回数据代表是否成功读出
uint8_t HDEE_Read( uint8_t EeAddr, uint8_t *pRdBuf , uint8_t RdCnt )
{
	uint8_t tmp8;
	uint8_t  RecordBuf[HDEE_EeSize+2];
	uint32_t tmp32;

	tmp8 = EeRecord_FindLastRecord( RecordBuf , &tmp32 );
	if( tmp8 )  //已找到最新的记录
	{
		if( EeAddr+RdCnt>HDEE_EeSize ) //待读取的数据超出记录的总数
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
// 将pWrBuf内的WrCnt个数据写入EE区
// 返回数据代表是否成功写入
uint8_t HDEE_Write( uint8_t EeAddr, uint8_t *pWrBuf , uint8_t WrCnt )
{
	uint8_t tmp8;
	uint8_t  RecordBuf[HDEE_EeSize+2];
	uint32_t tmp32;

	tmp8 = EeRecord_FindLastRecord( RecordBuf , &tmp32 );
	if( tmp8 )  //已找到最新的记录
	{
		if( EeAddr+WrCnt>HDEE_EeSize ) //待写入的数据超出记录的总数
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
// 查找Bak区的地址
uint32_t HDEE_BakZone_FindAddr( void )
{
	uint8_t  PageFlag;
	uint32_t PageFlag_Addr;

	//step1 找到页面标记不为0xF0的页面
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
// 将pWrBuf内的WrCnt个数据写入Bak区
// 备注：函数会将系统时钟切换为RCH4M
void HDEE_BakZone_Write( uint8_t *pWrBuf , uint8_t WrCnt )
{
	uint8_t  i, *pBuf;
	uint16_t tmp16;
	uint32_t tmp32;

	//step1, 切换系统时钟到RCH4M
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
	

	//step3, 找到存储地址,并保存
	tmp32 = HDEE_BakZone_FindAddr();
	Flash_Operation_Ini( 1 );  //write
	for( i=0; i<HDEE_BakSize; i++ )
	{
		
		*((volatile uint8_t * )(tmp32+i)) = pWrBuf[i];	
		//while( M0P_FLASH->CR_f.BUSY );  //在RAM中运行才需要该指令
	}
	*((volatile uint8_t * )(tmp32+HDEE_BakSize+0)) = tmp16 & 0xFF;
	*((volatile uint8_t * )(tmp32+HDEE_BakSize+1)) = tmp16>>8;

	Flash_Operation_DeIni();

}

//===================================================================================
// 从Bak区读出RdCnt个字节，存入pBuf
// 返回1:代表Bak区数据有效,  0:代表Bak区数据无效
uint8_t HDEE_BakZone_Read( uint8_t *pRdBuf , uint8_t RdCnt )
{
	uint8_t  i;
	uint32_t tmp32;


	//step1 , 找到存储地址
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
// 清除Bak区数据
// 备注：函数会将系统时钟切换为RCH4M
	void HDEE_BakZone_Clear( void )
{
	uint32_t tmp32;


	//step1, 切换系统时钟到RCH4M
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

	//step2 , 找到存储地址
	tmp32 = HDEE_BakZone_FindAddr();

	//step3, 擦除Bak所在页面
	Flash_ErasePage( tmp32 ); 
}

//===================================================================================









