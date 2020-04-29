/*!
 * @file       midriver.c
 * @brief      单片机外设中间层驱动
 * @details    基于华大外设底层驱动，封装常用功能，为应用层提供外设功能相关的函数接口
		   
 * @copyright  Revised BSD License, see section LICENSE. \ref LICENSE
 * @author     Lierda-WSN-LoRaWAN-Team
 * @date       2020-03-20
 * @version    V1.0
 */
 
 
 /* Includes ------------------------------------------------------------------*/
#include <math.h>
#include "ddl.h"
#include "lpuart.h"
#include "uart.h"
#include "bt.h"
#include "clk.h"
#include "lpm.h"
#include "gpio.h"
#include "lpt.h"
#include "pca.h"
#include "driver_mid.h"
#include "LSD_LoRaWAN_ICA_Driver.h"
#include "app.h"
#include "wdt.h"
#include "commonfun.h"
 
//系统定时器相关
uint32_t uwTick=0;
uint32_t tick_1ms=0;
uint32_t tick_led_1ms=0;

bool getSensorData_Tp=0;
bool heartbeatReport_Tp=0;

//接收缓存
uint8_t uart0_RxBuf[UART0_RXBUFSIZE];
uint8_t uart1_RxBuf[UART1_RXBUFSIZE];
uint8_t lpuart_RxBuf[LPUART_RXBUFSIZE];

//发送缓存
uint8_t uart0_TxData[256];

//字节计数
int uart0_RxByteCnt=0,uart0_TxByteCnt=0;
int uart1_RxByteCnt=0,uart1_TxByteCnt=0;
int lpuart_RxByteCnt=0,lpuart_TxByteCnt=0;

//接收完成标志
volatile uint32_t RxDoneFlag_lpuart = 0;
volatile uint32_t RxDoneFlag_uart0 = 0;
volatile uint32_t RxDoneFlag_uart1 = 0;

//功能开关
uint8_t keyFunTest = 1;
uint8_t vdetectEnable = 0;

//掉电检测标志
bool powerDownFlag = 0;
bool powerOnFlag = 0;

//日志等级
extern uint8_t logLevel;

//PWM输出占空比
uint8_t u8CcaplData = 0;
uint8_t u8CcaphData = 0;

//LED状态
ledStatType ledStat;

//按键状态
bool keyDetectedFlag = 0;

//心跳周期
extern uint8_t heartbeatPeriod;

//呼吸配置
//uint8_t ledBreath[55] = {0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,2,2,2,2,2,2,3,3,3,3,3,3,4,4,4,5,5,6,7,8,9,10,11,12,14,16,19,21,23,26,29,33,37,41,50,60,70};


static volatile uint32_t u32PcaTestFlag = 0;
static stc_adt_cntstate_cfg_t stcAdt5CntState;
static stc_adt_cntstate_cfg_t stcAdt4CntState;

/******************************************************************************
 * 系统systick定时器
 ******************************************************************************/

void SysTick_init(void)
{
	stc_clk_systickcfg_t stcCfg;
	
	DDL_ZERO_STRUCT(stcCfg);
    stcCfg.enClk = ClkRCH;          //hclk/8
    stcCfg.u32LoadVal = 4000;     //主频4M, 1ms中断

    Clk_SysTickConfig(&stcCfg);
    SysTick_Config(stcCfg.u32LoadVal);	
}

void SysTick_Handler(void)
{
	static bool ledstat=0;
	static uint8_t powerLowFlag=0;
	static uint8_t tick_5s=0;	
	
	uwTick++;
	tick_1ms++;
	tick_led_1ms++;
	
	if(tick_led_1ms>=50)
	{
		tick_led_1ms=0;
		//50ms处理一次LED状态
		ledStatHandle();	
	}

	if(tick_1ms>=5000)  //传感器读取周期5s
	{
		tick_1ms = 0;
		tick_5s++;
		getSensorData_Tp = 1;  
	}		
	//5s周期喂狗
	if(tick_5s>=2)  //10s周期喂狗 
	{
		tick_5s = 0;
		Wdt_Feed();
	}	
}

uint32_t getTick(void)
{
  return uwTick;
}

/******************************************************************************
 * 串口中断回调函数
 ******************************************************************************/
void Uart0_RxIntCallback(void)
{
	if(uart0_RxByteCnt>UART0_RXBUFSIZE) uart0_RxByteCnt=0;  //重覆盖
//	if(RxDoneFlag_uart0 ==1)  //清缓存
//	{
//		RxDoneFlag_uart0 = 0;
//		uart0_RxByteCnt=0;
//		memset(uart0_RxBuf,0,UART0_RXBUFSIZE);
//	}
	
	uart0_RxBuf[uart0_RxByteCnt++] = Uart_ReceiveData(UARTCH0);
	
	Lpt_Stop();
	Lpt_ARRSet(1374);//波特率9600下，单字节数据传输时间0.83ms 分帧间隔需>0.83ms*1.5  
	Lpt_Run();  //完成每字节接收后更新分帧定时器	
}

void Uart1_RxIntCallback(void)
{
	if(uart1_RxByteCnt>UART1_RXBUFSIZE) uart1_RxByteCnt=0;  //重覆盖
//	if(RxDoneFlag_uart1 ==1)  //清缓存
//	{
//		RxDoneFlag_uart1 = 0;
//		uart1_RxByteCnt=0;
//		memset(uart1_RxBuf,0,UART1_RXBUFSIZE);
//	}
	
	uart1_RxBuf[uart1_RxByteCnt++] = Uart_ReceiveData(UARTCH1);
	
	Lpt_Stop();
	Lpt_ARRSet(1374);//波特率9600下，单字节数据传输时间0.83ms 分帧间隔需>0.83ms*1.5  
	Lpt_Run();  //完成每字节接收后更新分帧定时器		
}

void Lpuart_RxIntCallback(void)
{
	if(lpuart_RxByteCnt>=LPUART_RXBUFSIZE) lpuart_RxByteCnt=0;  //重覆盖

	if(RxDoneFlag_lpuart ==1)  //清缓存
	{
		RxDoneFlag_lpuart = 0;
		lpuart_RxByteCnt=0;
		memset(lpuart_RxBuf,0,LPUART_RXBUFSIZE);
	}	
	
	lpuart_RxBuf[lpuart_RxByteCnt++] = LPUart_ReceiveData();
	
	Adt_StopCount(AdTIM4);   //更新分帧定时器
	Adt_ClearCount(AdTIM4);
	Adt_StartCount(AdTIM4);
}

void Uart0_ErrIntCallback(void)
{
  
}

void Uart1_ErrIntCallback(void)
{
  
}

void Lpuart_ErrIntCallback(void)
{
  
}

/******************************************************************************
 * 串口发送函数
 ******************************************************************************/
void Uart0_send_string(uint8_t *str)
{
	while((*str)!=0)
	{
		Uart_SendData(UARTCH0,*str++);	
	}	
}

void Uart1_send_string(uint8_t *str)
{
	while((*str)!=0)
	{
		Uart_SendData(UARTCH1,*str++);		
	}	
}

void Lpuart_send_string(uint8_t *str)  //发送字符串
{
	while((*str)!=0)
	{
		LPUart_SendData(*str++);	
	}	
}
void Lpuart_send_data(uint8_t *pdata, uint16_t Length)  //发送固定长度数据
{  
    uint32_t i = 0;
		
	for (i = 0; i < Length; i++)
	{
		LPUart_SendData(pdata[i]);
	}
}

/******************************************************************************
 * 串口初始化函数
 ******************************************************************************/
void Uart0_init()
{
    uint16_t timer=0;
    uint32_t pclk=0;
	
    stc_uart_config_t  stcConfig;
    stc_uart_irq_cb_t stcUartIrqCb;
    stc_uart_multimode_t stcMulti;
    stc_uart_baud_config_t stcBaud;
    stc_bt_config_t stcBtConfig;
    

    DDL_ZERO_STRUCT(stcUartIrqCb);
    DDL_ZERO_STRUCT(stcMulti);
    DDL_ZERO_STRUCT(stcBaud);
    DDL_ZERO_STRUCT(stcBtConfig);

    
//    Gpio_InitIO(T1_PORT,T1_PIN,GpioDirIn); 
//    Gpio_InitIO(0,3,GpioDirOut);
//    Gpio_SetIO(0,3,1);
    
    Gpio_InitIOExt(3,5,GpioDirOut,TRUE,FALSE,FALSE,FALSE);   
    Gpio_InitIOExt(3,6,GpioDirOut,TRUE,FALSE,FALSE,FALSE); 
    
    //通道端口配置
    Gpio_SetFunc_UART0TX_P35();
    Gpio_SetFunc_UART0RX_P36();

    //外设时钟使能
    Clk_SetPeripheralGate(ClkPeripheralBt,TRUE);//模式0/2可以不使能
    Clk_SetPeripheralGate(ClkPeripheralUart0,TRUE);



    stcUartIrqCb.pfnRxIrqCb = Uart0_RxIntCallback;
    stcUartIrqCb.pfnTxIrqCb = NULL;
    stcUartIrqCb.pfnRxErrIrqCb = Uart0_ErrIntCallback;
    stcConfig.pstcIrqCb = &stcUartIrqCb;
    stcConfig.bTouchNvic = TRUE;
  

    stcConfig.enRunMode = UartMode1;//测试项，更改此处来转换4种模式测试
   

    stcMulti.enMulti_mode = UartNormal;//测试项，更改此处来转换多主机模式，mode2/3才有多主机模式

    stcConfig.pstcMultiMode = &stcMulti;

    stcBaud.bDbaud = 0u;//双倍波特率功能
    stcBaud.u32Baud = 9600u;//更新波特率位置
    stcBaud.u8Mode = UartMode1; //计算波特率需要模式参数
    pclk = Clk_GetPClkFreq();
    timer=Uart_SetBaudRate(UARTCH0,pclk,&stcBaud);

    stcBtConfig.enMD = BtMode2;
    stcBtConfig.enCT = BtTimer;
    Bt_Init(TIM0, &stcBtConfig);//调用basetimer1设置函数产生波特率
    Bt_ARRSet(TIM0,timer);
    Bt_Cnt16Set(TIM0,timer);
    Bt_Run(TIM0);

    Uart_Init(UARTCH0, &stcConfig);
    Uart_EnableIrq(UARTCH0,UartRxIrq);
    Uart_ClrStatus(UARTCH0,UartRxFull);
    Uart_EnableFunc(UARTCH0,UartRx);
	
}

void Uart1_init()
{
    uint16_t timer=0;
    uint32_t pclk=0;
	
    stc_uart_config_t  stcConfig;
    stc_uart_irq_cb_t stcUartIrqCb;
    stc_uart_multimode_t stcMulti;
    stc_uart_baud_config_t stcBaud;
    stc_bt_config_t stcBtConfig;
    

    DDL_ZERO_STRUCT(stcUartIrqCb);
    DDL_ZERO_STRUCT(stcMulti);
    DDL_ZERO_STRUCT(stcBaud);
    DDL_ZERO_STRUCT(stcBtConfig);

    
//    Gpio_InitIO(T1_PORT,T1_PIN,GpioDirIn); 
//    Gpio_InitIO(0,3,GpioDirOut);
//    Gpio_SetIO(0,3,1);
    
    Gpio_InitIOExt(0,1,GpioDirOut,TRUE,FALSE,FALSE,FALSE);   
    Gpio_InitIOExt(0,2,GpioDirIn,TRUE,FALSE,FALSE,FALSE); 
    
    //通道端口配置
	Gpio_SetFunc_UART1_TXD_P01();
    Gpio_SetFunc_UART1_RXD_P02();

    //外设时钟使能
    Clk_SetPeripheralGate(ClkPeripheralBt,TRUE);//模式0/2可以不使能
    Clk_SetPeripheralGate(ClkPeripheralUart1,TRUE);



    stcUartIrqCb.pfnRxIrqCb = Uart1_RxIntCallback;
    stcUartIrqCb.pfnTxIrqCb = NULL;
    stcUartIrqCb.pfnRxErrIrqCb = Uart1_ErrIntCallback;
    stcConfig.pstcIrqCb = &stcUartIrqCb;
    stcConfig.bTouchNvic = TRUE;
  

    stcConfig.enRunMode = UartMode1;//测试项，更改此处来转换4种模式测试
   

    stcMulti.enMulti_mode = UartNormal;//测试项，更改此处来转换多主机模式，mode2/3才有多主机模式

    stcConfig.pstcMultiMode = &stcMulti;

    stcBaud.bDbaud = 0u;//双倍波特率功能
    stcBaud.u32Baud = 9600u;//更新波特率位置
    stcBaud.u8Mode = UartMode1; //计算波特率需要模式参数
    pclk = Clk_GetPClkFreq();
    timer=Uart_SetBaudRate(UARTCH1,pclk,&stcBaud);

    stcBtConfig.enMD = BtMode2;
    stcBtConfig.enCT = BtTimer;
    Bt_Init(TIM1, &stcBtConfig);//调用basetimer1设置函数产生波特率
    Bt_ARRSet(TIM1,timer);
    Bt_Cnt16Set(TIM1,timer);
    Bt_Run(TIM1);

    Uart_Init(UARTCH1, &stcConfig);
    Uart_EnableIrq(UARTCH1,UartRxIrq);
    Uart_ClrStatus(UARTCH1,UartRxFull);
    Uart_EnableFunc(UARTCH1,UartRx);
	
}

void Lpuart1_init()
{
    uint16_t u16timer;
    uint32_t u32sclk;

    stc_lpuart_config_t  stcConfig;
    stc_lpuart_irq_cb_t stcLPUartIrqCb;
    stc_lpuart_multimode_t stcMulti;
    stc_lpuart_sclk_sel_t  stcLpuart_clk;
    stc_lpuart_mode_t       stcRunMode;
    stc_lpuart_baud_config_t  stcBaud;
    stc_bt_config_t stcBtConfig;
    
    DDL_ZERO_STRUCT(stcConfig);
    DDL_ZERO_STRUCT(stcLPUartIrqCb);
    DDL_ZERO_STRUCT(stcMulti);
    DDL_ZERO_STRUCT(stcBtConfig);
    
    Clk_SetPeripheralGate(ClkPeripheralLpUart,TRUE);//使能LPUART时钟
    Clk_SetPeripheralGate(ClkPeripheralBt,TRUE);
    
    //通道端口配置
    Gpio_InitIOExt(2,5,GpioDirOut,TRUE,FALSE,FALSE,FALSE);
    Gpio_InitIOExt(2,6,GpioDirOut,TRUE,FALSE,FALSE,FALSE);

    Gpio_SetFunc_UART2RX_P25();
    Gpio_SetFunc_UART2TX_P26();

    //Gpio_InitIO(T1_PORT,T1_PIN,GpioDirIn);  //?
    
    stcLpuart_clk.enSclk_sel = LPUart_Pclk;//LPUart_Rcl;
    stcLpuart_clk.enSclk_Prs = LPUartDiv1;
    stcConfig.pstcLpuart_clk = &stcLpuart_clk;

    stcRunMode.enLpMode = LPUartNoLPMode;//正常工作模式或低功耗工作模式配置
    stcRunMode.enMode   = LPUartMode1;
    stcConfig.pstcRunMode = &stcRunMode;

    stcLPUartIrqCb.pfnRxIrqCb = Lpuart_RxIntCallback;
    stcLPUartIrqCb.pfnTxIrqCb = NULL;
    stcLPUartIrqCb.pfnRxErrIrqCb = Lpuart_ErrIntCallback;
    stcConfig.pstcIrqCb = &stcLPUartIrqCb;
    stcConfig.bTouchNvic = TRUE;

    stcMulti.enMulti_mode = LPUartNormal;//只有模式2/3才有多主机模式
    stcConfig.pstcMultiMode = &stcMulti;
   
    LPUart_EnableIrq(LPUartRxIrq);

    LPUart_Init(&stcConfig);

    if(LPUart_Pclk == stcLpuart_clk.enSclk_sel)
        u32sclk = Clk_GetPClkFreq();
    else if(LPUart_Rcl == stcLpuart_clk.enSclk_sel)
        u32sclk = 38400;//此处建议用户使用内部38.4K时钟，如果用户使用32.768K时钟的，此处更新成32768
    else
        u32sclk = 32768;
      
    stcBaud.u32Baud = 9600;
    stcBaud.bDbaud = 0;
    stcBaud.u8LpMode = LPUartNoLPMode;
    stcBaud.u8Mode = LPUartMode1;
    u16timer = LPUart_SetBaudRate(u32sclk,stcLpuart_clk.enSclk_Prs,&stcBaud);
    stcBtConfig.enMD = BtMode2;
    stcBtConfig.enCT = BtTimer;
    stcBtConfig.enTog = BtTogEnable;
    Bt_Init(TIM2, &stcBtConfig);//调用basetimer2设置函数产生波特率
    Bt_ARRSet(TIM2,u16timer);
    Bt_Cnt16Set(TIM2,u16timer);
    Bt_Run(TIM2);

    LPUart_EnableFunc(LPUartRx);    	
}
	
/******************************************************************************
 * 分帧定时器中断回调函数
 ******************************************************************************/
void LptInt(void)
{	
	if (TRUE == Lpt_GetIntFlag())
	{
		Lpt_ClearIntFlag();
		RxDoneFlag_uart1 = 1;  //串口接收完成标志
		RxDoneFlag_uart0 = 1;  //串口接收完成标志
		Lpt_Stop();
	}
}

/******************************************************************************
 * 串口分帧定时器初始化函数
 ******************************************************************************/
void Lptimer_Init(void)
{
	stc_lpt_config_t stcConfig;
    en_result_t      enResult = Error;
    //uint16_t         u16ArrData = 0;
 
    stc_lpm_config_t stcLpmCfg;
    
//    Gpio_InitIO(2, 5, GpioDirIn);
//    Gpio_InitIO(2, 6, GpioDirOut);
//    Gpio_SetIO(2, 6, FALSE);
    
	//CLK INIT
    stc_clk_config_t stcClkCfg;
    stcClkCfg.enClkSrc  = ClkRCH;
    stcClkCfg.enHClkDiv = ClkDiv1;
    stcClkCfg.enPClkDiv = ClkDiv1;
    
    Clk_Init(&stcClkCfg);
    
    //使能Lpt外设时钟
    Clk_SetPeripheralGate(ClkPeripheralLpTim, TRUE);
	
    stcConfig.enGateP  = LptPositive; 
    stcConfig.enGate   = LptGateDisable;
    stcConfig.enTckSel = LptPCLK;
    stcConfig.enTog    = LptTogDisable;
    stcConfig.enCT     = LptTimer;
    stcConfig.enMD     = LptMode2;
    
    stcConfig.pfnLpTimCb = LptInt;
    
    if (Ok != Lpt_Init(&stcConfig))
    {
        enResult = Error;
    }
    
    //Lpm Cfg
    stcLpmCfg.enSEVONPEND   = SevPndDisable;
    stcLpmCfg.enSLEEPDEEP   = SlpDpDisable;
    stcLpmCfg.enSLEEPONEXIT = SlpExtDisable;
    Lpm_Config(&stcLpmCfg);
    
    //Lpt 中断使能
    Lpt_ClearIntFlag();
    Lpt_EnableIrq();
    EnableNvic(LPTIM_IRQn, 0, TRUE);
    
    
    //设置重载值，计数初值，启动计数
//    Lpt_ARRSet(0);
//    Lpt_Run();
}

/******************************************************************************
 * PCA定时器初始化
 ******************************************************************************/
void Pca_Timer_Init(void)
{
	  stc_pca_config_t stcConfig;
    stc_pca_capmodconfig_t stcModConfig;
    en_result_t      enResult = Error;
    uint16_t         u16InitCntData = 0;
	uint16_t         u16CcapData = 0x009B;    
	
	Clk_SetPeripheralGate(ClkPeripheralPca, TRUE);  //开启PCA外设时钟
    stcConfig.enCIDL = IdleGoon; 
    stcConfig.enWDTE = PCAWDTDisable;
    stcConfig.enCPS  = PCAPCLKDiv32; 
    
    stcConfig.pfnPcaCb = PcaInt;
    
    stcModConfig.enECOM = ECOMEnable;//允许比较功能
    stcModConfig.enCAPP = CAPPDisable;
    stcModConfig.enCAPN = CAPNDisable;
    stcModConfig.enMAT  = MATEnable;//允许匹配
    stcModConfig.enTOG  = TOGDisable;
    stcModConfig.enPWM  = PCAPWMDisable;
     
    if (Ok != Pca_Init(&stcConfig))
    {
        enResult = Error;
    }
    if (Ok != Pca_CapModConfig(Module2, &stcModConfig))
    {
        enResult = Error;
    }
    
    //PCA 中断使能
    Pca_ClearIntFlag(Module2);
    Pca_EnableIrq(Module2);    
    EnableNvic(PCA_IRQn, 3, TRUE);

    Pca_CapData16Set(Module2, u16CcapData);//比较捕获寄存器设置
    //Pca_Cnt16Set(u16InitCntData);
	Pca_Run();

}

/******************************************************************************
 * ADTimer5中断服务函数-掉电检测
 ******************************************************************************/
void Adt5CompACalllback(void)
{
	static uint8_t powerLowFlag=0;
	
    Adt_GetCntState(AdTIM5, &stcAdt5CntState);

	Adt_SetCompareValue(AdTIM5, AdtCompareA, 0x00ff);    //??????????A??


	//50ms检测一次掉电
	if(vdetectEnable)
	{
		if(Gpio_GetIO(0, 3)==0 && powerLowFlag==1) 
		{	
			powerDown_Task();
		}
		if(Gpio_GetIO(0, 3)==0) 
		{
			powerLowFlag = 1;
		}
		
	}
}

/******************************************************************************
 * ADTimer4中断服务函数-Lpuart分帧
 ******************************************************************************/
void Adt4OVFCalllback(void)
{
	//static bool ledstat=0;
	static uint8_t powerLowFlag=0;

	Adt_ClearIrqFlag(AdTIM4,AdtOVFIrq);  //清中断标志
	RxDoneFlag_lpuart = 1;  //串口接收完成标志
	Adt_StopCount(AdTIM4);  //停止计数
	
	//功能验证时使用，确认分帧时刻
	//ledstat = !ledstat;  
	//Gpio_SetIO(3,2,ledstat);
		

}

/******************************************************************************
 * ADTimer5初始化-掉电检测
 ******************************************************************************/
void Adtimer5_init(void)
{
	en_adt_unit_t enAdt;
    uint16_t u16Period;
    en_adt_compare_t enAdtCompare;
    uint16_t u16Compare;
    
    stc_adt_basecnt_cfg_t stcAdtBaseCntCfg;
    stc_adt_CHxX_port_cfg_t stcAdtTIMACfg;
    stc_adt_CHxX_port_cfg_t stcAdtTIMBCfg;
    stc_adt_port_trig_cfg_t stcAdtPortTrig;
	
    DDL_ZERO_STRUCT(stcAdtBaseCntCfg);
    DDL_ZERO_STRUCT(stcAdtTIMACfg);
    DDL_ZERO_STRUCT(stcAdtPortTrig);
    
    Clk_SetPeripheralGate(ClkPeripheralAdt, TRUE);//ADT??????

    enAdt = AdTIM5;

    stcAdtBaseCntCfg.enCntMode = AdtSawtoothMode;
    stcAdtBaseCntCfg.enCntDir = AdtCntUp;
    stcAdtBaseCntCfg.enCntClkDiv = AdtClkPClk0Div64;
    Adt_Init(enAdt, &stcAdtBaseCntCfg);                      //ADT????????????

    u16Period = 0x0C35;
    Adt_SetPeriod(enAdt, u16Period);                         //????

    enAdtCompare = AdtCompareA;
    u16Compare = 0x00ff;
    Adt_SetCompareValue(enAdt, enAdtCompare, u16Compare);    //??????????A??
    
    Adt_ConfigIrq(enAdt, AdtCMAIrq, TRUE, Adt5CompACalllback);
    
    Adt_StartCount(enAdt);
}

/******************************************************************************
 * ADTimer4初始化-Lpuart分帧
 ******************************************************************************/
void Adtimer4_init(void)
{
	en_adt_unit_t enAdt;
    uint16_t u16Period;
    en_adt_compare_t enAdtCompare;
    uint16_t u16Compare;
    
    stc_adt_basecnt_cfg_t stcAdtBaseCntCfg;
    stc_adt_CHxX_port_cfg_t stcAdtTIMACfg;
    stc_adt_CHxX_port_cfg_t stcAdtTIMBCfg;
    stc_adt_port_trig_cfg_t stcAdtPortTrig;
	
    DDL_ZERO_STRUCT(stcAdtBaseCntCfg);
    DDL_ZERO_STRUCT(stcAdtTIMACfg);
    DDL_ZERO_STRUCT(stcAdtPortTrig);
    
    Clk_SetPeripheralGate(ClkPeripheralAdt, TRUE);

    enAdt = AdTIM4;

    stcAdtBaseCntCfg.enCntMode = AdtSawtoothMode;
    stcAdtBaseCntCfg.enCntDir = AdtCntUp;
    stcAdtBaseCntCfg.enCntClkDiv = AdtClkPClk0;
    Adt_Init(enAdt, &stcAdtBaseCntCfg);                      

	u16Period = 0x4E00;										//5ms帧超时时间 不分频
	//u16Period = 0x0138;                                    //5ms帧超时时间 64分频(有异常：定时器会提前发生中断)
    Adt_SetPeriod(enAdt, u16Period);                         //
    
    Adt_ConfigIrq(enAdt, AdtOVFIrq, TRUE, Adt4OVFCalllback);
}

/******************************************************************************
 * LoRaWAN模组GPIO初始化
 ******************************************************************************/
void Gpio_lora_Init(void)
{
	Gpio_InitIOExt(1,5,GpioDirIn,FALSE,FALSE,FALSE,FALSE);  //STAT
	Gpio_InitIOExt(1,4,GpioDirOut,FALSE,FALSE,FALSE,FALSE);  //MODE
	Gpio_InitIOExt(2,3,GpioDirIn,FALSE,FALSE,FALSE,FALSE);  //BUSY
	Gpio_InitIOExt(2,4,GpioDirOut,FALSE,FALSE,FALSE,FALSE);  //RST
	Gpio_InitIOExt(3,4,GpioDirOut,FALSE,FALSE,FALSE,FALSE);  //WAKE
}

/******************************************************************************
 * 按键初始化
 ******************************************************************************/
void Gpio_key_init(void)
{
    Gpio_InitIOExt(3, 3, GpioDirIn, TRUE, FALSE, FALSE, 0);  //KEY1
	
    //??GPIO??
    Gpio_ClearIrq(3, 3);
    Gpio_EnableIrq(3, 3, GpioIrqFalling);
    EnableNvic(PORT3_IRQn, DDL_IRQ_LEVEL_DEFAULT, TRUE);
}

/*****************************************************************************
 * LED初始化
 ******************************************************************************/
void Pca_led_init(void)
{
    stc_pca_config_t stcConfig;
    stc_pca_capmodconfig_t stcModConfig;
    en_result_t      enResult = Ok;
    uint32_t         u32Cnt; 

	//Gpio_InitIOExt(3, 2, GpioDirOut, FALSE, FALSE, TRUE, FALSE);  //LED1
	
	Clk_SetPeripheralGate(ClkPeripheralPca, TRUE);  //开启PCA外设时钟
    Gpio_SetFunc_PCA_CH4_P32(0);
    
    stcConfig.enCIDL = IdleGoon; 
    stcConfig.enWDTE = PCAWDTDisable;
    stcConfig.enCPS  = PCAPCLKDiv32; 
    
    stcConfig.pfnPcaCb = PcaInt;
    
    stcModConfig.enECOM = ECOMEnable;				
    stcModConfig.enCAPP = CAPPDisable;
    stcModConfig.enCAPN = CAPNDisable;
    stcModConfig.enMAT  = MATDisable;
    stcModConfig.enTOG  = TOGDisable;
    stcModConfig.enPWM  = PCAPWMEnable;
    
    if (Ok != Pca_Init(&stcConfig))
    {
        enResult = Error;
    }
    if (Ok != Pca_CapModConfig(Module4, &stcModConfig))
    {
        enResult = Error;
    }
    
    Pca_CapDataLSet(Module4, u8CcaplData);
    Pca_CapDataHSet(Module4, u8CcaphData);
    Pca_Run();	
	ledStat = OFF;
}

/******************************************************************************
 * 掉电检测初始化
 ******************************************************************************/
void Gpio_vdetect_init(void)
{
    Gpio_InitIOExt(0, 3, GpioDirIn, FALSE, FALSE, FALSE, 0);  //VDETECT
	
//    Gpio_ClearIrq(0, 3);
//    Gpio_EnableIrq(0, 3, GpioIrqFalling);
//    EnableNvic(PORT0_IRQn, DDL_IRQ_LEVEL_DEFAULT, TRUE);
}

/******************************************************************************
 * GPIO中断服务函数
 ******************************************************************************/
void Gpio_IRQHandler(uint8_t u8Param)
{
	unsigned int i=0;
	
	if(u8Param == 3)  //KEY1
	{		
		if(keyFunTest)
		{
			for(i=0;i<25000;i++);
			if(Gpio_GetIO(3, 3)==0)
			{
				for(i=0;i<25000;i++);
				if(Gpio_GetIO(3, 3)==0)	
				{
					ledStat = BLINK;	
					printf("Key pressed down!\r\n");
					keyDetectedFlag = 1;			
				}					
			}
		}
		Gpio_ClearIrq(3,3);	
	}
}

/******************************************************************************
 * PCA中断服务函数
 ******************************************************************************/
void PcaInt(void)
{
	static uint8_t powerLowFlag=0;
	static uint8_t tick_5s=0;
	uint16_t heartbeatData = 0;
	char hexData[2] = "00";

	if (TRUE == Pca_GetCntIntFlag())
	{
		Pca_ClearCntIntFlag();
	}
	if (TRUE == Pca_GetIntFlag(Module2))
	{
			Pca_ClearIntFlag(Module2);
			RxDoneFlag_lpuart = 1;  //串口接收完成标志
			Pca_Stop();
			
			//下行指令识别
			if(lpuart_RxByteCnt==4)
			{
				if(lpuart_RxBuf[0]=='C'||lpuart_RxBuf[1]=='4')  //心跳周期设置
				{
						//获取心跳周期
						hexData[0]=lpuart_RxBuf[2];
						hexData[1]=lpuart_RxBuf[3];
						heartbeatData = hexToDec(hexData);
						printf("set heart-beat period to %d hour\r\n",heartbeatData);  
						heartbeatPeriod = heartbeatData;
				}
			}
			
	}
	if (TRUE == Pca_GetIntFlag(Module4))
    {
        Pca_ClearIntFlag(Module4);
    }
}

/******************************************************************************
 * LoRaWAN模组复位
 ******************************************************************************/
void Lora_reset(void)
{
	SET_WAKE_HIGH;
	SET_MODE_HIGH;
	SET_RESET_LOW;
	system_delay_ms(50);	
	SET_RESET_HIGH;
	system_delay_ms(200);	
}

/******************************************************************************
 * 看门狗中断服务函数
 ******************************************************************************/
void WdtCallback(void)
{
    // comment following to demonstrate the hardware watchdog reset

}


/******************************************************************************
 * 看门狗初始化
 ******************************************************************************/
void Wdt_init(void)
{
	stc_wdt_config_t  stcWdt_Config;
    stc_lpm_config_t stcLpmCfg;
    
    DDL_ZERO_STRUCT(stcWdt_Config);
    DDL_ZERO_STRUCT(stcLpmCfg);
    
    stcLpmCfg.enSLEEPDEEP = SlpDpDisable;//SlpDpEnable;
    stcLpmCfg.enSLEEPONEXIT = SlpExtDisable;
    
    stcWdt_Config.u8LoadValue = 0x0E;//26.2s
    stcWdt_Config.enResetEnable = WRESET_EN;//WRESET_EN;////
    stcWdt_Config.pfnWdtIrqCb = WdtCallback;
    
    Clk_SetPeripheralGate(ClkPeripheralWdt,TRUE);//
    Wdt_Init(&stcWdt_Config);
    
    Wdt_Start();
}

