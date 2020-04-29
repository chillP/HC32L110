/*!
 * @file       midriver.c
 * @brief      ��Ƭ�������м������
 * @details    ���ڻ�������ײ���������װ���ù��ܣ�ΪӦ�ò��ṩ���蹦����صĺ����ӿ�
		   
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
 
//ϵͳ��ʱ�����
uint32_t uwTick=0;
uint32_t tick_1ms=0;
uint32_t tick_led_1ms=0;

bool getSensorData_Tp=0;
bool heartbeatReport_Tp=0;

//���ջ���
uint8_t uart0_RxBuf[UART0_RXBUFSIZE];
uint8_t uart1_RxBuf[UART1_RXBUFSIZE];
uint8_t lpuart_RxBuf[LPUART_RXBUFSIZE];

//���ͻ���
uint8_t uart0_TxData[256];

//�ֽڼ���
int uart0_RxByteCnt=0,uart0_TxByteCnt=0;
int uart1_RxByteCnt=0,uart1_TxByteCnt=0;
int lpuart_RxByteCnt=0,lpuart_TxByteCnt=0;

//������ɱ�־
volatile uint32_t RxDoneFlag_lpuart = 0;
volatile uint32_t RxDoneFlag_uart0 = 0;
volatile uint32_t RxDoneFlag_uart1 = 0;

//���ܿ���
uint8_t keyFunTest = 1;
uint8_t vdetectEnable = 0;

//�������־
bool powerDownFlag = 0;
bool powerOnFlag = 0;

//��־�ȼ�
extern uint8_t logLevel;

//PWM���ռ�ձ�
uint8_t u8CcaplData = 0;
uint8_t u8CcaphData = 0;

//LED״̬
ledStatType ledStat;

//����״̬
bool keyDetectedFlag = 0;

//��������
extern uint8_t heartbeatPeriod;

//��������
//uint8_t ledBreath[55] = {0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,2,2,2,2,2,2,3,3,3,3,3,3,4,4,4,5,5,6,7,8,9,10,11,12,14,16,19,21,23,26,29,33,37,41,50,60,70};


static volatile uint32_t u32PcaTestFlag = 0;
static stc_adt_cntstate_cfg_t stcAdt5CntState;
static stc_adt_cntstate_cfg_t stcAdt4CntState;

/******************************************************************************
 * ϵͳsystick��ʱ��
 ******************************************************************************/

void SysTick_init(void)
{
	stc_clk_systickcfg_t stcCfg;
	
	DDL_ZERO_STRUCT(stcCfg);
    stcCfg.enClk = ClkRCH;          //hclk/8
    stcCfg.u32LoadVal = 4000;     //��Ƶ4M, 1ms�ж�

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
		//50ms����һ��LED״̬
		ledStatHandle();	
	}

	if(tick_1ms>=5000)  //��������ȡ����5s
	{
		tick_1ms = 0;
		tick_5s++;
		getSensorData_Tp = 1;  
	}		
	//5s����ι��
	if(tick_5s>=2)  //10s����ι�� 
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
 * �����жϻص�����
 ******************************************************************************/
void Uart0_RxIntCallback(void)
{
	if(uart0_RxByteCnt>UART0_RXBUFSIZE) uart0_RxByteCnt=0;  //�ظ���
//	if(RxDoneFlag_uart0 ==1)  //�建��
//	{
//		RxDoneFlag_uart0 = 0;
//		uart0_RxByteCnt=0;
//		memset(uart0_RxBuf,0,UART0_RXBUFSIZE);
//	}
	
	uart0_RxBuf[uart0_RxByteCnt++] = Uart_ReceiveData(UARTCH0);
	
	Lpt_Stop();
	Lpt_ARRSet(1374);//������9600�£����ֽ����ݴ���ʱ��0.83ms ��֡�����>0.83ms*1.5  
	Lpt_Run();  //���ÿ�ֽڽ��պ���·�֡��ʱ��	
}

void Uart1_RxIntCallback(void)
{
	if(uart1_RxByteCnt>UART1_RXBUFSIZE) uart1_RxByteCnt=0;  //�ظ���
//	if(RxDoneFlag_uart1 ==1)  //�建��
//	{
//		RxDoneFlag_uart1 = 0;
//		uart1_RxByteCnt=0;
//		memset(uart1_RxBuf,0,UART1_RXBUFSIZE);
//	}
	
	uart1_RxBuf[uart1_RxByteCnt++] = Uart_ReceiveData(UARTCH1);
	
	Lpt_Stop();
	Lpt_ARRSet(1374);//������9600�£����ֽ����ݴ���ʱ��0.83ms ��֡�����>0.83ms*1.5  
	Lpt_Run();  //���ÿ�ֽڽ��պ���·�֡��ʱ��		
}

void Lpuart_RxIntCallback(void)
{
	if(lpuart_RxByteCnt>=LPUART_RXBUFSIZE) lpuart_RxByteCnt=0;  //�ظ���

	if(RxDoneFlag_lpuart ==1)  //�建��
	{
		RxDoneFlag_lpuart = 0;
		lpuart_RxByteCnt=0;
		memset(lpuart_RxBuf,0,LPUART_RXBUFSIZE);
	}	
	
	lpuart_RxBuf[lpuart_RxByteCnt++] = LPUart_ReceiveData();
	
	Adt_StopCount(AdTIM4);   //���·�֡��ʱ��
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
 * ���ڷ��ͺ���
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

void Lpuart_send_string(uint8_t *str)  //�����ַ���
{
	while((*str)!=0)
	{
		LPUart_SendData(*str++);	
	}	
}
void Lpuart_send_data(uint8_t *pdata, uint16_t Length)  //���͹̶���������
{  
    uint32_t i = 0;
		
	for (i = 0; i < Length; i++)
	{
		LPUart_SendData(pdata[i]);
	}
}

/******************************************************************************
 * ���ڳ�ʼ������
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
    
    //ͨ���˿�����
    Gpio_SetFunc_UART0TX_P35();
    Gpio_SetFunc_UART0RX_P36();

    //����ʱ��ʹ��
    Clk_SetPeripheralGate(ClkPeripheralBt,TRUE);//ģʽ0/2���Բ�ʹ��
    Clk_SetPeripheralGate(ClkPeripheralUart0,TRUE);



    stcUartIrqCb.pfnRxIrqCb = Uart0_RxIntCallback;
    stcUartIrqCb.pfnTxIrqCb = NULL;
    stcUartIrqCb.pfnRxErrIrqCb = Uart0_ErrIntCallback;
    stcConfig.pstcIrqCb = &stcUartIrqCb;
    stcConfig.bTouchNvic = TRUE;
  

    stcConfig.enRunMode = UartMode1;//��������Ĵ˴���ת��4��ģʽ����
   

    stcMulti.enMulti_mode = UartNormal;//��������Ĵ˴���ת��������ģʽ��mode2/3���ж�����ģʽ

    stcConfig.pstcMultiMode = &stcMulti;

    stcBaud.bDbaud = 0u;//˫�������ʹ���
    stcBaud.u32Baud = 9600u;//���²�����λ��
    stcBaud.u8Mode = UartMode1; //���㲨������Ҫģʽ����
    pclk = Clk_GetPClkFreq();
    timer=Uart_SetBaudRate(UARTCH0,pclk,&stcBaud);

    stcBtConfig.enMD = BtMode2;
    stcBtConfig.enCT = BtTimer;
    Bt_Init(TIM0, &stcBtConfig);//����basetimer1���ú�������������
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
    
    //ͨ���˿�����
	Gpio_SetFunc_UART1_TXD_P01();
    Gpio_SetFunc_UART1_RXD_P02();

    //����ʱ��ʹ��
    Clk_SetPeripheralGate(ClkPeripheralBt,TRUE);//ģʽ0/2���Բ�ʹ��
    Clk_SetPeripheralGate(ClkPeripheralUart1,TRUE);



    stcUartIrqCb.pfnRxIrqCb = Uart1_RxIntCallback;
    stcUartIrqCb.pfnTxIrqCb = NULL;
    stcUartIrqCb.pfnRxErrIrqCb = Uart1_ErrIntCallback;
    stcConfig.pstcIrqCb = &stcUartIrqCb;
    stcConfig.bTouchNvic = TRUE;
  

    stcConfig.enRunMode = UartMode1;//��������Ĵ˴���ת��4��ģʽ����
   

    stcMulti.enMulti_mode = UartNormal;//��������Ĵ˴���ת��������ģʽ��mode2/3���ж�����ģʽ

    stcConfig.pstcMultiMode = &stcMulti;

    stcBaud.bDbaud = 0u;//˫�������ʹ���
    stcBaud.u32Baud = 9600u;//���²�����λ��
    stcBaud.u8Mode = UartMode1; //���㲨������Ҫģʽ����
    pclk = Clk_GetPClkFreq();
    timer=Uart_SetBaudRate(UARTCH1,pclk,&stcBaud);

    stcBtConfig.enMD = BtMode2;
    stcBtConfig.enCT = BtTimer;
    Bt_Init(TIM1, &stcBtConfig);//����basetimer1���ú�������������
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
    
    Clk_SetPeripheralGate(ClkPeripheralLpUart,TRUE);//ʹ��LPUARTʱ��
    Clk_SetPeripheralGate(ClkPeripheralBt,TRUE);
    
    //ͨ���˿�����
    Gpio_InitIOExt(2,5,GpioDirOut,TRUE,FALSE,FALSE,FALSE);
    Gpio_InitIOExt(2,6,GpioDirOut,TRUE,FALSE,FALSE,FALSE);

    Gpio_SetFunc_UART2RX_P25();
    Gpio_SetFunc_UART2TX_P26();

    //Gpio_InitIO(T1_PORT,T1_PIN,GpioDirIn);  //?
    
    stcLpuart_clk.enSclk_sel = LPUart_Pclk;//LPUart_Rcl;
    stcLpuart_clk.enSclk_Prs = LPUartDiv1;
    stcConfig.pstcLpuart_clk = &stcLpuart_clk;

    stcRunMode.enLpMode = LPUartNoLPMode;//��������ģʽ��͹��Ĺ���ģʽ����
    stcRunMode.enMode   = LPUartMode1;
    stcConfig.pstcRunMode = &stcRunMode;

    stcLPUartIrqCb.pfnRxIrqCb = Lpuart_RxIntCallback;
    stcLPUartIrqCb.pfnTxIrqCb = NULL;
    stcLPUartIrqCb.pfnRxErrIrqCb = Lpuart_ErrIntCallback;
    stcConfig.pstcIrqCb = &stcLPUartIrqCb;
    stcConfig.bTouchNvic = TRUE;

    stcMulti.enMulti_mode = LPUartNormal;//ֻ��ģʽ2/3���ж�����ģʽ
    stcConfig.pstcMultiMode = &stcMulti;
   
    LPUart_EnableIrq(LPUartRxIrq);

    LPUart_Init(&stcConfig);

    if(LPUart_Pclk == stcLpuart_clk.enSclk_sel)
        u32sclk = Clk_GetPClkFreq();
    else if(LPUart_Rcl == stcLpuart_clk.enSclk_sel)
        u32sclk = 38400;//�˴������û�ʹ���ڲ�38.4Kʱ�ӣ�����û�ʹ��32.768Kʱ�ӵģ��˴����³�32768
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
    Bt_Init(TIM2, &stcBtConfig);//����basetimer2���ú�������������
    Bt_ARRSet(TIM2,u16timer);
    Bt_Cnt16Set(TIM2,u16timer);
    Bt_Run(TIM2);

    LPUart_EnableFunc(LPUartRx);    	
}
	
/******************************************************************************
 * ��֡��ʱ���жϻص�����
 ******************************************************************************/
void LptInt(void)
{	
	if (TRUE == Lpt_GetIntFlag())
	{
		Lpt_ClearIntFlag();
		RxDoneFlag_uart1 = 1;  //���ڽ�����ɱ�־
		RxDoneFlag_uart0 = 1;  //���ڽ�����ɱ�־
		Lpt_Stop();
	}
}

/******************************************************************************
 * ���ڷ�֡��ʱ����ʼ������
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
    
    //ʹ��Lpt����ʱ��
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
    
    //Lpt �ж�ʹ��
    Lpt_ClearIntFlag();
    Lpt_EnableIrq();
    EnableNvic(LPTIM_IRQn, 0, TRUE);
    
    
    //��������ֵ��������ֵ����������
//    Lpt_ARRSet(0);
//    Lpt_Run();
}

/******************************************************************************
 * PCA��ʱ����ʼ��
 ******************************************************************************/
void Pca_Timer_Init(void)
{
	  stc_pca_config_t stcConfig;
    stc_pca_capmodconfig_t stcModConfig;
    en_result_t      enResult = Error;
    uint16_t         u16InitCntData = 0;
	uint16_t         u16CcapData = 0x009B;    
	
	Clk_SetPeripheralGate(ClkPeripheralPca, TRUE);  //����PCA����ʱ��
    stcConfig.enCIDL = IdleGoon; 
    stcConfig.enWDTE = PCAWDTDisable;
    stcConfig.enCPS  = PCAPCLKDiv32; 
    
    stcConfig.pfnPcaCb = PcaInt;
    
    stcModConfig.enECOM = ECOMEnable;//����ȽϹ���
    stcModConfig.enCAPP = CAPPDisable;
    stcModConfig.enCAPN = CAPNDisable;
    stcModConfig.enMAT  = MATEnable;//����ƥ��
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
    
    //PCA �ж�ʹ��
    Pca_ClearIntFlag(Module2);
    Pca_EnableIrq(Module2);    
    EnableNvic(PCA_IRQn, 3, TRUE);

    Pca_CapData16Set(Module2, u16CcapData);//�Ƚϲ���Ĵ�������
    //Pca_Cnt16Set(u16InitCntData);
	Pca_Run();

}

/******************************************************************************
 * ADTimer5�жϷ�����-������
 ******************************************************************************/
void Adt5CompACalllback(void)
{
	static uint8_t powerLowFlag=0;
	
    Adt_GetCntState(AdTIM5, &stcAdt5CntState);

	Adt_SetCompareValue(AdTIM5, AdtCompareA, 0x00ff);    //??????????A??


	//50ms���һ�ε���
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
 * ADTimer4�жϷ�����-Lpuart��֡
 ******************************************************************************/
void Adt4OVFCalllback(void)
{
	//static bool ledstat=0;
	static uint8_t powerLowFlag=0;

	Adt_ClearIrqFlag(AdTIM4,AdtOVFIrq);  //���жϱ�־
	RxDoneFlag_lpuart = 1;  //���ڽ�����ɱ�־
	Adt_StopCount(AdTIM4);  //ֹͣ����
	
	//������֤ʱʹ�ã�ȷ�Ϸ�֡ʱ��
	//ledstat = !ledstat;  
	//Gpio_SetIO(3,2,ledstat);
		

}

/******************************************************************************
 * ADTimer5��ʼ��-������
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
 * ADTimer4��ʼ��-Lpuart��֡
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

	u16Period = 0x4E00;										//5ms֡��ʱʱ�� ����Ƶ
	//u16Period = 0x0138;                                    //5ms֡��ʱʱ�� 64��Ƶ(���쳣����ʱ������ǰ�����ж�)
    Adt_SetPeriod(enAdt, u16Period);                         //
    
    Adt_ConfigIrq(enAdt, AdtOVFIrq, TRUE, Adt4OVFCalllback);
}

/******************************************************************************
 * LoRaWANģ��GPIO��ʼ��
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
 * ������ʼ��
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
 * LED��ʼ��
 ******************************************************************************/
void Pca_led_init(void)
{
    stc_pca_config_t stcConfig;
    stc_pca_capmodconfig_t stcModConfig;
    en_result_t      enResult = Ok;
    uint32_t         u32Cnt; 

	//Gpio_InitIOExt(3, 2, GpioDirOut, FALSE, FALSE, TRUE, FALSE);  //LED1
	
	Clk_SetPeripheralGate(ClkPeripheralPca, TRUE);  //����PCA����ʱ��
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
 * �������ʼ��
 ******************************************************************************/
void Gpio_vdetect_init(void)
{
    Gpio_InitIOExt(0, 3, GpioDirIn, FALSE, FALSE, FALSE, 0);  //VDETECT
	
//    Gpio_ClearIrq(0, 3);
//    Gpio_EnableIrq(0, 3, GpioIrqFalling);
//    EnableNvic(PORT0_IRQn, DDL_IRQ_LEVEL_DEFAULT, TRUE);
}

/******************************************************************************
 * GPIO�жϷ�����
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
 * PCA�жϷ�����
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
			RxDoneFlag_lpuart = 1;  //���ڽ�����ɱ�־
			Pca_Stop();
			
			//����ָ��ʶ��
			if(lpuart_RxByteCnt==4)
			{
				if(lpuart_RxBuf[0]=='C'||lpuart_RxBuf[1]=='4')  //������������
				{
						//��ȡ��������
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
 * LoRaWANģ�鸴λ
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
 * ���Ź��жϷ�����
 ******************************************************************************/
void WdtCallback(void)
{
    // comment following to demonstrate the hardware watchdog reset

}


/******************************************************************************
 * ���Ź���ʼ��
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

