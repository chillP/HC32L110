/*!
 * @file       app.c
 * @brief      应用层功能的实现
 * @details    应用层软件功能：
				1、数据获取
				2、LoRaWAN网络维护
				3、心跳上报
				4、告警上报
				5、掉电告警
				6、心跳间隔修改
		   
 * @copyright  Revised BSD License, see section LICENSE. \ref LICENSE
 * @author     Lierda-WSN-LoRaWAN-Team
 * @date       2020-03-20
 * @version    V1.0
 */
 
 
 /* Includes ------------------------------------------------------------------*/
#include <math.h>
#include "app.h"
#include "driver_mid.h"
#include "commonfun.h"
#include "LSD_LoRaWAN_ICA_Driver.h"
#include "HDEE5.h"
#include "pca.h"

#define HandshakeReqLen 14
#define HandshakeAcpLen 16
#define Uplink1Len 15
#define Downlink1Len 14
#define Uplink2Len 15
#define Downlink2Len 14


bool onlineFlag = 0;
bool errorReportFlag=0;
uint16_t getDataCnt=0;
uint8_t saveDataCnt=0;
uint16_t dataMax=0;
uint16_t dataAvg=0;
uint32_t dataSum=0;
dataRecordType ch4DataBuf;
uint8_t heartbeatPeriod=8;  //缺省心跳周期8h

uint8_t getDataBuf[13];
uint8_t getCh4Cmd[23]={"*0401000000000000FA\r\n"};
uint8_t heartBeatBuf[50];
uint8_t deveuiBuf[30] = {0};
int8_t rssiThreshould = 0;
int8_t snrThreshould = 0;
int8_t rssiGw = 0;
int8_t rssiDtu = 0;
int8_t snrGw = 0;
int8_t snrDtu = 0;
uint8_t testModePower = 0;

bool alarmReportFlag_Lel=0;
bool alarmReportFlag_Sensor=0;
bool statReportFlag_Mute=0;
bool statReportFlag_Stest=0;

extern uint8_t vdetectEnable;
extern bool powerDownFlag;
extern bool timeout_start_flag;
extern bool heartbeatReport_Tp;
extern uint8_t EeBuf[HDEE_EeSize];
extern ledStatType ledStat;
extern bool keyDetectedFlag;
extern bool powerOnFlag;

uint8_t handShakePkt[HandshakeReqLen]={0xE4,0xFB,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0X00,0X00};  //2字节帧头+1字节控制码+8字节DEVEUI

void lorawanResetAndConfig(void)
{
	printf("\r\n===Module Config===\r\n");
	printf("-reset lorawan module \r\n ");
	node_hard_reset();
	
	system_delay_ms(200);
	printf("-set basic para: \r\n");
	if(save_cmd_configure())
	{
		printf("OK!!!\r\n");
	}
	else
	{
		printf("ERROR!!!\r\n");		
	}
	//transfer_configure_command("AT+DEBUG=1");
}

void getSensorData_Task(void)
{
	uint8_t checkSum = 0x00;
	uint16_t ch4Data = 0;
	char hexData[2] = "00";
	int i=0;
	
	//不论成功与否，进入即累加
	getDataCnt++;
	
	//CO浓度查询
	printf("\r\n===Data Fetch===\r\n");
	printf("-enquire sensor data(CH4) \r\n");
	UART_RECEIVE_FLAG = 0;
	uart0_RxByteCnt = 0;
	memset(uart0_RxBuf, 0, sizeof(uart0_RxBuf));
	
	Uart0_send_string(getCh4Cmd);
	
	timeout_start_flag = true;
	while(RxDoneFlag_uart0 == 0)
	{
		if(true == time_out_break_ms(2000))
		{
			break;
		}
	}
	RxDoneFlag_uart0 = 0;
	printf("-get sensor data(CH4): %s",uart0_RxBuf);

	for(i=0;i<13;i++)
	{
		getDataBuf[i] = uart0_RxBuf[i];
	}
	
	//校验和计算
	
	//获取浓度
	hexData[0]=getDataBuf[3];
	hexData[1]=getDataBuf[4];
	ch4Data = hexToDec(hexData);
	

	hexData[0]=getDataBuf[5];
	hexData[1]=getDataBuf[6];
	ch4Data += hexToDec(hexData)*256;
	printf("CH4: %.1f",ch4Data/10.0);  
	
	printf("%%");
	printf(" LEL\r\n");


//	//浓度判断
//	if(ch4Data>100)  //大于10%LEL
//	{
//		errorReportFlag = 1;
//	}
	
	//获取报警状态
	switch(getDataBuf[2])  //若取高2位，屏蔽CO部分 则|0X0C
	{
		case '0': //正常
			printf("CH4-STAT:OK\r\n");
			break;
		
		case '4': //CH4浓度报警
			printf("CH4-STAT:EXCEED!\r\n");
			alarmReportFlag_Lel = 1;
			break;
		
		case '8': //CH4传感器故障
			printf("CH4-STAT:SENSOR FAULT!\r\n");
			alarmReportFlag_Sensor = 1;
			break;		
	}
	
	//获取运行状态
	switch(getDataBuf[1])
	{
		case '4': 
			printf("mute now!\r\n");
			statReportFlag_Mute =1;
			break;
		case '8': 
			printf("self-testing!\r\n");
			statReportFlag_Stest =1;
			break;
		
	}
	
	if(alarmReportFlag_Lel|alarmReportFlag_Sensor|statReportFlag_Mute|statReportFlag_Stest)
	{
		errorReportFlag = 1;
	}
	
	//数据获取 - 5s周期
	if(ch4Data > dataMax) dataMax = ch4Data;  //记录峰值
	dataSum += ch4Data;  //累加和
	
	//数据存储 - 15s周期
	if(getDataCnt>=3)  
	{
		getDataCnt = 0;
		
		
		dataAvg = dataSum/3.0;  

		ch4DataBuf.dataAvg[saveDataCnt] = dataAvg;  //保存均值
		ch4DataBuf.dataMax[saveDataCnt] = dataMax;  //保存峰值
		
		printf("\r\n===Data Storage===\r\n");
		printf("-data stored!\r\n");
		printf("Avg: %.1f\r\n",dataAvg/10.0);  
		printf("Max: %.1f\r\n",dataMax/10.0);  
		
		dataSum = 0;
		dataMax = 0;
		saveDataCnt++;
	}
	
	//数据上报 - 120s周期
	if(saveDataCnt>=heartbeatPeriod)  
	{
		saveDataCnt = 0;
		heartbeatReport_Tp = 1;
	}
		
	
//	for(i=0;i<uart0_RxByteCnt/2;i++)
//	{
	//	printf("%d ",hexToDec((char*)uart0_RxBuf[3])*16);  //高4bit
	//	printf("%d ",hexToDec((char*)uart0_RxBuf[4]));  //低4bit
		
//		checkSum += hexToDec((char*)uart0_RxBuf[i])+16;  //高4bit
//		checkSum += hexToDec((char*)uart0_RxBuf[i+1]);  //低4bit
//	}
	//checkSum = 0XFF - checkSum;
}

void networkConnect_Task(void)
{
	printf("\r\n===Network Connect===\r\n");
	printf("-join the network: \r\n");
	ledStat = SLOWFALSH;  
	if(node_block_join(150))
	{
		printf("OK\r\n");
		onlineFlag = 1;
		vdetectEnable = 1;  //开启掉电告警
		ledStat = BREATH;  //开启呼吸灯
	}
	else
	{
		printf("ERROR\r\n");
		printf("-rejoin after 60s \r\n");
		timeout_start_flag = true;
		while(1)
		{
			if(true == time_out_break_ms(60000))
			{
				break;
			}
		}
	}
}

void heartBeatReport_Task(void)
{
	uint8_t i=0;
	down_list_t *head = NULL;
	execution_status_t send_result;
	uint8_t ch4DataAvg[2];
	uint8_t ch4DataMax[2];	
	
	//控制码字段
	heartBeatBuf[0] = 1;
	
	//心跳周期字段
	heartBeatBuf[1] = heartbeatPeriod;
	
	//浓度数据字段
	for(i=0;i<heartbeatPeriod;i++)
	{
		heartBeatBuf[4*i+2] = ch4DataBuf.dataMax[i]/256;  //峰值高字节
		heartBeatBuf[4*i+3] = ch4DataBuf.dataMax[i] & 0x00ff;  //峰值低字节
		heartBeatBuf[4*i+4] = ch4DataBuf.dataAvg[i]/256;  //均值高字节
		heartBeatBuf[4*i+5] = ch4DataBuf.dataAvg[i] & 0x00ff;  //均值低字节	
		
	}
	
	//数据打印
	printf("-heartbeatBuf: ");		
	for(i=0;i<4*heartbeatPeriod+2;i++)
	{
		printf("%02x ",heartBeatBuf[i]);		
	}
	printf("\r\n");	
	
	UART_RECEIVE_FLAG = 0;
	UART_RECEIVE_LENGTH = 0;
	memset(UART_RECEIVE_BUFFER, 0, sizeof(UART_RECEIVE_BUFFER));
	
	node_join_successfully=1;
	node_gpio_set(wake, wakeup);  //唤醒
	
	printf("\r\n===Data Report===\r\n");
	printf("-report heartbeat: \r\n");
	send_result = node_block_send(CONFIRM_TYPE | 0x03, heartBeatBuf, 4*heartbeatPeriod+2, &head);
	
	//node_gpio_set(wake, sleep);  //休眠
	if(logLevel == 2)
	{
		timeout_start_flag = true;
		while(!UART_RECEIVE_FLAG)
		{
			if(true == time_out_break_ms(1000))
			{
				break;
			}
		}
		if(UART_RECEIVE_FLAG)
		{
			DEBUG_PRINTF("%s\r\n",UART_RECEIVE_BUFFER);
			UART_RECEIVE_FLAG = 0;
			UART_RECEIVE_LENGTH = 0;
			memset(UART_RECEIVE_BUFFER, 0, sizeof(UART_RECEIVE_BUFFER));
			
		}
	}
	
	system_delay_ms(200);  //等待模块日志输出完成
	
	if(send_result == 1)
	{
		printf(" OK\r\n"); 
	}
	else
	{
		printf(" ERROR: %d \r\n",send_result); 
	}	
}

void errorReport_Task(void)
{
	down_list_t *head = NULL;
	execution_status_t send_result;
	uint8_t i=0;
	uint16_t seed = 0;
	uint16_t random_t = 0;
	char hexData[2];
	uint8_t lelReportBuf[4];
	uint8_t statReportBuf[3];
	uint8_t errorByte;
	uint16_t ch4Data = 0;

	//***报警帧***//
	
	//控制码
	lelReportBuf[0] = 0x31;
	
	//报警标志
	hexData[0]=getDataBuf[1];
	hexData[1]=getDataBuf[2];
	errorByte = hexToDec(hexData);	
	lelReportBuf[1] = errorByte;	

	//气体浓度
	hexData[0]=getDataBuf[3];
	hexData[1]=getDataBuf[4];
	ch4Data = hexToDec(hexData);
	lelReportBuf[3] = ch4Data;

	hexData[0]=getDataBuf[5];
	hexData[1]=getDataBuf[6];
	ch4Data = hexToDec(hexData)*256;
	lelReportBuf[2] = ch4Data;

	//***状态帧***//
	
	//控制码
	statReportBuf[0] = 0x32;

	//状态标志
	statReportBuf[1] = 0;
	statReportBuf[2] = 0;
	if(statReportFlag_Mute) statReportBuf[1] = 0xff;
	if(statReportFlag_Stest) statReportBuf[2] = 0xff;	
	
	for(i=0;i<3;i++)
	{
		UART_RECEIVE_FLAG = 0;
		UART_RECEIVE_LENGTH = 0;
		memset(UART_RECEIVE_BUFFER, 0, sizeof(UART_RECEIVE_BUFFER));
		
		node_join_successfully=1;
		
		node_gpio_set(wake, wakeup);  //唤醒
		
		printf("\r\n===Error Report===\r\n");
		
		if(alarmReportFlag_Lel|alarmReportFlag_Sensor)
		{
			if(alarmReportFlag_Lel) printf("-gas lel warning!\r\n");
			if(alarmReportFlag_Sensor) printf("-sensor fault!\r\n");
			
			//数据打印
			printf("-lelReportBuf: ");		
			for(i=0;i<4;i++)
			{
				printf("%02x ",lelReportBuf[i]);		
			}
			printf("\r\n");	
			
			send_result = node_block_send(CONFIRM_TYPE | 0x07, lelReportBuf, 4, &head);
		}
		if(statReportFlag_Mute|statReportFlag_Stest)
		{
			if(statReportFlag_Mute) printf("-muting now!\r\n");
			if(statReportFlag_Stest) printf("-self testing!\r\n");
			
			//数据打印
			printf("-statReportBuf: ");		
			for(i=0;i<3;i++)
			{
				printf("%02x ",statReportBuf[i]);		
			}
			printf("\r\n");	
			
			send_result = node_block_send(CONFIRM_TYPE | 0x07, statReportBuf, 3, &head);
		}			
		
		if(logLevel == 2)
		{
			timeout_start_flag = true;
			while(!UART_RECEIVE_FLAG)
			{
				if(true == time_out_break_ms(1000))
				{
					break;
				}
			}
			if(UART_RECEIVE_FLAG)
			{
				DEBUG_PRINTF("%s\r\n",UART_RECEIVE_BUFFER);
				UART_RECEIVE_FLAG = 0;
				UART_RECEIVE_LENGTH = 0;
				memset(UART_RECEIVE_BUFFER, 0, sizeof(UART_RECEIVE_BUFFER));
				
			}
		}
		
		system_delay_ms(200);  //等待模块日志输出完成
		
		if(send_result == 1)
		{
			printf(" OK\r\n"); 
			break;
		}
		else
		{
			
			printf(" ERROR: %d \r\n",send_result); 
			//随机延时
			random_t = rand() % 400 +300;  //生成随机数
			printf("\r\n-delay %d ms\r\n",random_t*10);
			
			timeout_start_flag = true;
			while(1)
			{
				if(true == time_out_break_ms(random_t*10))
				{
					break;
				}
			}		
		}			
	}	
	errorReportFlag = 0;
	alarmReportFlag_Lel = 0;
	alarmReportFlag_Sensor = 0;
	statReportFlag_Mute = 0;
	statReportFlag_Stest = 0;
}

void powerDown_Task(void)
{
	uint8_t i=0,j=0;
	down_list_t *head = NULL;
	execution_status_t send_result;
	uint16_t seed = 0;
	uint16_t random_t = 0;
	char hexData[2];	
	uint8_t pdReportBuf[2];
	
	ledStat = QUICKFLASH;
	printf("-power-down detected!!!\r\n"); 
	
	//FLASH存储掉电标记
	EeBuf[0]=0x11;
	HDEE_Write( 0x00 , EeBuf , HDEE_EeSize );
	
	UART_RECEIVE_FLAG = 0;
	UART_RECEIVE_LENGTH = 0;
	memset(UART_RECEIVE_BUFFER, 0, sizeof(UART_RECEIVE_BUFFER));
	
	node_join_successfully=1;
	
	//帧组合
	pdReportBuf[0] = 0x33;
	pdReportBuf[1] = 0xff;
	
	//随机数种子生成              
	hexData[0]=deveuiBuf[22];
	hexData[1]=deveuiBuf[23];
	
	seed = hexToDec(hexData);
	srand( seed );                  
	
	for(i=0;i<3;i++)
	{
		
		//随机延时(100-200-300)
		random_t = rand() % 100 +80*i;  //生成随机数
		printf("\r\n-delay %d ms\r\n",random_t*100);
		
		timeout_start_flag = true;
		while(1)
		{
			if(true == time_out_break_ms(random_t*100))
			{
				break;
			}
		}	
		
		node_gpio_set(wake, wakeup);  //唤醒
		
		printf("\r\n===PowerDown Report===\r\n");
		
		//数据打印
		printf("-pdReportBuf: ");		
		for(j=0;j<2;j++)
		{
			printf("%02x ",pdReportBuf[j]);		
		}
		printf("\r\n");			
		
		send_result = node_block_send(UNCONFIRM_TYPE | 0x01, pdReportBuf, 2, &head);
		
		node_gpio_set(wake, sleep);  //休眠
		if(logLevel == 2)
		{
			timeout_start_flag = true;
			while(!UART_RECEIVE_FLAG)
			{
				if(true == time_out_break_ms(1000))
				{
					break;
				}
			}
			if(UART_RECEIVE_FLAG)
			{
				DEBUG_PRINTF("%s\r\n",UART_RECEIVE_BUFFER);
				UART_RECEIVE_FLAG = 0;
				UART_RECEIVE_LENGTH = 0;
				memset(UART_RECEIVE_BUFFER, 0, sizeof(UART_RECEIVE_BUFFER));
				
			}
		}
		
		system_delay_ms(200);  //等待模块日志输出完成
		
		if(send_result == 1)
		{
			printf(" OK\r\n"); 
		}
		else
		{
			printf(" ERROR: %d \r\n",send_result); 
		}		
		
	}
	
	//上报完成后静默,等待上电后复位系统
	ledStat = OFF;
	while(Gpio_GetIO(0, 3)==0)
	{
	}	
	//__NVIC_SystemReset();
	NVIC_SystemReset();
}

void powerOn_Task(void)
{
	down_list_t *head = NULL;
	execution_status_t send_result;
	uint8_t i=0;
	uint16_t seed = 0;
	uint16_t random_t = 0;
	char hexData[2];	
	uint8_t poReportBuf[2];	

	//帧组合
	poReportBuf[0] = 0x33;
	poReportBuf[1] = 0x00;
	
	//随机数种子生成              
	hexData[0]=deveuiBuf[22];
	hexData[1]=deveuiBuf[23];
	
	seed = hexToDec(hexData);
	srand( seed );          
	
	for(i=0;i<3;i++)
	{
		//随机延时
		random_t = rand() % 400 +300;  //生成随机数
		printf("\r\n-delay %d ms\r\n",random_t*10);
		
		timeout_start_flag = true;
		while(1)
		{
			if(true == time_out_break_ms(random_t*10))
			{
				break;
			}
		}	
		
		//上报掉电恢复状态
		UART_RECEIVE_FLAG = 0;
		UART_RECEIVE_LENGTH = 0;
		memset(UART_RECEIVE_BUFFER, 0, sizeof(UART_RECEIVE_BUFFER));
		
		node_join_successfully=1;
		
		node_gpio_set(wake, wakeup);  //唤醒
		
		printf("\r\n===PowerRecovery Report===\r\n");
		
		//数据打印
		printf("-poReportBuf: ");		
		for(i=0;i<2;i++)
		{
			printf("%02x ",poReportBuf[i]);		
		}
		printf("\r\n");	
		
		send_result = node_block_send(CONFIRM_TYPE | 0x07, poReportBuf, 2, &head);
		
		if(logLevel == 2)
		{
			timeout_start_flag = true;
			while(!UART_RECEIVE_FLAG)
			{
				if(true == time_out_break_ms(1000))
				{
					break;
				}
			}
			if(UART_RECEIVE_FLAG)
			{
				DEBUG_PRINTF("%s\r\n",UART_RECEIVE_BUFFER);
				UART_RECEIVE_FLAG = 0;
				UART_RECEIVE_LENGTH = 0;
				memset(UART_RECEIVE_BUFFER, 0, sizeof(UART_RECEIVE_BUFFER));
				
			}
		}
		
		system_delay_ms(200);  //等待模块日志输出完成
		
		if(send_result == 1)
		{
			printf(" OK\r\n"); 
			break;
		}
		else
		{
			printf(" ERROR: %d \r\n",send_result); 
		}			
	}
}

void factoryTest_Task(void)
{
	//vdetectEnable = 0;  //关闭掉电检测
}

void pdRecovery_Judge(void)
{	
	//读取掉电标记
	HDEE_Read( 0x00 , EeBuf , HDEE_EeSize );
	
	//掉电恢复
	if(EeBuf[0]==0x11) 
	{
		printf("===recover from powerdown===\r\n");
		powerOnFlag = 1;
	}
	else printf("===nomal power on===\r\n");
	
}

void ledStatHandle(void)
{
	static bool handledFlag=0;
	static bool toogleFlag=0;
	static uint8_t timeCnt=0;
	static uint8_t breathCnt=0;
	static uint8_t flashCnt=0;
	static ledStatType lastLedState;
	uint8_t y=0;
	uint8_t breath_array[160]={0,	1,	3,	4,	5,	6,	8,	9,	10,	11,	12,	14,	15,	16,	17,	18,	19,	20,	
								21,	22,	23,	24,	25,	26,	27,	28,	29,	30,	31,	32,	33,	34,	34,	35,	36,	37,
								37,	38,	39,	40,	40,	41,	41,	42,	43,	43,	44,	44,	45,	45,	46,	46,	46,	47,
								47,	48,	48,	48,	48,	49,	49,	49,	49,	49,	50,	50,	50,	50,	50,	50,	50,	50,
								50,	50,	50,	50,	50,	49,	49,	49,	49,	49,	48,	48,	48,	47,	47,	47,	46,	46,
								45,	45,	44,	44,	43,	43,	42,	42,	41,	41,	40,	39,	39,	38,	37,	36,	36,	35,
								34,	33,	32,	31,	31,	30,	29,	28,	27,	26,	25,	24,	23,	22,	21,	20,	19,	18,
								16,	15,	14,	13,	12,	11,	9,	8,	7,	6,	5,	3,	2,	1};
	
	switch(ledStat)
	{
		case ON: //点亮
			Pca_CapDataHSet(Module4, 200);	
			lastLedState = ledStat;
			break;
		case OFF:  //熄灭
			Pca_CapDataHSet(Module4, 0);	
			lastLedState = ledStat;
			break;
		case BREATH:  //10s呼吸 
			breathCnt++;
			if(breathCnt>=200) breathCnt = 0;
			//y = 100*sin(0.015*breathCnt) - 50;
			
			if(breathCnt>=34 && breathCnt<=173)
			{
				y=breath_array[breathCnt-34];
			}
			else y=0;
			
			Pca_CapDataHSet(Module4, y);
			lastLedState = ledStat;		
			break;
		case BLINK:  //20ms闪烁一次
			timeCnt++;
			if(timeCnt<2)
			{
				Pca_CapDataHSet(Module4, 0);
			}				
			if(timeCnt>=2 && timeCnt<3)
			{
				Pca_CapDataHSet(Module4, 200);
			}				
			if(timeCnt>=3)
			{
				Pca_CapDataHSet(Module4, 0);
				timeCnt = 0;
				ledStat = lastLedState;
			}	
			break;
		case QUICKFLASH:  //500ms快闪
			flashCnt++;
			if(flashCnt>=10)
			{
				flashCnt=0;
				toogleFlag=!toogleFlag;
				if(toogleFlag) Pca_CapDataHSet(Module4, 0);
				else Pca_CapDataHSet(Module4, 200);
			}
			lastLedState = ledStat;
			break;
		case SLOWFALSH:  //1s慢闪 	
			flashCnt++;
			if(flashCnt>=20)
			{
				flashCnt=0;
				toogleFlag=!toogleFlag;
				if(toogleFlag) Pca_CapDataHSet(Module4, 0);
				else Pca_CapDataHSet(Module4, 200);
			}
			lastLedState = ledStat;
			break;
	}
}
void getDeveui(void)
{
	uint8_t return_data[50] = {0};
	uint16_t seed = 0;
	uint16_t random_t = 0;
	uint8_t i = 0;
	char hexData[2];
	
	transfer_inquire_command("AT+DEVEUI?", return_data);
	for(i=0;i<50;i++)
	{
		deveuiBuf[i]=return_data[i];
	}
	
	
	hexData[0]=return_data[22];
	hexData[1]=return_data[23];
	
	seed = hexToDec(hexData);
	srand( seed );                  
	random_t = rand() % 120;  //生成0-120的随机数
	printf("-delay %d seconds\r\n",random_t);
	
	timeout_start_flag = true;
	while(keyDetectedFlag != 1)
	{
		if(true == time_out_break_ms(random_t*1000))
		{
			break;
		}
	}
	keyDetectedFlag = 0;
}

void p2pSetup(void)
{
	LoRaNode_P2P_Info LoRa_P2P_Node;
	uint8_t confResult[10]={0};
	uint16_t i = 0;
	uint8_t errorFlag=0;
	
	node_hard_reset();
	
	//参数初始化
	LoRa_P2P_Node.P2P_FRQ = 4883;
	LoRa_P2P_Node.P2P_PWR = 10;
	LoRa_P2P_Node.P2P_SF = 7;
	
	//配置P2P
	printf("- set p2p para: "); 
	node_gpio_set(mode, command);
	transfer_configure_command("AT+factory");
	confResult[0] = transfer_configure_command("AT+MINIRF=1");
	confResult[1] = LoRaNode_SetP2P(LoRa_P2P_Node.P2P_FRQ*100000,1,LoRa_P2P_Node.P2P_PWR,LoRa_P2P_Node.P2P_SF,0,1,0,0,0);	
	confResult[2] = transfer_configure_command("AT+STATUS=3,3");
	confResult[3] = transfer_configure_command("AT+SAVE");
	
	for(i=0;i<4;i++)	
	{
		if(logLevel == 2)  printf("%d\r\n",confResult[i]);
		if(confResult[i]==0) errorFlag=1;
	}		
	
	if(!errorFlag) printf("OK\r\n");
	else printf("ERROR\r\n");	
}

uint8_t frameCheck(uint8_t* return_data)
{
	uint8_t i=0;
	bool devCheckOk=false;
	
	if(return_data[0]==0xC8 && return_data[1]==0xDF)  //帧头
	{
		for(i=0;i<8;i++)  //DEVEUI校验
		{
			if(return_data[i+3] != handShakePkt[i+3])
			{
				printf("Deveui check error\r\n");
				return 0;
			}
		}
		
		switch (return_data[2])	  //返回控制码
		{
			case 0x01:
				printf("Frame check ok\r\n");
				return 0x01;
				break;
			case 0xF2:
				printf("Frame check ok\r\n");
				return 0xF2;
				break;
			case 0xF4:
				printf("Frame check ok\r\n");
				return 0xF4;
				break;
			default:
				printf("Ctrl bit check error\r\n");
				return 0;
		}	
	}
	else
	{
		printf("Frame head check error\r\n");
		return 0;
	}	
}

uint8_t handShakeAttempt(void)
{
	uint8_t return_data[50] = {0};
	uint8_t tryCnt = 0;
	uint16_t dataLen = 0;
	bool frameCheckOk = true;
	
	uint16_t i = 0;
	char hexData[2];
	uint16_t seed = 0;
	uint16_t random_t = 0;
	
	printf("\r\n===Hand Shake Attempt===\r\n");
	
	//组帧
	for(i=0;i<8;i++)  //DEVEUI写入
	{
		hexData[0]=deveuiBuf[i*3+1];
		hexData[1]=deveuiBuf[i*3+2];
		
		handShakePkt[i+3] = (uint8_t)hexToDec(hexData);
	}

	if(logLevel == 2)  //帧打印
	{
		printf("handshakepkt: ");
		for(i=0;i<14;i++)
		{
			printf("%02x",handShakePkt[i]);
		}
		printf("\r\n");
	}

	for(tryCnt=0;tryCnt<3;tryCnt++)  //最多3次握手尝试
	{
		printf("\r\n-Handshake Try %d/3 \r\n",tryCnt+1);
		//随机延时
		hexData[0]=deveuiBuf[22];
		hexData[1]=deveuiBuf[23];
		
		seed = hexToDec(hexData);
		srand( seed );                  
		random_t = rand() % 20;  //生成0-20的随机数
		printf("-delay %d ms\r\n",random_t);
		system_delay_ms(random_t);  //20ms内随机延时
		
		//数据发送
		node_gpio_set(mode, wakeup);
		node_gpio_set(mode, transparent);
		
		UART_RECEIVE_FLAG = 0;
		UART_RECEIVE_LENGTH = 0;
		memset(UART_RECEIVE_BUFFER, 0, sizeof(UART_RECEIVE_BUFFER));
		
		UART_WRITE_DATA((uint8_t*)handShakePkt, 14);
		
		//数据接收
		timeout_start_flag = true;
		while(UART_RECEIVE_FLAG == 0)
		{
			if(true == time_out_break_ms(500))
			{
				break;
			}	
		}
		
		if(UART_RECEIVE_FLAG)
		{
			//帧缓存
			dataLen = UART_RECEIVE_LENGTH;
			for(i=0;i<dataLen;i++)
			{
				return_data[i] = UART_RECEIVE_BUFFER[i];
			}
			
			//调试日志
			if(logLevel == 2)
			{
				printf("get data：");
				for(i=0;i<dataLen;i++)
				{
					printf("%02x ",return_data[i]);
				}
				printf("\r\n");	
			}
			
			//帧校验
			if(frameCheck(return_data)==0x01)  //HandshakeAcp
			{
				//帧内容处理
				rssiThreshould = return_data[13];  //信号质量阈值设定
				snrThreshould = return_data[14];
				
				printf("阈值：rssi> %d ,snr> %d\r\n",rssiThreshould,snrThreshould);	
				printf("#HAND SHAKE SUCCESS\r\n");
				
				testModePower = return_data[12];  //发射功率设定
				
				if(return_data[11] == 1)  //模式识别
				{
					return 1;  //测试模式
				}
				else if(return_data[11] == 2)
				{
					return 2;  //透传模式
				}
				else
				{
					printf("-unknown mode code\r\n");
					return 0;  //数据错误
				}
				
			}	
			else
			{
				printf("-pkt is not HandshakeAcp\r\n");
			}
		}
		else
		{
			printf("- no response\r\n");
		}		
	}
	printf("#HAND SHAKE FAIL\r\n");
	return 0;  //3次握手尝试失败
}

bool sensorTest(void)
{
	uint8_t checkSum = 0x00;
	uint16_t ch4Data = 0;
	char hexData[2] = "00";
	uint8_t i,j=0;
	uint8_t getTemCmd[23]={"*0402000000000000F9\r\n"};
	uint8_t getCh4Threshould[23]={"*0408000000000000F3\r\n"};
	
	
	//报警器串口通信
	printf("\r\n===Sensor Test===\r\n");
	for(j=0;j<3;j++)  //3条不同指令测试
	{
		switch(j)  //发送指令
		{
			case 0: 
				printf("-enquire sensor data(CH4) \r\n");
				RxDoneFlag_uart0 = 0;
				uart0_RxByteCnt = 0;
				memset(uart0_RxBuf, 0, sizeof(uart0_RxBuf));
				Uart0_send_string(getCh4Cmd);
				break;
			case 1: 
				printf("-enquire temperature \r\n");
				RxDoneFlag_uart0 = 0;
				uart0_RxByteCnt = 0;
				memset(uart0_RxBuf, 0, sizeof(uart0_RxBuf));
				Uart0_send_string(getTemCmd);
				break;
			case 2: 
				printf("-enquire CH4 threshould \r\n");
				RxDoneFlag_uart0 = 0;
				uart0_RxByteCnt = 0;
				memset(uart0_RxBuf, 0, sizeof(uart0_RxBuf));
				Uart0_send_string(getCh4Threshould);
				break;			
		}
		timeout_start_flag = true;
		while(RxDoneFlag_uart0 == 0)  //等待接收
		{
			if(true == time_out_break_ms(100))
			{
				break;
			}
		}
		if(RxDoneFlag_uart0 == 1)  //数据校验
		{
			RxDoneFlag_uart0 = 0;
			if(uart0_RxByteCnt == 13)
			{
				for(i=0;i<13;i++)  //缓存数据
				{
					getDataBuf[i] = uart0_RxBuf[i];
				}
				if(getDataBuf[0]=='*' && getDataBuf[11]=='\r' &&  getDataBuf[12]=='\n')  //帧头帧尾校验
				{
					printf("-frame check ok\r\n");
				}
				else
				{
					printf("-frame check error\r\n");
					return 0;
				}
			}
			else
			{
				printf("-len check error\r\n");
				return 0;
			}
		}
		else
		{
			printf("-no response\r\n");
			return 0;
		}
		
	}
	printf("#SENSOR TEST PASS\r\n");
	return 1;
}

bool radioTest(uint8_t sensorTestResult)
{
	uint8_t upLink1Pkt[Uplink1Len]={0xE4,0xFB,0xF1,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0X00,0X00,0X00};  //2字节帧头+1字节控制码+8字节DEVEUI
	uint8_t upLink2Pkt[Uplink2Len]={0xE4,0xFB,0xF3,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0X00,0X00,0X00};  //2字节帧头+1字节控制码+8字节DEVEUI
	uint8_t i = 0;
	char hexData[3]={"00"};
	uint16_t seed = 0;
	uint16_t random_t = 0;
	uint16_t dataLen = 0;
	uint8_t return_data[50] = {0};
	uint8_t tryCnt = 0;
	bool radioTestPass = false;
	uint8_t devbuf=0;
	
	printf("\r\n===Radio Test===\r\n");
	//组帧
	for(i=0;i<8;i++)  //DEVEUI写入
	{
		hexData[0]=deveuiBuf[i*3+1];
		hexData[1]=deveuiBuf[i*3+2];
		
		devbuf = (uint8_t)hexToDec(hexData);
		upLink1Pkt[i+3] = devbuf;
		upLink2Pkt[i+3] = devbuf;
	}

	upLink1Pkt[13] = sensorTestResult;  //报警器测试结果写入
	upLink2Pkt[13] = sensorTestResult;
	
	for(tryCnt=0;tryCnt<3;tryCnt++)  //最多3组射频交互
	{
		printf("\r\n-Radio Test %d/3 \r\n",tryCnt+1);
		//数据发送-UpLink1
		printf("uplink1pkt: ");  //帧打印
		for(i=0;i<Uplink1Len;i++)
		{
			printf("%02x",upLink1Pkt[i]);
		}
		printf("\r\n");
		
		node_gpio_set(mode, wakeup);
		node_gpio_set(mode, transparent);
		
		UART_RECEIVE_FLAG = 0;
		UART_RECEIVE_LENGTH = 0;
		memset(UART_RECEIVE_BUFFER, 0, sizeof(UART_RECEIVE_BUFFER));
		
		UART_WRITE_DATA((uint8_t*)upLink1Pkt, Uplink1Len);
		
		//数据接收
		timeout_start_flag = true;
		while(UART_RECEIVE_FLAG == 0)
		{
			if(true == time_out_break_ms(200))
			{
				break;
			}	
		}
		
		if(UART_RECEIVE_FLAG)
		{
			//帧缓存
			dataLen = UART_RECEIVE_LENGTH;
			for(i=0;i<dataLen;i++)
			{
				return_data[i] = UART_RECEIVE_BUFFER[i];
			}
			
			//调试日志
			if(logLevel == 2)
			{
				printf("get data：");
				for(i=0;i<dataLen;i++)
				{
					printf("%02x",return_data[i]);
				}
				printf("\r\n");	
			}
			
			//帧校验
			if(frameCheck(return_data)==0xF2)  //Downlink1
			{
				//帧内容处理
				rssiDtu = return_data[dataLen-2];  //信号质量记录
				snrDtu = return_data[dataLen-1]/4;
				rssiGw = return_data[11];
				snrGw = return_data[12];
									
				printf("rssiDtu: %d\r\n",(int8_t)rssiDtu);
				printf("snrDtu: %d\r\n",(int8_t)snrDtu);
				printf("rssiGw: %d\r\n",(int8_t)rssiGw);
				printf("snrGw: %d\r\n",(int8_t)snrGw);	
			}	
			else
			{
				printf("-pkt is not Downlink1\r\n");
			}
		}
		else
		{
			printf("-no response\r\n");
		}

		//数据发送-UpLink2
		upLink2Pkt[11] = rssiDtu;  //信号质量写入
		upLink2Pkt[12] = snrDtu;
		
		printf("\r\nsend uplink2pkt: ");  //帧打印
		for(i=0;i<Uplink2Len;i++)
		{
			printf("%02x",upLink2Pkt[i]);
		}
		printf("\r\n");
		
		node_gpio_set(mode, wakeup);
		node_gpio_set(mode, transparent);
		
		UART_RECEIVE_FLAG = 0;
		UART_RECEIVE_LENGTH = 0;
		memset(UART_RECEIVE_BUFFER, 0, sizeof(UART_RECEIVE_BUFFER));
		
		UART_WRITE_DATA((uint8_t*)upLink2Pkt, Uplink2Len);
		
		//数据接收
		timeout_start_flag = true;
		while(UART_RECEIVE_FLAG == 0)
		{
			if(true == time_out_break_ms(200))
			{
				break;
			}	
		}
		
		if(UART_RECEIVE_FLAG)
		{
			//帧缓存
			dataLen = UART_RECEIVE_LENGTH;
			for(i=0;i<dataLen;i++)
			{
				return_data[i] = UART_RECEIVE_BUFFER[i];
			}
			
			//调试日志
			if(logLevel == 2)
			{
				printf("get data：");
				for(i=0;i<dataLen;i++)
				{
					printf("%02x ",return_data[i]);
				}
				printf("\r\n");	
			}
			
			//帧校验
			if(frameCheck(return_data)==0xF4)  //Downlink2
			{
				radioTestPass = 1;
				break;
			}	
			else
			{
				printf("-pkt is not Downlink2\r\n");
			}
		}
		else
		{
			printf("-no response\r\n");
		}
	}
	
	if(radioTestPass)
	{
		printf("#RADIO TEST PASS\r\n");
		return true;
	}
	else
	{
		printf("#RADIO TEST FAIL\r\n");
		return false;
	}
}

void factoryTest(void)
{
	uint8_t return_data[50] = {0};
	uint16_t seed = 0;
	uint16_t random_t = 0;
	uint16_t i = 0;
	char hexData[2];
	uint8_t confResult[10]={0};
	uint8_t errorFlag=0;
	uint8_t P2P_DATA[10]={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
	bool rxTimeoutFlag = false;
	uint8_t sensorTestResult = 0x00;
	uint8_t handShakeResult = 0;
	
	//***LED状态***//
	ledStat = ON;
	
	//***P2P配置***//
	p2pSetup();
	
	//***DEVEUI获取***//
	node_gpio_set(mode, command);
	transfer_inquire_command("AT+DEVEUI?", return_data);
	for(i=0;return_data[i]!=NULL;i++)
	{
		deveuiBuf[i]=return_data[i];
	}
	
	//***握手尝试***//
	handShakeResult = handShakeAttempt();
	
	switch(handShakeResult)
	{
		case 0:  //握手失败
			
			ledStat = OFF;
			break;
		
		case 1:  //测试模式
			
			//报警器测试
			if(sensorTest()) sensorTestResult = 0xff;
			else sensorTestResult = 0x00;
			
			//射频通信测试
			if(radioTest(sensorTestResult) == true && sensorTestResult == 0xff)  //测试通过
			{
				ledStat = QUICKFLASH;
			}
			else  //测试失败
			{
				ledStat = OFF;
			}
			break;
		
		case 2:  //透传模式
			
			transperantMode();
			break;
	}
	
//	system_delay_ms(50000);  //测试结束后保持200s
//	system_delay_ms(50000);
//	system_delay_ms(50000);
//	system_delay_ms(50000);
}

