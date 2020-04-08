/*!
 * @file       app.c
 * @brief      Ӧ�ò㹦�ܵ�ʵ��
 * @details    Ӧ�ò�������ܣ�
				1�����ݻ�ȡ
				2��LoRaWAN����ά��
				3�������ϱ�
				4���澯�ϱ�
				5������澯
				6����������޸�
		   
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

bool onlineFlag = 0;
bool errorReportFlag=0;
uint16_t getDataCnt=0;
uint8_t saveDataCnt=0;
uint16_t dataMax=0;
uint16_t dataAvg=0;
uint32_t dataSum=0;
dataRecordType ch4DataBuf;

uint8_t getDataBuf[13];
uint8_t getCh4Cmd[23]={"*0401000000000000FA\r\n"};
uint8_t heartBeatBuf[34]={"0100000000000000000000000000000000"};

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

void lorawanResetAndConfig(void)
{
	printf("\r\n===Module Config===\r\n");
	printf("-reset lorawan module \r\n ");
	node_hard_reset();
	
	system_delay_ms(200);
	printf("-set basic para: \r\n");
	if(save_cmd_configure())
	{
		printf("OK\r\n");
	}
	else
	{
		printf("ERROR\r\n");		
	}
	transfer_configure_command("AT+DEBUG=1");
}

void getSensorData_Task(void)
{
	uint8_t checkSum = 0x00;
	uint16_t ch4Data = 0;
	char hexData[2];
	int i=0;
	
	//���۳ɹ���񣬽��뼴�ۼ�
	getDataCnt++;
	
	//COŨ�Ȳ�ѯ
	printf("\r\n===Data Fetch===\r\n");
	printf("-enquire sensor data(CH4) \r\n");
	UART_RECEIVE_FLAG = 0;
	uart0_RxByteCnt = 0;
	memset(uart0_RxBuf, 0, sizeof(uart0_RxBuf));
	
	Uart0_send_string(getCh4Cmd);
	
	while(UART_RECEIVE_FLAG == 0)
	{
		if(true == time_out_break_ms(2000))
		{
			break;
		}
	}
	UART_RECEIVE_FLAG = 0;
	printf("-get sensor data(CH4): %s",uart0_RxBuf);

	for(i=0;i<13;i++)
	{
		getDataBuf[i] = uart0_RxBuf[i];
	}
	
	//У��ͼ���
	
	//��ȡŨ��
	hexData[0]=getDataBuf[3];
	hexData[1]=getDataBuf[4];
	ch4Data = hexToDec(hexData);
	
	hexData[0]=getDataBuf[5];
	hexData[1]=getDataBuf[6];
	ch4Data += hexToDec(hexData)*256;
	printf("CH4: %.1f",ch4Data/10.0);  
	
	printf("%%");
	printf(" LEL\r\n");


//	//Ũ���ж�
//	if(ch4Data>100)  //����10%LEL
//	{
//		errorReportFlag = 1;
//	}
	
	//��ȡ����״̬
	switch(getDataBuf[2])  //��ȡ��2λ������CO���� ��|0X0C
	{
		case '0': //����
			printf("CH4-STAT:OK\r\n");
			break;
		
		case '4': //CH4Ũ�ȱ���
			printf("CH4-STAT:EXCEED!\r\n");
			alarmReportFlag_Lel = 1;
			break;
		
		case '8': //CH4����������
			printf("CH4-STAT:SENSOR FAULT!\r\n");
			alarmReportFlag_Sensor = 1;
			break;		
	}
	
	//��ȡ����״̬
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
	
	//���ݻ�ȡ - 5s����
	if(ch4Data > dataMax) dataMax = ch4Data;  //��¼��ֵ
	dataSum += ch4Data;  //�ۼӺ�
	
	//���ݴ洢 - 15s����
	if(getDataCnt>=3)  
	{
		getDataCnt = 0;
		
		
		dataAvg = dataSum/3.0;  

		ch4DataBuf.dataAvg[saveDataCnt] = dataAvg;  //�����ֵ
		ch4DataBuf.dataMax[saveDataCnt] = dataMax;  //�����ֵ
		
		printf("\r\n===Data Storage===\r\n");
		printf("-data stored!\r\n");
		printf("Avg: %.1f\r\n",dataAvg/10.0);  
		printf("Max: %.1f\r\n",dataMax/10.0);  
		
		dataSum = 0;
		dataMax = 0;
		saveDataCnt++;
	}
	
	//�����ϱ� - 120s����
	if(saveDataCnt>=8)  
	{
		saveDataCnt = 0;
		heartbeatReport_Tp = 1;
	}
		
	
//	for(i=0;i<uart0_RxByteCnt/2;i++)
//	{
	//	printf("%d ",hexToDec((char*)uart0_RxBuf[3])*16);  //��4bit
	//	printf("%d ",hexToDec((char*)uart0_RxBuf[4]));  //��4bit
		
//		checkSum += hexToDec((char*)uart0_RxBuf[i])+16;  //��4bit
//		checkSum += hexToDec((char*)uart0_RxBuf[i+1]);  //��4bit
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
		vdetectEnable = 1;  //��������澯
		ledStat = BREATH;  //����������
	}
	else
	{
		printf("ERROR\r\n");
		printf("-rejoin after 10s \r\n");
		timeout_start_flag = true;
		while(1)
		{
			if(true == time_out_break_ms(10000))
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
	
	//��ʷ���ݴ���
	for(i=0;i<8;i++)
	{
		sprintf((char*)ch4DataAvg,"%02x",(uint32_t)ch4DataBuf.dataAvg[i]);
		sprintf((char*)ch4DataMax,"%02x",(uint32_t)ch4DataBuf.dataMax[i]);
		
		heartBeatBuf[4*i+2] = ch4DataAvg[0];
		heartBeatBuf[4*i+3] = ch4DataAvg[1];
		heartBeatBuf[4*i+4] = ch4DataMax[0];
		heartBeatBuf[4*i+5] = ch4DataMax[1];
	}
	printf("heartBeatBuf: %s\r\n",heartBeatBuf);		
	
	UART_RECEIVE_FLAG = 0;
	UART_RECEIVE_LENGTH = 0;
	memset(UART_RECEIVE_BUFFER, 0, sizeof(UART_RECEIVE_BUFFER));
	
	node_join_successfully=1;
	node_gpio_set(wake, wakeup);  //����
	
	printf("\r\n===Data Report===\r\n");
	printf("-report heartbeat: \r\n");
	send_result = node_block_send(CONFIRM_TYPE | 0x01, (uint8_t*)"01234567890123456789", 20, &head);
	
	//node_gpio_set(wake, sleep);  //����
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
	
	system_delay_ms(200);  //�ȴ�ģ����־������
	
	if(send_result == 1)
	{
		printf(" OK\r\n"); 
	}
	else
	{
		printf(" ERROR: %d \r\n",send_result); 
	}	
}

bool errorReport_Task(void)
{
	return 1;
}

void powerDown_Task(void)
{
	uint8_t i=0;
	down_list_t *head = NULL;
	execution_status_t send_result;
	
	
	ledStat = QUICKFLASH;
	printf("power-down detected!!!\r\n"); 
	
//	//FLASH�洢������
//	EeBuf[0]=0x11;
//	HDEE_Write( 0x00 , EeBuf , HDEE_EeSize );
	
	UART_RECEIVE_FLAG = 0;
	UART_RECEIVE_LENGTH = 0;
	memset(UART_RECEIVE_BUFFER, 0, sizeof(UART_RECEIVE_BUFFER));
	
	node_join_successfully=1;
	
	for(i=0;i<3;i++)
	{
		//node_gpio_set(wake, wakeup);  //����
		
		printf("\r\n===Data Report===\r\n");
		printf("-report heartbeat: \r\n");
		send_result = node_block_send(CONFIRM_TYPE | 0x01, (uint8_t*)"powerdown", 9, &head);
		
		node_gpio_set(wake, sleep);  //����
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
		
		system_delay_ms(200);  //�ȴ�ģ����־������
		
		if(send_result == 1)
		{
			printf(" OK\r\n"); 
		}
		else
		{
			printf(" ERROR: %d \r\n",send_result); 
		}				
	}

	
}

void powerOn_Task(void)
{
	
	//FLASH��ȡ������
	
}

void factoryTest_Task(void)
{
	//vdetectEnable = 0;  //�رյ�����
}

void pdRecovery_Judge(void)
{
	//powerDownFlag = 0;
	HDEE_Read( 0x00 , EeBuf , HDEE_EeSize );
	if(EeBuf[0]==0x11) printf("===recover from powerdown===\r\n");
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
	
	switch(ledStat)
	{
		case ON: //����
			Pca_CapDataHSet(Module4, 200);	
			lastLedState = ledStat;
			break;
		case OFF:  //Ϩ��
			Pca_CapDataHSet(Module4, 0);	
			lastLedState = ledStat;
			break;
		case BREATH:  //10s���� 
			breathCnt++;
			if(breathCnt>=200) breathCnt = 0;
			y = 100*sin(0.015*breathCnt) - 50;
			Pca_CapDataHSet(Module4, y);
			lastLedState = ledStat;		
			break;
		case BLINK:  //20ms��˸һ��
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
		case QUICKFLASH:  //500ms����
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
		case SLOWFALSH:  //1s���� 	
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
	char hexData[2];
	
	transfer_inquire_command("AT+DEVEUI?", return_data);

	hexData[0]=return_data[22];
	hexData[1]=return_data[23];
	
	seed = hexToDec(hexData);
	srand( seed );                  
  random_t = rand() % 120;  //����0-120�������
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
