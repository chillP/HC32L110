#ifndef _APP__H_
#define _APP__H_

#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>


void lorawanResetAndConfig(void);
void getSensorData_Task(void);
void networkConnect_Task(void);
void heartBeatReport_Task(void);
void errorReport_Task(void);
void powerDown_Task(void);
void powerOn_Task(void);
void factoryTest_Task(void);
void pdRecovery_Judge(void);
void ledStatHandle(void);
void getDeveui(void);
void factoryTest(void);
bool radioTest(uint8_t sensorTestResult);
uint8_t frameCheck(uint8_t* return_data);

typedef struct {
	char dataContent[30];  //数据内容，如"CH4"
	uint16_t dataAvg[12];  
	uint16_t dataMax[12];  
}dataRecordType;


typedef struct {
	uint8_t ver;
	uint8_t P2P_Mode;
	uint8_t P2P_SF;
	uint16_t P2P_FRQ;
	uint8_t P2P_FRQ_step;
	uint8_t P2P_PWR;
	uint8_t P2P_PWR_code;
	uint8_t P2P_TIME;
	uint8_t P2P_RSSI[5];
	uint8_t P2P_SNR[5];
} LoRaNode_P2P_Info;


#endif
