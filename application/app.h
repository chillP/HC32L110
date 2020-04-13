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

typedef struct {
	char dataContent[30];  //数据内容，如"CH4"
	uint16_t dataAvg[12];  
	uint16_t dataMax[12];  
}dataRecordType;





#endif
