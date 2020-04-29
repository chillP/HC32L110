/*!
 * @file       LSD_LoRaWAN_ICA_Driver.c
 * @brief      LSD_LoRaWAN_ICA_Driver是针对lierda LoRaWAN ICA模块的驱动程序
 * @details    该文件为用户提供一个对LoRaWAN模块标准操作的驱动流程。将模块各种复杂,的操作封装成函数接口，供用户调用，从而缩短产品开发周期
 * @copyright  Revised BSD License, see section LICENSE. \ref LICENSE
 * @author     Lierda-WSN-LoRaWAN-毛庭
 * @date       2019-07-23
 * @version    V1.1.6
 */

/*!
 * @defgroup  ICA-Driver LoRaWAN_ICA_Node_Driver API
 * @brief     该文件为用户提供一个针对LoRaWAN_ICA模块的串口驱动。
 * @{
 */   
#include "LSD_LoRaWAN_ICA_Driver.h"

/** @defgroup ICA_Driver_Exported_Variables ICA_Driver_Variables
  * @brief ICA驱动程序提供给用户的变量
  * @{
  */
  
/** 设置超时开始标致 */
bool timeout_start_flag = true;
/** 模块是否已成功入网 */
bool node_join_successfully = false;
/** 确认帧连续失败次数 */
uint8_t confirm_continue_failure_count = 0;
/** 最后一条上行数据的速率 */
int8_t last_up_datarate = -1;
/** 终端不复位的情况下对模块复位的信号量 */
node_reset_single_t node_reset_signal;
//LED状态
extern ledStatType ledStat;

/**
  * @}
  */ 
  
const char *at_command_array_save[] = {AT_COMMAND_AND_SAVE};
const char *at_command_array_not_save[1] = {AT_COMMAND_NOT_SAVE};
 
bool unsave_cmd_configure(void);
void free_down_info_list(down_list_t **list_head, free_level_t free_level);

/**
*  发送用户数据给模块
*@param 	buffer: 要发送的数据内容
*@param		size: 数据的长度
*/
static void transfer_node_data(uint8_t *buffer, uint8_t size)
{	
	UART_RECEIVE_FLAG = 0;
	UART_WRITE_DATA(buffer, size);
}

/**
*  超时函数
*@param 	time: 设置的具体超时时间值，单位毫秒
*@return	是否到设定的超时时间
*/
bool time_out_break_ms(uint32_t time)
{
	static uint32_t start_time;
	
	if(true == timeout_start_flag)
	{
		start_time = GET_SYSTEM_TIME;
		timeout_start_flag = false;
	}
//	if(SUBTRANCTION_CORSS_ZERO(GET_SYSTEM_TIME, start_time) >= time)
//	{
//		timeout_start_flag = true;
//		return true;
//	}
	if(GET_SYSTEM_TIME-start_time >= time)
	{
		timeout_start_flag = true;
		return true;
	}
	return false;
}

/**
*  延时函数
*@param 	delay: 延时的时间，单位毫秒
*/
void system_delay_ms(uint32_t delay) 
{
	uint32_t tickstart = 0U;
	tickstart = GET_SYSTEM_TIME;
	while((GET_SYSTEM_TIME - tickstart) < delay);
}

/**
*  设置模块的引脚电平
*@param 	type: 表示设置的是WAKE引脚还是STAT引脚
*@param 	value: 表示设置对应引脚的高低电平
*/
void node_gpio_set(node_gpio_t type, node_status_t value)
{
	if(mode == type)
	{
		if(command == value)
		{
			SET_MODE_HIGH;
		}
		else if(transparent == value)
		{
			SET_MODE_LOW;
		}
	}
	else if(wake == type)
	{
		if(wakeup == value)
		{
			SET_WAKE_HIGH;
		}
		else if(sleep == value)
		{
			SET_WAKE_LOW;
		}		
	}

	system_delay_ms(10);
}

/**
*  读取模块引脚的电平
*@param 	gpio: 表示要读取的是STAT引脚还是BUSY引脚
*@param		d: 要查找的字符串
*@return	对应引脚的电平
*/
static gpio_level_t node_gpio_read(node_gpio_t gpio)
{
	if(stat == gpio)
	{
		return (gpio_level_t)GET_STAT_LEVEL;
	}
	else if(busy == gpio)
	{
		return (gpio_level_t)GET_BUSY_LEVEL;
	}
	
	return unknow;
}

/**
*  将小写字母转为大写并删除空格
*@param 	[IN]src: 原始字符串
*@param 	[OUT]des: 目标地址
*@return	无
*/
static void lower2upper_and_remove_spaces(uint8_t *src,  uint8_t *des)
{
    do
    {
        if(*src >= 'a' && *src <= 'z')
        {
            *des++ = *src - 'a' + 'A';
        }
        else if(' ' != *src)
        {
            *des++ = *src;
        }
        
    }while(*src++);	
}

/**
*  查找一个字符串是否在另一个字符串中出现
*@param 	s: 原始字符串
*@param 	d: 要查找的字符串
*@return	返回查找的结果
*@retval		NULL: d字符串未出现在s字符串
*@retval		s:    d字符串出现在s字符串,且返回当前指针s所指向的地址
*/
uint8_t* find_string(uint8_t *s, uint8_t *d)
{
    uint8_t *tmp;
    
    while(0 != *s && 0 != *d)
    {
        tmp = d;
        while(*s == *tmp && *s && *tmp)
        {
            s++;
            tmp++;
        }
        
        if(0 == *tmp)
        {
            return s;
        }
        s++;
    }
    return NULL;
}

/**
*  截取指定开头和结尾的中间字符
*@param		str: 原始字符串
*@param 	s: 开始字符串
*@param		e: 结尾字符串
*@param		res: 截取到的字符串
*/
static void match_string(uint8_t *str, uint8_t *s, uint8_t *e, uint8_t *res)
{
    uint8_t *first_result = NULL;
    uint8_t *f_t = NULL, *tmp = NULL;
    uint8_t i = 0, t_i = 0, result_flag = 0;

    first_result = find_string(str, s);

    if(NULL != first_result)
    {
        f_t = first_result;
        while(0 != *f_t && 0 != *e)
        {
            tmp = e;
            t_i = 0;
            while(*f_t == *tmp && *f_t && *tmp)
            {
                f_t++;
                tmp++;
                t_i++;
            }

            if(0 == *tmp)
            {
				result_flag = 1;
                break;
            }
            else if(t_i > 0)
            {
                f_t--;
                t_i--;
            }

            f_t++;
            i += t_i + 1;
        }

        while(i > 0 && result_flag)
        {
            *res = *first_result;
            res ++;
            first_result ++;
            i--;
        }
    }
}

/**
*  十六进制字符串转整数
*@param		s: 十六进制字符串
*@param 	s: 转换字符的升序
*@return	整数结果
*/
static uint32_t htoi(uint8_t s[], uint8_t size)  
{  
    uint8_t i = 0;  
    uint32_t  n = 0;  
	
	
    for (i = 0; i < size; i++)  
    {  
		s[i] = s[i] >= 'A' && s[i] <= 'Z' ? s[i] + 'a' - 'A' : s[i];

		if((s[i] >= '0' && s[i] <= '9') || (s[i] >= 'a' && s[i] <= 'z'))
		{
			if (s[i] > '9')  
			{
				n = (n << 4) + (10 + s[i] - 'a');  
			}
			else  
			{  
				n = (n << 4) + (s[i] - '0');  
			} 			
		}
		else
		{
			break;
		}
    }

    return n;  
}  

/**
* 查询模块是否开启热启动以及开启热启动的情况下是否已入网，该函数适用于对模块复位而用户MCU不复位的情况
*@return    查询指令是否正常执行
*/
static bool hot_start_check(void)
{
	uint8_t return_data[50] = {0};
	bool result_tmp = true;
	// 模块首次上电或复位会调用hot_start_check()函数，故将该信号量置位
	node_reset_signal.value = 0xff;
	
	result_tmp &= transfer_inquire_command("AT+OTAA?", return_data);
	
	if(true == result_tmp && '1' == return_data[4])
	{
		result_tmp &= transfer_inquire_command("AT+JOIN?", return_data);

		if(true == result_tmp && '1' == return_data[1])
		{
			node_join_successfully = true;
		}
		else
		{
			node_join_successfully = false;
		}
	}
	
	//unsave_cmd_configure();
	result_tmp &= save_cmd_configure();
	return result_tmp;
}

/**
*  执行模块配置指令，自带模式切换
*@param		cmd: 需要执行的配置指令
*@return	指令执行结果
*@retval		true:  执行成功
*@retval		false: 执行失败
*/
bool transfer_configure_command(char *cmd)
{
	uint8_t *result = NULL;
	uint16_t timeouts = 0;
	char tmp_cmd[255];
	
	node_gpio_set(wake, wakeup);
	node_gpio_set(mode, command);
	
	lower2upper_and_remove_spaces((uint8_t *)cmd, (uint8_t *)tmp_cmd);

	//Debug Info Print
	if(logLevel == 2)
	{
		printf("-send: %s",tmp_cmd);
	}		
	
	strcat(tmp_cmd, "\r\n");
	
	UART_RECEIVE_FLAG = 0;
	UART_RECEIVE_LENGTH = 0;
	memset(UART_RECEIVE_BUFFER, 0, sizeof(UART_RECEIVE_BUFFER));
	
	UART_WRITE_STRING((uint8_t *)tmp_cmd);
	
	if(NULL != find_string((uint8_t *)tmp_cmd, (uint8_t *)"AT+SAVE"))
	{
		timeouts = 2000;
	}
	else
	{
		timeouts = 200; //原50 APPKEY配置需150ms
	}

	timeout_start_flag = true;
	while(UART_RECEIVE_FLAG == 0)
	{
		if(true == time_out_break_ms(timeouts))
		{
			break;
		}
	}
	
	UART_RECEIVE_FLAG = 0;
	
	result = find_string(UART_RECEIVE_BUFFER, (uint8_t *)"OK");
	
	//Debug Info Print
	if(logLevel == 2)
	{
		if(find_string(UART_RECEIVE_BUFFER, (uint8_t *)"\r\n")!=NULL)  //若为指令响应则换行，若为数据输出不换行
		{
			printf("\r\n");
		}		
		printf("-receive: %s",UART_RECEIVE_BUFFER);
	}	
	
#ifdef DEBUG_LOG_LEVEL_1	
	if(NULL == result)
	{
		//DEBUG_PRINTF ("Command \"%s\" Execution failed, Module returns results: %s\r\n", cmd, UART_RECEIVE_BUFFER);
		printf("0\r\n");
	}
	else
	{
		printf("1\r\n");
	}
#endif
	
	if(NULL != find_string((uint8_t *)tmp_cmd, (uint8_t *)"AT+RESET") && NULL != result)
	{
		system_delay_ms(150);
		//hot_start_check();
	}
	
	if(NULL != find_string((uint8_t *)tmp_cmd, (uint8_t *)"AT+FACTORY") && NULL != result)
	{
		system_delay_ms(2000);
	}

	return result;
}

/**
*  执行模块查询指令，自带模式切换
*@param		cmd: 需要执行的查询指令
*@return	result: 查询指令的返回结果
*@return	指令执行结果
*@retval		true:  执行成功
*@retval		false: 执行失败
*/
bool transfer_inquire_command(char *cmd, uint8_t *result)
{
	uint8_t cmd_content[20] = {0}, i = 0;
	char tmp_cmd[255];
	
	node_gpio_set(wake, wakeup);
	node_gpio_set(mode, command);
	
	lower2upper_and_remove_spaces((uint8_t *)cmd, (uint8_t *)tmp_cmd);
	strcat(tmp_cmd, "\r\n");
	
	UART_RECEIVE_FLAG = 0;
	UART_RECEIVE_LENGTH = 0;
	memset(UART_RECEIVE_BUFFER, 0, sizeof(UART_RECEIVE_BUFFER));
	
	UART_WRITE_STRING((uint8_t *)tmp_cmd);
	
	timeout_start_flag = true;
	while(UART_RECEIVE_FLAG == 0)
	{
		if(true == time_out_break_ms(500))
		{
			break;
		}
	}
	UART_RECEIVE_FLAG = 0;
	
	match_string((uint8_t *)cmd, (uint8_t *)"AT", (uint8_t *)"?", cmd_content);	
	
	while(0 != cmd_content[++i]);
	cmd_content[i++] = ':';
	
	if(NULL != find_string(UART_RECEIVE_BUFFER, (uint8_t *)"OK"))
	{
		match_string(UART_RECEIVE_BUFFER, cmd_content, (uint8_t *)"OK", result);
		return true;
	}
#ifdef DEBUG_LOG_LEVEL_1	
	else
	{	
		DEBUG_PRINTF ("Command \"%s\" Execution failed, Module returns results: %s\r\n", cmd, UART_RECEIVE_BUFFER);
	}
#endif	
	
	return false;
}

/**
* 下行接收数据处理
*@param		list_head: 指向下行信息的链表头的指针
*@return    无
*/
static void down_data_process(down_list_t **list_head)
{
	down_list_t *p1 = NULL;
	down_list_t *p2 = NULL;
	uint8_t cnt = 0;
	
#ifdef USE_NODE_STATUS		
	// 若这两个条件均不满足，则表示这不是最后一个下行包，且无业务数据
	if(UART_RECEIVE_LENGTH > 5 || high == node_gpio_read(busy)) 
	{
#endif
		if(NULL != *list_head)
		{
			p2 = *list_head;
			
			while(p2->next)
			{
				p2 = p2->next;
			}
		}
					
		p1 = (down_list_t *)malloc(sizeof(down_list_t));
		
		if(NULL != p1)
		{
#ifdef USE_NODE_STATUS
			p1->down_info.size = UART_RECEIVE_LENGTH - 5;
#else
			p1->down_info.size = UART_RECEIVE_LENGTH;
#endif				
			if(0 == p1->down_info.size)
			{
				p1->down_info.business_data = NULL;
			}
			else
			{
				p1->down_info.business_data = (uint8_t *)malloc(p1->down_info.size);
			}		
			
			if(NULL == p1->down_info.business_data && 0 != p1->down_info.size)  
			{
				free(p1);
#ifdef DEBUG_LOG_LEVEL_0
				DEBUG_PRINTF ("Error：Dynamic application for memory failure -- p1->down_info.business_data\r\n");
#endif											
			}
			else
			{	
#ifdef USE_NODE_STATUS
				for(cnt = 1; cnt <= p1->down_info.size; cnt++)
				{
					*(p1->down_info.business_data + cnt - 1) = UART_RECEIVE_BUFFER[cnt];
				}	
				p1->down_info.result_ind = UART_RECEIVE_BUFFER[cnt++];
				p1->down_info.snr = UART_RECEIVE_BUFFER[cnt++];
				p1->down_info.rssi = UART_RECEIVE_BUFFER[cnt++];
				p1->down_info.datarate = UART_RECEIVE_BUFFER[cnt] & 0x0f;
#else
				for(cnt = 0; cnt < p1->down_info.size; cnt++)
				{
					*(p1->down_info.business_data + cnt) = UART_RECEIVE_BUFFER[cnt];
				}						
#endif	
				p1->next = NULL;
				
				if(NULL != *list_head)
				{
					p2->next = p1;	
				}
				else
				{
					*list_head = p1;
				}					
			}
		}
		else
		{
#ifdef DEBUG_LOG_LEVEL_0
			DEBUG_PRINTF ("Error：Dynamic application for memory failure -- p1\r\n");
#endif
		}
#ifdef USE_NODE_STATUS
	}
		last_up_datarate = UART_RECEIVE_BUFFER[UART_RECEIVE_LENGTH - 1] & 0x0f;
#endif				
}


/**
* 发送最一个分包并等待AS应答（仅供node_block_send_big_packet函数调用）
*@param		packet_end: 最后一个分包的地址
*@param		size: 最后一个分包的长度
*@param		packet_mark: 大数据包的序号
*@param		resend_num: 最后一个小数据包在未收到应用服务器（AS）特定应答的情况下，最后一个小数据包的重发次数
*@return    AS对所有分包的接收结果
*/
static uint32_t wait_as_respone(uint8_t *packet_end, uint8_t size, uint8_t packet_mark, uint8_t resend_num, down_list_t **list_head)
{
	uint8_t resend_count = 0;
	uint32_t receive_success_bit = 0;
	bool receive_as_answer = false;
	bool first_point = true;
	down_list_t *_list_head = NULL;
	down_list_t *list_tmp = NULL;
	down_list_t *list_storage = NULL;

	while(resend_count++ <= resend_num)
	{
#ifdef DEBUG_LOG_LEVEL_1
		uint8_t tmp = 0;
		DEBUG_PRINTF("[SEND-LAST] ");
		for(tmp = 0; tmp < size; tmp++)
		{
			DEBUG_PRINTF("%02x ", packet_end[tmp]);
		}
		DEBUG_PRINTF("\r\n");
#endif
		
		_list_head = NULL;
		node_block_send(UNCONFIRM_TYPE | 0x01, packet_end, size, &_list_head);	

		list_tmp = _list_head;
		
		while(NULL != list_tmp)
		{			
			if(list_tmp->down_info.size > 0)
			{
				if(0xBB == list_tmp->down_info.business_data[0] && packet_mark == list_tmp->down_info.business_data[1] &&
				   0xAA == list_tmp->down_info.business_data[list_tmp->down_info.size - 1] && 7 == list_tmp->down_info.size)
				{
					receive_success_bit = list_tmp->down_info.business_data[2] | (list_tmp->down_info.business_data[3] << 8) | \
										 (list_tmp->down_info.business_data[4] << 16) | (list_tmp->down_info.business_data[5] << 24);
					// 分包协议数据不计入业务数据，但可能是大包中的最后一个下行，故暂保留其它状态值
					list_tmp->down_info.size = 0;
					receive_as_answer = true;
				}
				
#ifndef USE_NODE_STATUS
				else
				{
#endif
					if(0 != list_tmp->down_info.size || NULL == list_tmp->next)
					{
						if(true == first_point)
						{
							first_point = false;
							list_storage = list_tmp;
							*list_head = list_storage;
						}
						else
						{
							list_storage->next = list_tmp;
							list_storage = list_storage->next;						
						}						
					}
					
#ifndef USE_NODE_STATUS
				}
#endif
			}
#ifdef USE_NODE_STATUS
			else
			{
				if(true == receive_as_answer)
				{
					list_storage->next = list_tmp;
					list_storage = list_storage->next;					
				}
				else 
					if(resend_count > resend_num)
				{
					if(true == first_point)
					{
						first_point = false;
						list_storage = list_tmp;
						*list_head = list_storage;	
					}
					else
					{
						list_storage->next = list_tmp;
						list_storage = list_storage->next;	
					}					
				}
			}
#endif
			list_tmp = list_tmp->next; 
		}

		if(resend_count > resend_num)
		{
			free_down_info_list(&_list_head, save_data_and_last);
		}
		else
		{
			if(true == receive_as_answer)
			{
				free_down_info_list(&_list_head, save_data_and_last);
				break;
			}
			else
			{
				free_down_info_list(&_list_head, save_data);
			}
		}
	}
	
	return receive_success_bit;
}

/**
* 解析部分查询指令的返回值（仅供驱动内部调用）
*/
static uint8_t handle_cmd_return_data(char *data, uint8_t start, uint8_t length)
{
	static uint8_t inquire_return_data[150];
	
	memset(inquire_return_data, 0, 150);
	transfer_inquire_command(data, inquire_return_data);

	return htoi((inquire_return_data + start), 2);	
}

/**
* 模块硬件复位，不建议用户直接使用该函数，建议用户使用node_hard_reset_and_configure()来硬复位模块
*/
void node_hard_reset(void)
{
	// 模块复位前将WAKE引脚拉高（重要）
	node_gpio_set(wake, wakeup);
	SET_RESET_LOW;
	system_delay_ms(15);
	SET_RESET_HIGH;
	system_delay_ms(200);	
}


/*------------------------------------------------------------------------------
|>                        以下为给用户提供的调用函数                          <|
------------------------------------------------------------------------------*/

/** @defgroup ICA_Driver_Exported_Functions ICA_Driver_API_Interface
  * @brief ICA模块驱动的用户API接口,主要包括操作模块进行复位、AT参数配置、入网、发送数据等
  * @{
  */

/**
 * @brief   硬件复位
 * @details 通过Reset引脚复位模块.
 * @param   无.
 * @return  无
 */
void node_hard_reset_and_configure(void)
{
	node_hard_reset();
	hot_start_check();
}

/**
 * @brief   配置模块不保存的指令
 * @details 配置一些模块掉电不保存的指令，该函数适用于对模块复位而用户MCU不复位的情况
 * @param   无
 * @return  指令配置结果
 */

bool unsave_cmd_configure(void)
{
	bool result_tmp = true;
	uint8_t i = 0;
	
	for(i = 0; i < sizeof(at_command_array_not_save) / sizeof(char *); i++)
	{
		if(NULL != at_command_array_not_save[i])
		{
			result_tmp &= transfer_configure_command((char *)at_command_array_not_save[i]);
		}
	}
	
	return result_tmp;
}


/**
 * @brief   配置模块保存的指令
 * @details 该函数为针对那些保存的指令，配置成功后，再次上电或复位不再调用该函数。用户在次函数中添加一些需要的指令
 * @param   无.
 * @return  指令执行结果
 * @retval		true:  所有指令均正确执行
 * @retval		false: 一条或多条指令执行出现异常
 */
bool save_cmd_configure(void)
{
	bool result_tmp = true;
	uint8_t i = 0;
	
	for(i = 0; i < sizeof(at_command_array_save) / sizeof(char *); i++)
	{
		if(NULL != at_command_array_save[i])
		{
			result_tmp &= transfer_configure_command((char *)at_command_array_save[i]);
		}
	}

#ifdef USE_NODE_STATUS
	/* 使用本驱动时，若开启模块STATUS状态输出功能，以下两条指令不能删除 */
	result_tmp &= transfer_configure_command("AT+STATUS=1,1");
	result_tmp &= transfer_configure_command("AT+STATUS=2,2");
#else
	result_tmp &= transfer_configure_command("AT+STATUS=1,0");
	result_tmp &= transfer_configure_command("AT+STATUS=2,0");
#endif
	result_tmp &= transfer_configure_command("AT+SAVE");
	result_tmp &= transfer_configure_command("AT+RESET");
	
	return result_tmp;	
}

/**
 * @brief   模块上电初始
 * @details 用户MCU上电时对模块配置一些保存的指令（这些指令由用户根据相应的需要补充）
 * @param   无.
 * @return	指令执行结果
 * @retval		true:  所有指令均正确执行
 * @retval		false: 一条或多条指令执行出现异常
 */
bool node_configure(void)
{
	bool result_tmp = true;
	uint8_t result[200] = {0};
	
	node_hard_reset();
	
#ifdef DEBUG_LOG_LEVEL_0
	DEBUG_PRINTF("Driver Version Info: %s\r\n", DRIVER_VERSION_INFO);
	result_tmp &= transfer_inquire_command("AT+VER?", result);
	DEBUG_PRINTF("Firmware Version Info: %s\r\n", result);	
#endif
	
	
	/*该函数为一次性调用，配置成功后再次上电或复位不再调用该函数，可在FLASH中保存一个变量，来判断该函数有没有被调用过(用户自行实现)*/
	result_tmp &= save_cmd_configure();
	
	
	/*该函数为每次上电或复位均需调用*/
	result_tmp &= hot_start_check();
	
	return result_tmp;
}

/**
 * @brief   入网扫描
 * @details 以阻塞的形式进行入网扫描
 * @param   [IN]time_second: 入网超时时间，单位：秒
 * @return  返回入网的结果
 * @retval		true:  入网成功
 * @retval		false: 入网失败
 *
 * @par 示例:
 * 下面显示如何调用该API，进行入网扫描
 * @code
 * // 复位节点并入网扫描，超时时间设置为300秒（超时时间可平衡功耗与入网成功率设定）
 * if(node_block_join(300) == true)
 * {
 *   // 已成功入网
 * }
 * else
 * {
 *	 // 入网失败
 * }
 * @endcode
 */
bool node_block_join(uint16_t time_second)
{
	gpio_level_t stat_level;
	gpio_level_t busy_level;
	
#ifdef DEBUG_LOG_LEVEL_1
	DEBUG_PRINTF("Start to join...\r\n");
#endif

	// 1.切换到透传模式开始入网
	node_gpio_set(wake, wakeup);
	node_gpio_set(mode, transparent);
	
	UART_RECEIVE_FLAG = 0;
	UART_RECEIVE_LENGTH = 0;
	memset(UART_RECEIVE_BUFFER, 0, sizeof(UART_RECEIVE_BUFFER));
	
	timeout_start_flag = true;
	do
	{
		//Debug Info Print
		if(logLevel == 2)
		{
			if(1 == UART_RECEIVE_FLAG)
			{
				printf("-receive: %s\r\n",UART_RECEIVE_BUFFER);
				UART_RECEIVE_FLAG = 0;
				UART_RECEIVE_LENGTH = 0;
				memset(UART_RECEIVE_BUFFER, 0, sizeof(UART_RECEIVE_BUFFER));
			}		
		}
		
		// 2.循环查询STAT和BUSY引脚电平
		stat_level = node_gpio_read(stat);
		busy_level = node_gpio_read(busy);
		
		// 3.到达设定时间后若未入网则超时返回
		if(true == time_out_break_ms(time_second * 1000))
		{
			node_gpio_set(wake, sleep);
			//DEBUG_PRINTF("Join failure\r\n");
			return false;
		}	
	}while(high != stat_level || high != busy_level);

	//Debug Info Print
	if(logLevel == 2)
	{
		UART_RECEIVE_FLAG = 0;
		timeout_start_flag = true;
		while(0 == UART_RECEIVE_FLAG && false == time_out_break_ms(100));

		if(1 == UART_RECEIVE_FLAG)
		{
			printf("-receive: %s\r\n",UART_RECEIVE_BUFFER);
			UART_RECEIVE_FLAG = 0;
			UART_RECEIVE_LENGTH = 0;
			memset(UART_RECEIVE_BUFFER, 0, sizeof(UART_RECEIVE_BUFFER));
		}	
	}	
	
#ifdef USE_NODE_STATUS
	UART_RECEIVE_FLAG = 0;
	
	timeout_start_flag = true;
	while(0 == UART_RECEIVE_FLAG && false == time_out_break_ms(50));
	
	if(1 == UART_RECEIVE_FLAG)
	{
		UART_RECEIVE_FLAG = 0;
		last_up_datarate = UART_RECEIVE_BUFFER[4];
	}
#endif	

#ifdef DEBUG_LOG_LEVEL_1
	DEBUG_PRINTF("Join seccussfuly\r\n");
#endif
	// 5.若未超时返回则表明模块入网成功
	system_delay_ms(200);  //等待join信息打印完毕
	return true;
}

/**
 * @brief   复位节点并入网扫描
 * @details 复位节点并以阻塞的形式进行入网扫描
 * @param   [IN]time_second: 入网超时时间，单位：秒
 * @return  返回入网的结果
 * @retval		true:  入网成功
 * @retval		false: 入网失败
 *
 * @par 示例:
 * 下面显示如何调用该API，进行复位节点并入网扫描
 * @code
 * // 复位节点并入网扫描，超时时间设置为300秒（超时时间可平衡功耗与入网成功率设定）
 * node_reset_block_join(300);
 * if(true == node_join_successfully)
 * {
 *   // 已成功入网
 * }
 * else
 * {
 *	 // 入网失败
 * }
 * @endcode
 */
void node_reset_block_join(uint16_t time_second)
{
	node_join_successfully = false;
	
	// 硬件复位模块, 并配置一些掉电不保存的指令
	node_hard_reset_and_configure();
	
	if(false == node_join_successfully)
	{
		node_join_successfully = node_block_join(time_second);
	}

	node_gpio_set(wake, sleep);
}

/**
 * @brief   触发模块在热启动下重新入网
 * @details 模块处于热启动且已入网时，触发模块重新入网，可通过node_join_successfully的值判断是否成功入网
 * @param   [IN]time_second: 入网超时时间，单位：秒
 * @return  返回入网的结果
 * @retval		true:  入网成功
 * @retval		false: 入网失败
 *
 * @par 示例:
 * 下面显示如何调用该API，进行热启动下重新入网
 * @code
 *
 * if(hot_start_rejoin(300) == true && node_join_successfully == true)
 * {
 *   // 已重新入网
 * }
 * else
 * {
 *	 // 重新入网失败
 * }
 * @endcode
 */
bool hot_start_rejoin(uint16_t time_second)
{
	bool cmd_result = transfer_configure_command("AT+JOIN=1");
	
	if(true == cmd_result)
	{
		node_join_successfully = node_block_join(time_second);
	}
	
	return cmd_result;
}

/**
 * @brief   释放下行信息的链表资源
 * @details 释放下行信息的链表资源 在node_block_send()和node_block_send_big_packet()中已调用该函数释放相应的资源
 * @param   [IN]list_head: 链表表头
 * @param   [IN]free_level: 释放等级
 *			- all_list：按顺序全部释放
 *			- save_data：保留链表中data不为空的资源
 *			- save_data_and_last：保留链表中data不为空的资源和最后一条资源
 * @return  无
 *
 * @par 示例:
 * 下面显示如何调用该API，释放链表资源
 *
 * @code
 * //释放list_head所指向的链表资源
 * free_down_info_list(&list_head, all_list);
 * @endcode
 */
void free_down_info_list(down_list_t **list_head, free_level_t free_level)
{
	down_list_t *p1 = NULL;
	down_list_t *p2 = NULL;
	
	p1 = *list_head;
	
	while(p1)  
    {  
        p2 = p1->next;  
		
		switch(free_level)
		{
			case all_list:
				if(NULL != p1->down_info.business_data)
				{
					free(p1->down_info.business_data);
				}
				free(p1); 
				break;
			case save_data:
				if(p1->down_info.size == 0)
				{
					if(NULL != p1->down_info.business_data)
					{
						free(p1->down_info.business_data);
					}
					free(p1);  
				}	
				break;
			case save_data_and_last:
				if(p1->down_info.size == 0 && NULL != p2)
				{
					if(NULL != p1->down_info.business_data)
					{
						free(p1->down_info.business_data);
					}
					free(p1);  
				}	
				break;
		}
        
        p1 = p2;  
    }  
	
	*list_head = NULL;
}

/**
 * @brief   以阻塞的形式发送数据
 * @details 
 * @param	[IN]frame_type: 十六进制，表示本次发送的帧类型和发送次数\n
						高四位：要发送的帧为确认帧(0001)还是非确认(0000),\n
						低四位：若确认帧，则表示未收到ACK的情况下共发送的次数；若为非确认，则表示总共需发送的次数
 * @param   [IN]buffer: 要发送的数据包的内容
 * @param	[IN]size: 要发送的数据包的长度
 * @param	[OUT]list_head: 指向下行信息的链表头的指针，指向的链表中只有包含业务数据或在开启模块的STATUS指令是不包含业务数据的最后一条信息
 * @return  本次通信的状态
 * @retval		00000001B -01- COMMUNICATION_SUCCESSS：通信成功
 * @retval		00000010B -02- NO_JOINED：模块未入网
 * @retval		00000100B -04- COMMUNICATION_FAILURE：确认帧未收到ACK
 * @retval		00001000B -08- NODE_BUSY：模块当前处于忙状态
 * @retval		00010000B -16- NODE_EXCEPTION：模块处于异常状态
 * @retval		00100000B -32- NODE_NO_RESPONSE：模块串口无响应
 * @retval		01000000B -64- PACKET_LENGTH_ERROR：数据包长度错误
 *
 * @par 示例:
 * 下面显示如何调用该API，发送数据
 *
 * @code
 * uint8_t test_data[100];
 * down_list_t *head = NULL;
 * execution_status_t send_result;
 *
 * //发送51字节确认帧数据，重发次数为4次
 * send_result = node_block_send(CONFIRM_TYPE | 0x04, test_data, 51, &head);
 * DEBUG_PRINTF("Send result: %02x \n", send_result);
 * @endcode
 */
execution_status_t node_block_send(uint8_t frame_type, uint8_t *buffer, uint8_t size, down_list_t **list_head)
{
	static uint8_t max_timeout = 0;
	static uint8_t last_frame_tpye = 0;
	static char comfirm_cmd[15] = {0};
	static uint8_t abnormal_count = 0;
	
	/* 若连续多次发生，发数据前模块一直忙或模块串口无响应，则复位模块并重新入网*/
	if(abnormal_count > ABNORMAL_CONTINUOUS_COUNT_MAX)
	{
		abnormal_count = 0;
		node_reset_block_join(300);
	}
	
	free_down_info_list(list_head, all_list);

	// 1.发送数据前先判断模块是否入网
	if(false == node_join_successfully)
	{
		return NO_JOINED; 
	}
	// 2.判断传入的数据长度是否正确 
	if(0 == size)
	{
		return PACKET_LENGTH_ERROR; 
	}
	
	// 3.拉高或保持WAKE脚为高
	node_gpio_set(wake, wakeup);
	
	// 4.判断本次要发送的帧类型是否有更改
	if(frame_type != last_frame_tpye || 1 == node_reset_signal.Bits.frame_type_modify)
	{
		node_reset_signal.Bits.frame_type_modify = 0;
		max_timeout = frame_type >> 4 == 0x01 ? (frame_type & 0x0f) * 9 : (frame_type & 0x0f) * 7;	
		max_timeout = max_timeout == 0 ? 10 : max_timeout;
		sprintf(comfirm_cmd, "AT+CONFIRM=%d,%d", frame_type >> 4, frame_type & 0x0f);
		transfer_configure_command(comfirm_cmd);
	}
	
	last_frame_tpye = frame_type;
	
	// 5.切到透传模式
	node_gpio_set(mode, transparent);
	
	// 6.发送数据前判断模块BUSY状态，若为忙则等待TIMEOUT后返回
	timeout_start_flag = true;
	while(low == node_gpio_read(busy))
	{
		if(true == time_out_break_ms(max_timeout * 1000))
		{
			abnormal_count++;
			return NODE_BUSY;
		}
	}
	// 7.通过串口向模块发送数据
	transfer_node_data(buffer, size);
	// 8.判断模块BUSY是否拉低，若1s内未拉低，则模块串口异常
	timeout_start_flag = true;
	while(high == node_gpio_read(busy))
	{
		if(true == time_out_break_ms(1000))
		{
			abnormal_count++;
			return NODE_NO_RESPONSE;
		}
	}
	
	abnormal_count = 0;
	
	// 9.等待BUSY拉高
	do
	{
		timeout_start_flag = true;
		while(low == node_gpio_read(busy))
		{
			if(true == time_out_break_ms(max_timeout * 1000))
			{
				return NODE_EXCEPTION;
			}
		}
		
		timeout_start_flag = true;
		while(0 == UART_RECEIVE_FLAG)
		{
			if(true == time_out_break_ms(300))
			{
				break;
			}
		}
		
		if(1 == UART_RECEIVE_FLAG && UART_RECEIVE_LENGTH > 0)
		{
			UART_RECEIVE_FLAG = 0;
			//DEBUG_PRINTF("%s\r\n",UART_RECEIVE_BUFFER);

#ifdef DEBUG_LOG_LEVEL_1
			uint8_t tmp = 0;
			DEBUG_PRINTF("[RECEIVE] ");
			for(tmp = 0; tmp < UART_RECEIVE_LENGTH; tmp++)
			{
				DEBUG_PRINTF("%02x ", UART_RECEIVE_BUFFER[tmp]);	
			}
			DEBUG_PRINTF("\r\n");
#endif
			down_data_process(list_head);
		}
		else
		{
			UART_RECEIVE_LENGTH = 0;
		}
		
		if(31 == confirm_continue_failure_count)
		{
			if(low == node_gpio_read(busy) && UART_RECEIVE_LENGTH < 10)
			{
				// 判断BUSY脚为低后查询模块当前已否已经处于重新注册状态
				if(1 == handle_cmd_return_data("AT+JOIN?", 1, 1))
				{
					confirm_continue_failure_count = handle_cmd_return_data("AT+STATUS0?", 37, 2);
				}
				else
				{
					// 确认模块已发起重新注册，则清除之前的入网状态标致和确认帧连续失败计数
					node_join_successfully = false;
					confirm_continue_failure_count = 0;
#ifdef DEBUG_LOG_LEVEL_1
					DEBUG_PRINTF("Start Rejoin...\r\n");
#endif					
					node_join_successfully = node_block_join(300);
					
					return COMMUNICATION_FAILURE;
				}
			}
		}
	}while(low == node_gpio_read(busy));

	// 11.通过STAT引脚判断当前通信状态
	if(low == node_gpio_read(stat))
	{
		// 若该包为确认帧且未收到ACK，则确认帧连续失败计数加一
		if(0x01 == frame_type >> 4)
		{
			confirm_continue_failure_count += 1;
		}
		return COMMUNICATION_FAILURE;
	}
	else
	{
		ledStat = BLINK;
		confirm_continue_failure_count = 0;
		return COMMUNICATION_SUCCESSS;
	}
}

/**
 * @brief   带模块休眠的发送函数
 * @details 以阻塞的形式发送数据，发送前唤醒模块，发送完成后休眠模块
 * @param	[IN]frame_type: 十六进制，表示本次发送的帧类型和发送次数\n
						高四位：要发送的帧为确认帧(0001)还是非确认(0000),\n
						低四位：若确认帧，则表示未收到ACK的情况下共发送的次数；若为非确认，则表示总共需发送的次数
 * @param   [IN]buffer: 要发送的数据包的内容
 * @param	[IN]size: 要发送的数据包的长度
 * @param	[OUT]list_head: 指向下行信息的链表头的指针，指向的链表中只有包含业务数据或在开启模块的STATUS指令是不包含业务数据的最后一条信?
 * @return  本次通信的状态
 * @retval		00000001B -01- COMMUNICATION_SUCCESSS：通信成功
 * @retval		00000010B -02- NO_JOINED：模块未入网
 * @retval		00000100B -04- COMMUNICATION_FAILURE：确认帧未收到ACK
 * @retval		00001000B -08- NODE_BUSY：模块当前处于忙状态
 * @retval		00010000B -16- NODE_EXCEPTION：模块处于异常状态
 * @retval		00100000B -32- NODE_NO_RESPONSE：模块串口无响应
 * @retval		01000000B -64- PACKET_LENGTH_ERROR：数据包长度错误
 *
 * @par 示例:
 * 下面显示如何调用该API，发送数据
 *
 * @code
 * uint8_t test_data[100];
 * down_list_t *head = NULL;
 * execution_status_t send_result;
 *
 * //发送51字节确认帧数据，重发次数为3次
 * send_result = node_block_send_lowpower(CONFIRM_TYPE | 0x03, test_data, 51, &head);
 * DEBUG_PRINTF("Send result: %02x \n", send_result);
 * @endcode
 */
execution_status_t node_block_send_lowpower(uint8_t frame_type, uint8_t *buffer, uint8_t size, down_list_t **list_head)
{
	execution_status_t status_result;

	// 1.调用node_block_send函数发送数据并得到返回
	status_result = node_block_send(frame_type, buffer, size, list_head);
	// 2.发送数据完成后休眠模块
	node_gpio_set(wake, sleep);
		
	return status_result;
}

/**
 * @brief   大数据包处理发送
 * @details 将大数据包根据不同的速率拆分成多个小数据包分别发送(最大支持拆分成32个小包)，使用该函数时，AS（应用服务器）需实现本驱动定义的分包协议，该函数已实现终端的分包协议
 * 分包协议如下：\n
  1. 上行包结构（终端到应用服务器）
 * 	  - 第一个字节为包头，固定为0xAA
 *	  - 第二个字节为大数据包的序号
 *	  - 第三个字节第7bit为应答请求位（1表示请求应答，0表示AS无需应答），第5~6bit保留，第0~4bit为分包的包序号
 *	  - 最后一个字节为包尾，固定为0xBB 
 *	  - 其余字节为用户数据
  2. 下行包结构（应用服务器到终端）
 *	   - 第一个字节为包头，固定为0xBB
 *	   - 第二个字节表示当前大数据包的序号
 *	   - 第三到六个字节表示应用服务器的接收该大数据包各分包的结果。按位表示，小端模式，四个字节共32位, 如接收到的分包序号为：0、1、2、5、6、7、8、10，则第三到六个字节依次为：0xE7 0x05 0x00 0x00
 *	   - 最后一个字节为包尾，固定为0xAA
  3. 终端分包逻辑
 *	   - 3.1 关闭ADR，终端根据当前模块的速率将大数据包拆分成若干个分包，并加个四个分包协议字节，以非确认帧发送出去
 *	   - 3.2 发送最后一个分包时将该包应答请求位置1，并等待AS应答。若未收到AS的应答，则重发最后一包，若重发到设定的次数后均未收到AS的正确应答，则返回大数据包发送失败
 *	   - 3.3 终端根据AS应答的内容解析出AS未收到哪些分包，然后重新发送这些未被AS接收的分包，并将这些分包中的最后一包标记为结束包，并执行3.2的内容
 *	   - 3.4 当AS应答的内容为成功接收了所有的分包后，结束大数据包发送流程，返回结果
  4. 应用服务器(AS)组包及应答逻辑
 *	   - 4.1 AS每接收一包数据后根据包头和包尾判断该包是否为大数据包的分包
 *	   - 4.2 若该包为大数据包的分包，并判断该包大数据包的序号与上一分包是否相同，如果不同，则将“接收标致变量”（uint32）清零
 *	   - 4.3 根据该分包序号将“接收标致变量”对应的位置1。判断该分包是否需要应答，若无需应答则将该包暂存起来
 *	   - 4.4 若4.3中判断出该分包需应答，则将“接收标致变量” 按下行包结构要求组包并发送给终端，并根据当前大数据包序号下的各分包序号判断是否已接收所有分包
 *		 	（最大分包序号：同一大数据包序号中，所有应答请求位为1的分包中分包序号最大的那包）
 *	   - 4.5 若4.4中判断为已全部接收，则按当前大数据包序号下的分包序号对用户数据进行组包（可能会有重复的分包，需过去除）。完成一个大数据包的接收流程
 * 注：发送大数据包时默认使用非确认帧，以减小对空口资源的占用
 *
 * @param	[IN]buffer: 要发送的数据包的内容
 * @param	[IN]size: 要发送的数据包的长度
 * @param	[IN]resend_num: 最后一个小数据包在未收到应用服务器（AS）特定应答的情况下，最后一个小数据包的重发次数
 * @param	[OUT]list_head: 指向下行信息的链表头的指针，指向的链表中只有包含业务数据或不包含业务数据的最后一条信息
 * @return  本次大数据包发送是否成功
 * @retval		true: 大数据包发送完全成功
 * @retval		false: 大数据包发送失败
 *
 * @par 示例:
 * 下面显示如何调用该API，发送大数据包
 *
 * @code
 * uint8_t test_data[810];
 * down_list_t *head = NULL;
 *
 * //发送800字节的大数据包，最后一个分包在未收到协议回复的情况下重发8次
 * if(node_block_send_big_packet(test_data, 800, 8, &head) == true)
 * {
 *   // 发送数据成功
 * }
 * // 发送数据失败
 * @endcode
 */
bool node_block_send_big_packet(uint8_t *buffer, uint16_t size, uint8_t resend_num, down_list_t **list_head)
{
	uint8_t datarate = 0;
	uint8_t max_data_len[6] = {47, 47, 47, 111, 218, 218};  /* X - 4 example: SF7 222 - 4 = 218 */
	uint8_t sub_full_size = 0;
	uint8_t sub_last_size = 0;
	uint8_t i = 0, j = 0;
	uint8_t sended_count = 0;
	uint16_t packet_start_bit[32] = {0};
	uint8_t packet_length[32] = {0};
	uint8_t spilt_packet_data[222] = {0};
	uint8_t sub_fcnt = 0;
	uint8_t sub_fcnt_end = 0;
	uint32_t communication_result_mask = 0;
	uint32_t full_success_mask = 0;
	static uint8_t packet_mark = 0;
	bool first_point = true;
	
	down_list_t *single_list = NULL;
	down_list_t *single_list_head = NULL;
	down_list_t	*tmp_storage = NULL;
	down_list_t	*last_packet_list = NULL;
	down_list_t	*last_packet_list_head = NULL;
	
	free_down_info_list(list_head, all_list);
	
	node_gpio_set(wake, wakeup);

	transfer_configure_command("AT+ADR=0");
	
	if(-1 == last_up_datarate)
	{
		datarate = handle_cmd_return_data("AT+DATARATE?", 1, 1);
	}
	else
	{
		datarate = last_up_datarate;
	}
	// 根据查回的速率通过查表得知当前能发送用户数据的最大长度
	sub_full_size = max_data_len[datarate]; /*表示模块返回的当前速率*/
	sub_last_size = size % sub_full_size;
	// 每发送一个大数据包，大包序号加一
	packet_mark++;
	
	// 计算每个分包在大包的起始位置和分包的长度
	for(sub_fcnt = 0; sub_fcnt < size / sub_full_size + 1; sub_fcnt++, i = 0)
	{	
		packet_start_bit[sub_fcnt] = sub_fcnt * sub_full_size;
		
		if(sub_fcnt == size / sub_full_size)
		{
			packet_length[sub_fcnt] = sub_last_size + 4;
		}
		else
		{
			packet_length[sub_fcnt] = sub_full_size + 4;
		}
	}
	
	sub_fcnt--;
	
	// 计算所有分包均被服务器成功接收后的掩码
	for(i = 0; i <= sub_fcnt; i++)
	{
		full_success_mask |= (0x01 << sub_fcnt) >> i;
	}
	
	while(full_success_mask != (communication_result_mask & full_success_mask))
	{
		// 若对丢失包的补发次数大于分包的个数，则可能是应用服务器处大数据处理发生异常
		if(sended_count > sub_fcnt)
		{
			node_gpio_set(wake, sleep);
			return false;
		}
		// 查找最后一包的分包序号
		for(j = 0; j <= sub_fcnt; j++)
		{
			if(0 == ((communication_result_mask >> j) & 0x01))
			{
				sub_fcnt_end = j;
			}
		}
		// 按照分包协议对用户数据拆分并组包发送
		for(j = 0; j <= sub_fcnt; j++)
		{
			if(0 == ((communication_result_mask >> j) & 0x01))
			{
				// 分包的包头
				spilt_packet_data[0] = 0xAA;
				// 大数据包的序号
				spilt_packet_data[1] = packet_mark;
				// 该字节第0~4bit为分包的包序号，第5~6bit保留，第7bit为应答请求位
				spilt_packet_data[2] = j & 0x1f;
				// 中间为用户数据
				for(i = 0; i < packet_length[j] - 4; i++)
				{
					spilt_packet_data[i + 3] = *(buffer + packet_start_bit[j] + i);
				}
				// 分包的包尾
				spilt_packet_data[i + 3] = 0xBB;
				
				if(j == sub_fcnt_end)
				{
					// 若当前为所有分包中的最后一包，则第二个字节的最高位置，通知应用服务器回终端(AS成功收到了哪些分包)
					spilt_packet_data[2] |= 0x80;
				}
				else
				{
#ifdef DEBUG_LOG_LEVEL_1
					uint8_t tmp = 0;
					DEBUG_PRINTF("[SEND] ");
					for(tmp = 0; tmp < packet_length[j]; tmp++)
					{
						DEBUG_PRINTF("%02x ", spilt_packet_data[tmp]);
					}
					DEBUG_PRINTF("\r\n");	
#endif
					
					single_list = NULL;
					node_block_send(UNCONFIRM_TYPE | 0x01, spilt_packet_data, packet_length[j], &single_list);
					
					// 以下代码段为使用链表对接收到的模块串口数据做相应的处理，只保留有业务数据的部分
					single_list_head = single_list;
					
					while(NULL != single_list)
					{
						if(single_list->down_info.size > 0)
						{
							if(true == first_point)
							{
								first_point = false;
								tmp_storage = single_list;
								*list_head = tmp_storage;								
							}
							else
							{
								tmp_storage->next = single_list;
								tmp_storage = tmp_storage->next;
							}
						}
						single_list = single_list->next;
					}
					free_down_info_list(&single_list_head, save_data);
				}
			}
		}
		
		// 发送最后一个分包，并等待AS应答
		last_packet_list = NULL;
		communication_result_mask = wait_as_respone(spilt_packet_data, 
													packet_length[sub_fcnt_end], 
													packet_mark, 
													resend_num, 
													&last_packet_list);

		// 以下代码段为使用链表，对接收到的模块串口数据做相应的处理，只保留有业务数据的部分或所有分包中最后一包的下行信息
		last_packet_list_head = last_packet_list;

		if(full_success_mask == (communication_result_mask & full_success_mask) || 
		   0 == communication_result_mask || sended_count++ == sub_fcnt)
		{
			while(NULL != last_packet_list)
			{
				if(true == first_point)
				{
					first_point = false;
					tmp_storage = last_packet_list;
					*list_head = tmp_storage;	
				}
				else
				{
					tmp_storage->next = last_packet_list;
					tmp_storage = tmp_storage->next;
				}

				last_packet_list = last_packet_list->next;
			}

			free_down_info_list(&last_packet_list_head, save_data_and_last);			
		}
		else
		{
			while(NULL != last_packet_list)
			{
				if(last_packet_list->down_info.size > 0)
				{
					if(true == first_point)
					{
						first_point = false;
						tmp_storage = last_packet_list;
						*list_head = tmp_storage;								
					}
					else
					{
						tmp_storage->next = last_packet_list;
						tmp_storage = tmp_storage->next;
					}
				}
		
				last_packet_list = last_packet_list->next;
			}
			
			free_down_info_list(&last_packet_list_head, save_data);
		}
		
		if(0 == communication_result_mask)
		{
#ifdef DEBUG_LOG_LEVEL_1
			DEBUG_PRINTF("Did not receive a reply from AS\r\n");
#endif
			node_gpio_set(wake, sleep);
			// 若AS未应答终端或错误应答，则本次大数据包通信失败
			return false;
		}
#ifdef DEBUG_LOG_LEVEL_1
		DEBUG_PRINTF("full_success_mask: 0x%08x, communication_result_mask: 0x%08x\r\n", full_success_mask, communication_result_mask);
#endif
	}
	
	transfer_configure_command("AT+ADR=1");
	node_gpio_set(wake, sleep);
	
	return true;
}

/**
  * @}
  */ 


/*! @} defgroup ICA-Driver */
