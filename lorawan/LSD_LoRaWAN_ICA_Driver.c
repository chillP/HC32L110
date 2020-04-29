/*!
 * @file       LSD_LoRaWAN_ICA_Driver.c
 * @brief      LSD_LoRaWAN_ICA_Driver�����lierda LoRaWAN ICAģ�����������
 * @details    ���ļ�Ϊ�û��ṩһ����LoRaWANģ���׼�������������̡���ģ����ָ���,�Ĳ�����װ�ɺ����ӿڣ����û����ã��Ӷ����̲�Ʒ��������
 * @copyright  Revised BSD License, see section LICENSE. \ref LICENSE
 * @author     Lierda-WSN-LoRaWAN-ëͥ
 * @date       2019-07-23
 * @version    V1.1.6
 */

/*!
 * @defgroup  ICA-Driver LoRaWAN_ICA_Node_Driver API
 * @brief     ���ļ�Ϊ�û��ṩһ�����LoRaWAN_ICAģ��Ĵ���������
 * @{
 */   
#include "LSD_LoRaWAN_ICA_Driver.h"

/** @defgroup ICA_Driver_Exported_Variables ICA_Driver_Variables
  * @brief ICA���������ṩ���û��ı���
  * @{
  */
  
/** ���ó�ʱ��ʼ���� */
bool timeout_start_flag = true;
/** ģ���Ƿ��ѳɹ����� */
bool node_join_successfully = false;
/** ȷ��֡����ʧ�ܴ��� */
uint8_t confirm_continue_failure_count = 0;
/** ���һ���������ݵ����� */
int8_t last_up_datarate = -1;
/** �ն˲���λ������¶�ģ�鸴λ���ź��� */
node_reset_single_t node_reset_signal;
//LED״̬
extern ledStatType ledStat;

/**
  * @}
  */ 
  
const char *at_command_array_save[] = {AT_COMMAND_AND_SAVE};
const char *at_command_array_not_save[1] = {AT_COMMAND_NOT_SAVE};
 
bool unsave_cmd_configure(void);
void free_down_info_list(down_list_t **list_head, free_level_t free_level);

/**
*  �����û����ݸ�ģ��
*@param 	buffer: Ҫ���͵���������
*@param		size: ���ݵĳ���
*/
static void transfer_node_data(uint8_t *buffer, uint8_t size)
{	
	UART_RECEIVE_FLAG = 0;
	UART_WRITE_DATA(buffer, size);
}

/**
*  ��ʱ����
*@param 	time: ���õľ��峬ʱʱ��ֵ����λ����
*@return	�Ƿ��趨�ĳ�ʱʱ��
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
*  ��ʱ����
*@param 	delay: ��ʱ��ʱ�䣬��λ����
*/
void system_delay_ms(uint32_t delay) 
{
	uint32_t tickstart = 0U;
	tickstart = GET_SYSTEM_TIME;
	while((GET_SYSTEM_TIME - tickstart) < delay);
}

/**
*  ����ģ������ŵ�ƽ
*@param 	type: ��ʾ���õ���WAKE���Ż���STAT����
*@param 	value: ��ʾ���ö�Ӧ���ŵĸߵ͵�ƽ
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
*  ��ȡģ�����ŵĵ�ƽ
*@param 	gpio: ��ʾҪ��ȡ����STAT���Ż���BUSY����
*@param		d: Ҫ���ҵ��ַ���
*@return	��Ӧ���ŵĵ�ƽ
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
*  ��Сд��ĸתΪ��д��ɾ���ո�
*@param 	[IN]src: ԭʼ�ַ���
*@param 	[OUT]des: Ŀ���ַ
*@return	��
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
*  ����һ���ַ����Ƿ�����һ���ַ����г���
*@param 	s: ԭʼ�ַ���
*@param 	d: Ҫ���ҵ��ַ���
*@return	���ز��ҵĽ��
*@retval		NULL: d�ַ���δ������s�ַ���
*@retval		s:    d�ַ���������s�ַ���,�ҷ��ص�ǰָ��s��ָ��ĵ�ַ
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
*  ��ȡָ����ͷ�ͽ�β���м��ַ�
*@param		str: ԭʼ�ַ���
*@param 	s: ��ʼ�ַ���
*@param		e: ��β�ַ���
*@param		res: ��ȡ�����ַ���
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
*  ʮ�������ַ���ת����
*@param		s: ʮ�������ַ���
*@param 	s: ת���ַ�������
*@return	�������
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
* ��ѯģ���Ƿ����������Լ�������������������Ƿ����������ú��������ڶ�ģ�鸴λ���û�MCU����λ�����
*@return    ��ѯָ���Ƿ�����ִ��
*/
static bool hot_start_check(void)
{
	uint8_t return_data[50] = {0};
	bool result_tmp = true;
	// ģ���״��ϵ��λ�����hot_start_check()�������ʽ����ź�����λ
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
*  ִ��ģ������ָ��Դ�ģʽ�л�
*@param		cmd: ��Ҫִ�е�����ָ��
*@return	ָ��ִ�н��
*@retval		true:  ִ�гɹ�
*@retval		false: ִ��ʧ��
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
		timeouts = 200; //ԭ50 APPKEY������150ms
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
		if(find_string(UART_RECEIVE_BUFFER, (uint8_t *)"\r\n")!=NULL)  //��Ϊָ����Ӧ���У���Ϊ�������������
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
*  ִ��ģ���ѯָ��Դ�ģʽ�л�
*@param		cmd: ��Ҫִ�еĲ�ѯָ��
*@return	result: ��ѯָ��ķ��ؽ��
*@return	ָ��ִ�н��
*@retval		true:  ִ�гɹ�
*@retval		false: ִ��ʧ��
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
* ���н������ݴ���
*@param		list_head: ָ��������Ϣ������ͷ��ָ��
*@return    ��
*/
static void down_data_process(down_list_t **list_head)
{
	down_list_t *p1 = NULL;
	down_list_t *p2 = NULL;
	uint8_t cnt = 0;
	
#ifdef USE_NODE_STATUS		
	// �������������������㣬���ʾ�ⲻ�����һ�����а�������ҵ������
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
				DEBUG_PRINTF ("Error��Dynamic application for memory failure -- p1->down_info.business_data\r\n");
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
			DEBUG_PRINTF ("Error��Dynamic application for memory failure -- p1\r\n");
#endif
		}
#ifdef USE_NODE_STATUS
	}
		last_up_datarate = UART_RECEIVE_BUFFER[UART_RECEIVE_LENGTH - 1] & 0x0f;
#endif				
}


/**
* ������һ���ְ����ȴ�ASӦ�𣨽���node_block_send_big_packet�������ã�
*@param		packet_end: ���һ���ְ��ĵ�ַ
*@param		size: ���һ���ְ��ĳ���
*@param		packet_mark: �����ݰ������
*@param		resend_num: ���һ��С���ݰ���δ�յ�Ӧ�÷�������AS���ض�Ӧ�������£����һ��С���ݰ����ط�����
*@return    AS�����зְ��Ľ��ս��
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
					// �ְ�Э�����ݲ�����ҵ�����ݣ��������Ǵ���е����һ�����У����ݱ�������״ֵ̬
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
* �������ֲ�ѯָ��ķ���ֵ�����������ڲ����ã�
*/
static uint8_t handle_cmd_return_data(char *data, uint8_t start, uint8_t length)
{
	static uint8_t inquire_return_data[150];
	
	memset(inquire_return_data, 0, 150);
	transfer_inquire_command(data, inquire_return_data);

	return htoi((inquire_return_data + start), 2);	
}

/**
* ģ��Ӳ����λ���������û�ֱ��ʹ�øú����������û�ʹ��node_hard_reset_and_configure()��Ӳ��λģ��
*/
void node_hard_reset(void)
{
	// ģ�鸴λǰ��WAKE�������ߣ���Ҫ��
	node_gpio_set(wake, wakeup);
	SET_RESET_LOW;
	system_delay_ms(15);
	SET_RESET_HIGH;
	system_delay_ms(200);	
}


/*------------------------------------------------------------------------------
|>                        ����Ϊ���û��ṩ�ĵ��ú���                          <|
------------------------------------------------------------------------------*/

/** @defgroup ICA_Driver_Exported_Functions ICA_Driver_API_Interface
  * @brief ICAģ���������û�API�ӿ�,��Ҫ��������ģ����и�λ��AT�������á��������������ݵ�
  * @{
  */

/**
 * @brief   Ӳ����λ
 * @details ͨ��Reset���Ÿ�λģ��.
 * @param   ��.
 * @return  ��
 */
void node_hard_reset_and_configure(void)
{
	node_hard_reset();
	hot_start_check();
}

/**
 * @brief   ����ģ�鲻�����ָ��
 * @details ����һЩģ����粻�����ָ��ú��������ڶ�ģ�鸴λ���û�MCU����λ�����
 * @param   ��
 * @return  ָ�����ý��
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
 * @brief   ����ģ�鱣���ָ��
 * @details �ú���Ϊ�����Щ�����ָ����óɹ����ٴ��ϵ��λ���ٵ��øú������û��ڴκ��������һЩ��Ҫ��ָ��
 * @param   ��.
 * @return  ָ��ִ�н��
 * @retval		true:  ����ָ�����ȷִ��
 * @retval		false: һ�������ָ��ִ�г����쳣
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
	/* ʹ�ñ�����ʱ��������ģ��STATUS״̬������ܣ���������ָ���ɾ�� */
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
 * @brief   ģ���ϵ��ʼ
 * @details �û�MCU�ϵ�ʱ��ģ������һЩ�����ָ���Щָ�����û�������Ӧ����Ҫ���䣩
 * @param   ��.
 * @return	ָ��ִ�н��
 * @retval		true:  ����ָ�����ȷִ��
 * @retval		false: һ�������ָ��ִ�г����쳣
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
	
	
	/*�ú���Ϊһ���Ե��ã����óɹ����ٴ��ϵ��λ���ٵ��øú���������FLASH�б���һ�����������жϸú�����û�б����ù�(�û�����ʵ��)*/
	result_tmp &= save_cmd_configure();
	
	
	/*�ú���Ϊÿ���ϵ��λ�������*/
	result_tmp &= hot_start_check();
	
	return result_tmp;
}

/**
 * @brief   ����ɨ��
 * @details ����������ʽ��������ɨ��
 * @param   [IN]time_second: ������ʱʱ�䣬��λ����
 * @return  ���������Ľ��
 * @retval		true:  �����ɹ�
 * @retval		false: ����ʧ��
 *
 * @par ʾ��:
 * ������ʾ��ε��ø�API����������ɨ��
 * @code
 * // ��λ�ڵ㲢����ɨ�裬��ʱʱ������Ϊ300�루��ʱʱ���ƽ�⹦���������ɹ����趨��
 * if(node_block_join(300) == true)
 * {
 *   // �ѳɹ�����
 * }
 * else
 * {
 *	 // ����ʧ��
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

	// 1.�л���͸��ģʽ��ʼ����
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
		
		// 2.ѭ����ѯSTAT��BUSY���ŵ�ƽ
		stat_level = node_gpio_read(stat);
		busy_level = node_gpio_read(busy);
		
		// 3.�����趨ʱ�����δ������ʱ����
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
	// 5.��δ��ʱ���������ģ�������ɹ�
	system_delay_ms(200);  //�ȴ�join��Ϣ��ӡ���
	return true;
}

/**
 * @brief   ��λ�ڵ㲢����ɨ��
 * @details ��λ�ڵ㲢����������ʽ��������ɨ��
 * @param   [IN]time_second: ������ʱʱ�䣬��λ����
 * @return  ���������Ľ��
 * @retval		true:  �����ɹ�
 * @retval		false: ����ʧ��
 *
 * @par ʾ��:
 * ������ʾ��ε��ø�API�����и�λ�ڵ㲢����ɨ��
 * @code
 * // ��λ�ڵ㲢����ɨ�裬��ʱʱ������Ϊ300�루��ʱʱ���ƽ�⹦���������ɹ����趨��
 * node_reset_block_join(300);
 * if(true == node_join_successfully)
 * {
 *   // �ѳɹ�����
 * }
 * else
 * {
 *	 // ����ʧ��
 * }
 * @endcode
 */
void node_reset_block_join(uint16_t time_second)
{
	node_join_successfully = false;
	
	// Ӳ����λģ��, ������һЩ���粻�����ָ��
	node_hard_reset_and_configure();
	
	if(false == node_join_successfully)
	{
		node_join_successfully = node_block_join(time_second);
	}

	node_gpio_set(wake, sleep);
}

/**
 * @brief   ����ģ��������������������
 * @details ģ�鴦����������������ʱ������ģ��������������ͨ��node_join_successfully��ֵ�ж��Ƿ�ɹ�����
 * @param   [IN]time_second: ������ʱʱ�䣬��λ����
 * @return  ���������Ľ��
 * @retval		true:  �����ɹ�
 * @retval		false: ����ʧ��
 *
 * @par ʾ��:
 * ������ʾ��ε��ø�API����������������������
 * @code
 *
 * if(hot_start_rejoin(300) == true && node_join_successfully == true)
 * {
 *   // ����������
 * }
 * else
 * {
 *	 // ��������ʧ��
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
 * @brief   �ͷ�������Ϣ��������Դ
 * @details �ͷ�������Ϣ��������Դ ��node_block_send()��node_block_send_big_packet()���ѵ��øú����ͷ���Ӧ����Դ
 * @param   [IN]list_head: �����ͷ
 * @param   [IN]free_level: �ͷŵȼ�
 *			- all_list����˳��ȫ���ͷ�
 *			- save_data������������data��Ϊ�յ���Դ
 *			- save_data_and_last������������data��Ϊ�յ���Դ�����һ����Դ
 * @return  ��
 *
 * @par ʾ��:
 * ������ʾ��ε��ø�API���ͷ�������Դ
 *
 * @code
 * //�ͷ�list_head��ָ���������Դ
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
 * @brief   ����������ʽ��������
 * @details 
 * @param	[IN]frame_type: ʮ�����ƣ���ʾ���η��͵�֡���ͺͷ��ʹ���\n
						����λ��Ҫ���͵�֡Ϊȷ��֡(0001)���Ƿ�ȷ��(0000),\n
						����λ����ȷ��֡�����ʾδ�յ�ACK������¹����͵Ĵ�������Ϊ��ȷ�ϣ����ʾ�ܹ��跢�͵Ĵ���
 * @param   [IN]buffer: Ҫ���͵����ݰ�������
 * @param	[IN]size: Ҫ���͵����ݰ��ĳ���
 * @param	[OUT]list_head: ָ��������Ϣ������ͷ��ָ�룬ָ���������ֻ�а���ҵ�����ݻ��ڿ���ģ���STATUSָ���ǲ�����ҵ�����ݵ����һ����Ϣ
 * @return  ����ͨ�ŵ�״̬
 * @retval		00000001B -01- COMMUNICATION_SUCCESSS��ͨ�ųɹ�
 * @retval		00000010B -02- NO_JOINED��ģ��δ����
 * @retval		00000100B -04- COMMUNICATION_FAILURE��ȷ��֡δ�յ�ACK
 * @retval		00001000B -08- NODE_BUSY��ģ�鵱ǰ����æ״̬
 * @retval		00010000B -16- NODE_EXCEPTION��ģ�鴦���쳣״̬
 * @retval		00100000B -32- NODE_NO_RESPONSE��ģ�鴮������Ӧ
 * @retval		01000000B -64- PACKET_LENGTH_ERROR�����ݰ����ȴ���
 *
 * @par ʾ��:
 * ������ʾ��ε��ø�API����������
 *
 * @code
 * uint8_t test_data[100];
 * down_list_t *head = NULL;
 * execution_status_t send_result;
 *
 * //����51�ֽ�ȷ��֡���ݣ��ط�����Ϊ4��
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
	
	/* ��������η�����������ǰģ��һֱæ��ģ�鴮������Ӧ����λģ�鲢��������*/
	if(abnormal_count > ABNORMAL_CONTINUOUS_COUNT_MAX)
	{
		abnormal_count = 0;
		node_reset_block_join(300);
	}
	
	free_down_info_list(list_head, all_list);

	// 1.��������ǰ���ж�ģ���Ƿ�����
	if(false == node_join_successfully)
	{
		return NO_JOINED; 
	}
	// 2.�жϴ�������ݳ����Ƿ���ȷ 
	if(0 == size)
	{
		return PACKET_LENGTH_ERROR; 
	}
	
	// 3.���߻򱣳�WAKE��Ϊ��
	node_gpio_set(wake, wakeup);
	
	// 4.�жϱ���Ҫ���͵�֡�����Ƿ��и���
	if(frame_type != last_frame_tpye || 1 == node_reset_signal.Bits.frame_type_modify)
	{
		node_reset_signal.Bits.frame_type_modify = 0;
		max_timeout = frame_type >> 4 == 0x01 ? (frame_type & 0x0f) * 9 : (frame_type & 0x0f) * 7;	
		max_timeout = max_timeout == 0 ? 10 : max_timeout;
		sprintf(comfirm_cmd, "AT+CONFIRM=%d,%d", frame_type >> 4, frame_type & 0x0f);
		transfer_configure_command(comfirm_cmd);
	}
	
	last_frame_tpye = frame_type;
	
	// 5.�е�͸��ģʽ
	node_gpio_set(mode, transparent);
	
	// 6.��������ǰ�ж�ģ��BUSY״̬����Ϊæ��ȴ�TIMEOUT�󷵻�
	timeout_start_flag = true;
	while(low == node_gpio_read(busy))
	{
		if(true == time_out_break_ms(max_timeout * 1000))
		{
			abnormal_count++;
			return NODE_BUSY;
		}
	}
	// 7.ͨ��������ģ�鷢������
	transfer_node_data(buffer, size);
	// 8.�ж�ģ��BUSY�Ƿ����ͣ���1s��δ���ͣ���ģ�鴮���쳣
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
	
	// 9.�ȴ�BUSY����
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
				// �ж�BUSY��Ϊ�ͺ��ѯģ�鵱ǰ�ѷ��Ѿ���������ע��״̬
				if(1 == handle_cmd_return_data("AT+JOIN?", 1, 1))
				{
					confirm_continue_failure_count = handle_cmd_return_data("AT+STATUS0?", 37, 2);
				}
				else
				{
					// ȷ��ģ���ѷ�������ע�ᣬ�����֮ǰ������״̬���º�ȷ��֡����ʧ�ܼ���
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

	// 11.ͨ��STAT�����жϵ�ǰͨ��״̬
	if(low == node_gpio_read(stat))
	{
		// ���ð�Ϊȷ��֡��δ�յ�ACK����ȷ��֡����ʧ�ܼ�����һ
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
 * @brief   ��ģ�����ߵķ��ͺ���
 * @details ����������ʽ�������ݣ�����ǰ����ģ�飬������ɺ�����ģ��
 * @param	[IN]frame_type: ʮ�����ƣ���ʾ���η��͵�֡���ͺͷ��ʹ���\n
						����λ��Ҫ���͵�֡Ϊȷ��֡(0001)���Ƿ�ȷ��(0000),\n
						����λ����ȷ��֡�����ʾδ�յ�ACK������¹����͵Ĵ�������Ϊ��ȷ�ϣ����ʾ�ܹ��跢�͵Ĵ���
 * @param   [IN]buffer: Ҫ���͵����ݰ�������
 * @param	[IN]size: Ҫ���͵����ݰ��ĳ���
 * @param	[OUT]list_head: ָ��������Ϣ������ͷ��ָ�룬ָ���������ֻ�а���ҵ�����ݻ��ڿ���ģ���STATUSָ���ǲ�����ҵ�����ݵ����һ����?
 * @return  ����ͨ�ŵ�״̬
 * @retval		00000001B -01- COMMUNICATION_SUCCESSS��ͨ�ųɹ�
 * @retval		00000010B -02- NO_JOINED��ģ��δ����
 * @retval		00000100B -04- COMMUNICATION_FAILURE��ȷ��֡δ�յ�ACK
 * @retval		00001000B -08- NODE_BUSY��ģ�鵱ǰ����æ״̬
 * @retval		00010000B -16- NODE_EXCEPTION��ģ�鴦���쳣״̬
 * @retval		00100000B -32- NODE_NO_RESPONSE��ģ�鴮������Ӧ
 * @retval		01000000B -64- PACKET_LENGTH_ERROR�����ݰ����ȴ���
 *
 * @par ʾ��:
 * ������ʾ��ε��ø�API����������
 *
 * @code
 * uint8_t test_data[100];
 * down_list_t *head = NULL;
 * execution_status_t send_result;
 *
 * //����51�ֽ�ȷ��֡���ݣ��ط�����Ϊ3��
 * send_result = node_block_send_lowpower(CONFIRM_TYPE | 0x03, test_data, 51, &head);
 * DEBUG_PRINTF("Send result: %02x \n", send_result);
 * @endcode
 */
execution_status_t node_block_send_lowpower(uint8_t frame_type, uint8_t *buffer, uint8_t size, down_list_t **list_head)
{
	execution_status_t status_result;

	// 1.����node_block_send�����������ݲ��õ�����
	status_result = node_block_send(frame_type, buffer, size, list_head);
	// 2.����������ɺ�����ģ��
	node_gpio_set(wake, sleep);
		
	return status_result;
}

/**
 * @brief   �����ݰ�������
 * @details �������ݰ����ݲ�ͬ�����ʲ�ֳɶ��С���ݰ��ֱ���(���֧�ֲ�ֳ�32��С��)��ʹ�øú���ʱ��AS��Ӧ�÷���������ʵ�ֱ���������ķְ�Э�飬�ú�����ʵ���ն˵ķְ�Э��
 * �ְ�Э�����£�\n
  1. ���а��ṹ���ն˵�Ӧ�÷�������
 * 	  - ��һ���ֽ�Ϊ��ͷ���̶�Ϊ0xAA
 *	  - �ڶ����ֽ�Ϊ�����ݰ������
 *	  - �������ֽڵ�7bitΪӦ������λ��1��ʾ����Ӧ��0��ʾAS����Ӧ�𣩣���5~6bit��������0~4bitΪ�ְ��İ����
 *	  - ���һ���ֽ�Ϊ��β���̶�Ϊ0xBB 
 *	  - �����ֽ�Ϊ�û�����
  2. ���а��ṹ��Ӧ�÷��������նˣ�
 *	   - ��һ���ֽ�Ϊ��ͷ���̶�Ϊ0xBB
 *	   - �ڶ����ֽڱ�ʾ��ǰ�����ݰ������
 *	   - �����������ֽڱ�ʾӦ�÷������Ľ��ոô����ݰ����ְ��Ľ������λ��ʾ��С��ģʽ���ĸ��ֽڹ�32λ, ����յ��ķְ����Ϊ��0��1��2��5��6��7��8��10��������������ֽ�����Ϊ��0xE7 0x05 0x00 0x00
 *	   - ���һ���ֽ�Ϊ��β���̶�Ϊ0xAA
  3. �ն˷ְ��߼�
 *	   - 3.1 �ر�ADR���ն˸��ݵ�ǰģ������ʽ������ݰ���ֳ����ɸ��ְ������Ӹ��ĸ��ְ�Э���ֽڣ��Է�ȷ��֡���ͳ�ȥ
 *	   - 3.2 �������һ���ְ�ʱ���ð�Ӧ������λ��1�����ȴ�ASӦ����δ�յ�AS��Ӧ�����ط����һ�������ط����趨�Ĵ������δ�յ�AS����ȷӦ���򷵻ش����ݰ�����ʧ��
 *	   - 3.3 �ն˸���ASӦ������ݽ�����ASδ�յ���Щ�ְ���Ȼ�����·�����Щδ��AS���յķְ���������Щ�ְ��е����һ�����Ϊ����������ִ��3.2������
 *	   - 3.4 ��ASӦ�������Ϊ�ɹ����������еķְ��󣬽��������ݰ��������̣����ؽ��
  4. Ӧ�÷�����(AS)�����Ӧ���߼�
 *	   - 4.1 ASÿ����һ�����ݺ���ݰ�ͷ�Ͱ�β�жϸð��Ƿ�Ϊ�����ݰ��ķְ�
 *	   - 4.2 ���ð�Ϊ�����ݰ��ķְ������жϸð������ݰ����������һ�ְ��Ƿ���ͬ�������ͬ���򽫡����ձ��±�������uint32������
 *	   - 4.3 ���ݸ÷ְ���Ž������ձ��±�������Ӧ��λ��1���жϸ÷ְ��Ƿ���ҪӦ��������Ӧ���򽫸ð��ݴ�����
 *	   - 4.4 ��4.3���жϳ��÷ְ���Ӧ���򽫡����ձ��±����� �����а��ṹҪ����������͸��նˣ������ݵ�ǰ�����ݰ�����µĸ��ְ�����ж��Ƿ��ѽ������зְ�
 *		 	�����ְ���ţ�ͬһ�����ݰ�����У�����Ӧ������λΪ1�ķְ��зְ���������ǰ���
 *	   - 4.5 ��4.4���ж�Ϊ��ȫ�����գ��򰴵�ǰ�����ݰ�����µķְ���Ŷ��û����ݽ�����������ܻ����ظ��ķְ������ȥ���������һ�������ݰ��Ľ�������
 * ע�����ʹ����ݰ�ʱĬ��ʹ�÷�ȷ��֡���Լ�С�Կտ���Դ��ռ��
 *
 * @param	[IN]buffer: Ҫ���͵����ݰ�������
 * @param	[IN]size: Ҫ���͵����ݰ��ĳ���
 * @param	[IN]resend_num: ���һ��С���ݰ���δ�յ�Ӧ�÷�������AS���ض�Ӧ�������£����һ��С���ݰ����ط�����
 * @param	[OUT]list_head: ָ��������Ϣ������ͷ��ָ�룬ָ���������ֻ�а���ҵ�����ݻ򲻰���ҵ�����ݵ����һ����Ϣ
 * @return  ���δ����ݰ������Ƿ�ɹ�
 * @retval		true: �����ݰ�������ȫ�ɹ�
 * @retval		false: �����ݰ�����ʧ��
 *
 * @par ʾ��:
 * ������ʾ��ε��ø�API�����ʹ����ݰ�
 *
 * @code
 * uint8_t test_data[810];
 * down_list_t *head = NULL;
 *
 * //����800�ֽڵĴ����ݰ������һ���ְ���δ�յ�Э��ظ���������ط�8��
 * if(node_block_send_big_packet(test_data, 800, 8, &head) == true)
 * {
 *   // �������ݳɹ�
 * }
 * // ��������ʧ��
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
	// ���ݲ�ص�����ͨ������֪��ǰ�ܷ����û����ݵ���󳤶�
	sub_full_size = max_data_len[datarate]; /*��ʾģ�鷵�صĵ�ǰ����*/
	sub_last_size = size % sub_full_size;
	// ÿ����һ�������ݰ��������ż�һ
	packet_mark++;
	
	// ����ÿ���ְ��ڴ������ʼλ�úͷְ��ĳ���
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
	
	// �������зְ������������ɹ����պ������
	for(i = 0; i <= sub_fcnt; i++)
	{
		full_success_mask |= (0x01 << sub_fcnt) >> i;
	}
	
	while(full_success_mask != (communication_result_mask & full_success_mask))
	{
		// ���Զ�ʧ���Ĳ����������ڷְ��ĸ������������Ӧ�÷������������ݴ������쳣
		if(sended_count > sub_fcnt)
		{
			node_gpio_set(wake, sleep);
			return false;
		}
		// �������һ���ķְ����
		for(j = 0; j <= sub_fcnt; j++)
		{
			if(0 == ((communication_result_mask >> j) & 0x01))
			{
				sub_fcnt_end = j;
			}
		}
		// ���շְ�Э����û����ݲ�ֲ��������
		for(j = 0; j <= sub_fcnt; j++)
		{
			if(0 == ((communication_result_mask >> j) & 0x01))
			{
				// �ְ��İ�ͷ
				spilt_packet_data[0] = 0xAA;
				// �����ݰ������
				spilt_packet_data[1] = packet_mark;
				// ���ֽڵ�0~4bitΪ�ְ��İ���ţ���5~6bit��������7bitΪӦ������λ
				spilt_packet_data[2] = j & 0x1f;
				// �м�Ϊ�û�����
				for(i = 0; i < packet_length[j] - 4; i++)
				{
					spilt_packet_data[i + 3] = *(buffer + packet_start_bit[j] + i);
				}
				// �ְ��İ�β
				spilt_packet_data[i + 3] = 0xBB;
				
				if(j == sub_fcnt_end)
				{
					// ����ǰΪ���зְ��е����һ������ڶ����ֽڵ����λ�ã�֪ͨӦ�÷��������ն�(AS�ɹ��յ�����Щ�ְ�)
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
					
					// ���´����Ϊʹ������Խ��յ���ģ�鴮����������Ӧ�Ĵ���ֻ������ҵ�����ݵĲ���
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
		
		// �������һ���ְ������ȴ�ASӦ��
		last_packet_list = NULL;
		communication_result_mask = wait_as_respone(spilt_packet_data, 
													packet_length[sub_fcnt_end], 
													packet_mark, 
													resend_num, 
													&last_packet_list);

		// ���´����Ϊʹ�������Խ��յ���ģ�鴮����������Ӧ�Ĵ���ֻ������ҵ�����ݵĲ��ֻ����зְ������һ����������Ϣ
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
			// ��ASδӦ���ն˻����Ӧ���򱾴δ����ݰ�ͨ��ʧ��
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
