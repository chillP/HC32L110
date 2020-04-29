#ifndef _LSD_LORAWAN_ICA_DRIVER__H_
#define _LSD_LORAWAN_ICA_DRIVER__H_



#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

/*-------------------------����ʵ�������ӱ�Ҫ��ͷ�ļ�-------------------------*/
#include "driver_mid.h"
#include "gpio.h"

/*=====================================END======================================*/

/*-------------���ݾ����Ӳ��ƽ̨����д���º궨���в����ľ���ʵ��---------------*/

extern uint8_t uart0_RxBuf[UART0_RXBUFSIZE];
extern uint8_t uart1_RxBuf[UART1_RXBUFSIZE];
extern uint8_t lpuart_RxBuf[LPUART_RXBUFSIZE];

extern int uart0_RxByteCnt,uart0_TxByteCnt;
extern int uart1_RxByteCnt,uart1_TxByteCnt;
extern int lpuart_RxByteCnt,lpuart_TxByteCnt;

extern volatile uint32_t RxDoneFlag_lpuart;
extern volatile uint32_t RxDoneFlag_uart0;
extern volatile uint32_t RxDoneFlag_uart1;

/** ����ʵ��MCU�Ĵ�ӡ�����滻�궨�� */
#define DEBUG_PRINTF		printf

/** ��ȡ����ģ��STAT���ŵĵ�ƽ */
#define GET_STAT_LEVEL		Gpio_GetIO(1, 5)

/** ��ȡ����ģ��BUSY���ŵĵ�ƽ */
#define GET_BUSY_LEVEL		Gpio_GetIO(2, 3)

/** ��������ģ��RESET���ŵ�Ϊ�ߵ�ƽ */
#define SET_RESET_HIGH		Gpio_SetIO(2, 4, TRUE);

/** ��������ģ��RESET���ŵ�Ϊ�͵�ƽ */
#define SET_RESET_LOW		Gpio_SetIO(2, 4, FALSE);

/** ��������ģ��MODE���ŵ�Ϊ�ߵ�ƽ */
#define SET_MODE_HIGH		Gpio_SetIO(1, 4, TRUE);

/** ��������ģ��MODE���ŵ�Ϊ�͵�ƽ */
#define SET_MODE_LOW		Gpio_SetIO(1, 4, FALSE);

/** ��������ģ��WAKE���ŵ�Ϊ�ߵ�ƽ */
#define SET_WAKE_HIGH		Gpio_SetIO(3, 4, TRUE);

/** ��������ģ��WAKE���ŵ�Ϊ�͵�ƽ */
#define SET_WAKE_LOW		Gpio_SetIO(3, 4, FALSE);

/** ��ȡ��ǰϵͳ��ʱ�䣬��λΪms */
#define GET_SYSTEM_TIME		getTick()

/** ��ģ�������ӵĴ��ڷ��ͺ�������������ָ���ĳ��ȷ��ʹ������� */
#define UART_WRITE_DATA(buffer, size)		Lpuart_send_data(buffer, size)

/** ��ģ�������ӵĴ��ڷ��ͺ�������ָ�����ȣ���\0���� */
#define UART_WRITE_STRING(buffer)			Lpuart_send_string(buffer)

/** ��ģ�������ӵĴ��ڽ��ջ��壬Ŀǰ���ڽ������ݽ�֧���жϼ�BUFFER����ʽ����֧��FIFO��ʽ */
#define UART_RECEIVE_BUFFER					lpuart_RxBuf

/** ��ģ�������ӵĴ��ڽ��յ����ݵı��� */
#define UART_RECEIVE_FLAG					RxDoneFlag_lpuart

/** ��ģ�������ӵĴ��ڱ��ν��յ����ݳ��� */
#define UART_RECEIVE_LENGTH					lpuart_RxByteCnt
/*=====================================END======================================*/

/** �û����ݲ�Ʒ��Ҫ�������º궨�������в���ATָ�Ĭ��ΪNULL�����˺궨�������ӵ�ָ����ᱻ���浽ģ���С�
 * @par ʾ��:
 * ������������ɨ��Ƶ��ʽΪ����ģ����Ƶ��1A2�������¸�д�궨�壺
 * @code
	#define AT_COMMAND_AND_SAVE		"AT+BAND=47,0",  \
									"AT+CHMASK=00FF" \
 * @endcode
*/
//004A7700660014D1
#define AT_COMMAND_AND_SAVE			"AT+DEVEUI=004A770066FFFFFF,D391010220102816,1",  \
									"AT+APPEUI=2C26C50066000002",  \
									"AT+APPKEY=00112233445566778899AABBCCDDEEFF",  \
									"AT+MINIRF=0",  \
									"AT+BAND=3,0",  \
									"AT+OTAA=1,1",  \
									"AT+CLASS=2",  \
									"AT+DEBUG=0" \


/** �û����ݲ�Ʒ��Ҫ�������º궨�����������ϣ��ģ��ÿ�θ�λ���ִ�е�ָ�Ĭ��ΪNULL����ÿ��ָ����","������*/
#define AT_COMMAND_NOT_SAVE			NULL

/** ģ�������쳣�������ֵ��������ֵʱ��������λģ�� */
#define ABNORMAL_CONTINUOUS_COUNT_MAX			6


/* -------------------------------------------------------------------------------
 * 					!!!!���������û�����ƽ̨����Ҫ�����޸�!!!!
 *********************************************************************************
 *						!!!!�������£��������޸�!!!!
 --------------------------------------------------------------------------------*/


/*--------------------��־����ȼ���δβ����Խ�󣬵ȼ�Խ��----------------------*/
#ifdef DEBUG_LOG_LEVEL_1
	#define DEBUG_LOG_LEVEL_0
#endif
/*=====================================END======================================*/

/*------------------------------�������ݵ�֡����--------------------------------*/
/** ����λΪ1����ȷ��֡ */
#define CONFIRM_TYPE		0x10

/** ����λΪ0�����ȷ��֡ */
#define UNCONFIRM_TYPE		0x00
/*=====================================END======================================*/

#define SUBTRANCTION_CORSS_ZERO(e, s)   e < s ? e + 4294967296 - s : e - s

/*---------------------------�����ļ��汾��Ϣ�����޸�---------------------------*/
#define DRIVER_VERSION_INFO		"LR-Modem_Driver_V1.1.6_1907023_ICA_Release"
/*=====================================END======================================*/

/*-------------------------����״̬�����š�����ֵ��ö��-------------------------*/
/**
 * ģ��ĸ��ֹ���״̬
 */
typedef enum {
	command, 	///< ����ģʽ
	transparent,///< ͸��ģʽ
	wakeup,		///< ����ģ��
	sleep,		///< ����ģ��
} node_status_t;

/**
 * ģ��ĸ��ֹ�������
 */
typedef enum {
	mode,	///< ģʽ����(�л������͸��)
	wake,	///< ��������(�л����ߺͻ���)
	stat,	///< ģ��ͨ��״̬����
	busy,	///< ģ���Ƿ�Ϊæ����
} node_gpio_t;

/**
 * GPIO��ƽ״̬
 */
typedef enum {
	low,	///< �͵�ƽ
	high,	///< �ߵ�ƽ
	unknow,	///< δ֪��ƽ
} gpio_level_t;

/**
 * �ͷ�������Դ�ĵȼ�
 */
typedef enum {
	all_list,	///< �ͷ�����������������Դ
	save_data,	///< ����������data��Ϊ�յ���Դ
	save_data_and_last,  ///< ����������data��Ϊ�յ���Դ�����һ����Դ
} free_level_t;

/**
 * �������ݵķ���״̬
 */
typedef enum {
	COMMUNICATION_SUCCESSS	= 0x01,		///< ͨ�ųɹ�
	NO_JOINED				= 0x02,		///< ģ��δ����
	COMMUNICATION_FAILURE	= 0x04,		///< ȷ��֡δ�յ�ACK
	NODE_BUSY				= 0x08,		///< ģ�鵱ǰ����æ״̬
	NODE_EXCEPTION			= 0x10,		///< ģ�鴦���쳣״̬
	NODE_NO_RESPONSE		= 0x20,		///< ģ�鴮������Ӧ
	PACKET_LENGTH_ERROR		= 0x40		///< ���ݰ����ȴ���
}execution_status_t;

/**
 * �ն�������λģ����źű���
 */
typedef union node_reset_single
{
    uint8_t value;

    struct
    {
		uint8_t frame_type_modify      	 : 1;	///< ��λ��������֡����
		uint8_t RFU          			 : 7;	///< ����λ��ʱ���������Ժ�ʹ��
    }Bits;
}node_reset_single_t;


/**
 * ����ͨ��Ϣ��������Ϣ
 */
typedef struct down_info {
	uint8_t size;		///< ҵ�����ݳ���
	uint8_t *business_data;	///< ҵ�����ݡ�
#ifdef USE_NODE_STATUS
	uint8_t result_ind;	///< ���ֶ����ģ��ʹ��˵����	
	int8_t snr;			///< ����SNR���������У����ֵΪ0
	int8_t rssi;		///< ����RSSI���������У����ֵΪ0
	uint8_t datarate;	///< �������ж�Ӧ����������
#endif
} down_info_t;

/**
 * ������Ϣ������ṹ
 */
typedef struct list {
	down_info_t down_info;   ///< ������Ϣ�ṹ
	struct list *next;
}down_list_t;

/*=====================================END======================================*/

/*------------------------------�ṩ���û��Ľӿں���----------------------------*/
void node_hard_reset_and_configure(void);
bool node_configure(void);
bool hot_start_rejoin(uint16_t time_second);
bool save_cmd_configure(void);
bool unsave_cmd_configure(void);
bool node_block_join(uint16_t time);	
void node_reset_block_join(uint16_t time_second);
execution_status_t node_block_send(uint8_t frame_type, uint8_t *buffer, uint8_t size, down_list_t **list_head);
execution_status_t node_block_send_lowpower(uint8_t frame_tpye, uint8_t *buffer, uint8_t size, down_list_t **list_head);
bool node_block_send_big_packet(uint8_t *buffer, uint16_t size, uint8_t resend_num, down_list_t **list_head);
bool hot_start_rejoin(uint16_t time_second);
bool transfer_configure_command(char *cmd);
bool transfer_inquire_command(char *cmd, uint8_t *result);
bool time_out_break_ms(uint32_t time);
void system_delay_ms(uint32_t delay);
void node_hard_reset(void);
void node_gpio_set(node_gpio_t type, node_status_t value);
uint8_t* find_string(uint8_t *s, uint8_t *d);
/*=====================================END======================================*/

extern uint8_t confirm_continue_failure_count;
extern int8_t last_up_datarate;
extern uint8_t logLevel;
/*--------------------------------�ṩ���û��ı���------------------------------*/
extern bool node_join_successfully;
/*=====================================END======================================*/

#endif
