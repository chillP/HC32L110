#ifndef _LSD_LORAWAN_ICA_DRIVER__H_
#define _LSD_LORAWAN_ICA_DRIVER__H_



#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

/*-------------------------根据实际情况添加必要的头文件-------------------------*/
#include "driver_mid.h"
#include "gpio.h"

/*=====================================END======================================*/

/*-------------根据具体的硬件平台，改写以下宏定义中操作的具体实现---------------*/

extern uint8_t uart0_RxBuf[UART0_RXBUFSIZE];
extern uint8_t uart1_RxBuf[UART1_RXBUFSIZE];
extern uint8_t lpuart_RxBuf[LPUART_RXBUFSIZE];

extern int uart0_RxByteCnt,uart0_TxByteCnt;
extern int uart1_RxByteCnt,uart1_TxByteCnt;
extern int lpuart_RxByteCnt,lpuart_TxByteCnt;

extern volatile uint32_t RxDoneFlag_lpuart;
extern volatile uint32_t RxDoneFlag_uart0;
extern volatile uint32_t RxDoneFlag_uart1;

/** 根据实际MCU的打印函数替换宏定义 */
#define DEBUG_PRINTF		printf

/** 读取连接模块STAT引脚的电平 */
#define GET_STAT_LEVEL		Gpio_GetIO(1, 5)

/** 读取连接模块BUSY引脚的电平 */
#define GET_BUSY_LEVEL		Gpio_GetIO(2, 3)

/** 设置连接模块RESET引脚的为高电平 */
#define SET_RESET_HIGH		Gpio_SetIO(2, 4, TRUE);

/** 设置连接模块RESET引脚的为低电平 */
#define SET_RESET_LOW		Gpio_SetIO(2, 4, FALSE);

/** 设置连接模块MODE引脚的为高电平 */
#define SET_MODE_HIGH		Gpio_SetIO(1, 4, TRUE);

/** 设置连接模块MODE引脚的为低电平 */
#define SET_MODE_LOW		Gpio_SetIO(1, 4, FALSE);

/** 设置连接模块WAKE引脚的为高电平 */
#define SET_WAKE_HIGH		Gpio_SetIO(3, 4, TRUE);

/** 设置连接模块WAKE引脚的为低电平 */
#define SET_WAKE_LOW		Gpio_SetIO(3, 4, FALSE);

/** 读取当前系统的时间，单位为ms */
#define GET_SYSTEM_TIME		getTick()

/** 与模块相连接的串口发送函数，按参数中指定的长度发送串口数据 */
#define UART_WRITE_DATA(buffer, size)		Lpuart_send_data(buffer, size)

/** 与模块相连接的串口发送函数，不指定长度，遇\0结束 */
#define UART_WRITE_STRING(buffer)			Lpuart_send_string(buffer)

/** 与模块相连接的串口接收缓冲，目前串口接收数据仅支持中断加BUFFER的形式，不支持FIFO形式 */
#define UART_RECEIVE_BUFFER					lpuart_RxBuf

/** 与模块相连接的串口接收到数据的标致 */
#define UART_RECEIVE_FLAG					RxDoneFlag_lpuart

/** 与模块相连接的串口本次接收到数据长度 */
#define UART_RECEIVE_LENGTH					lpuart_RxByteCnt
/*=====================================END======================================*/

/** 用户根据产品需要，在以下宏定义中自行补充AT指令（默认为NULL），此宏定义中所加的指令均会被保存到模块中。
 * @par 示例:
 * 如需设置入网扫描频方式为：单模、异频、1A2，可如下改写宏定义：
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


/** 用户根据产品需要，在以下宏定义中自行添加希望模块每次复位后均执行的指令（默认为NULL），每条指令以","隔开。*/
#define AT_COMMAND_NOT_SAVE			NULL

/** 模块连续异常计数最大值，超过该值时驱动将复位模块 */
#define ABNORMAL_CONTINUOUS_COUNT_MAX			6


/* -------------------------------------------------------------------------------
 * 					!!!!此线以上用户根据平台及需要自行修改!!!!
 *********************************************************************************
 *						!!!!此线以下，请勿做修改!!!!
 --------------------------------------------------------------------------------*/


/*--------------------日志输出等级，未尾数字越大，等级越高----------------------*/
#ifdef DEBUG_LOG_LEVEL_1
	#define DEBUG_LOG_LEVEL_0
#endif
/*=====================================END======================================*/

/*------------------------------发送数据的帧类型--------------------------------*/
/** 高四位为1代表确认帧 */
#define CONFIRM_TYPE		0x10

/** 高四位为0代表非确认帧 */
#define UNCONFIRM_TYPE		0x00
/*=====================================END======================================*/

#define SUBTRANCTION_CORSS_ZERO(e, s)   e < s ? e + 4294967296 - s : e - s

/*---------------------------驱动文件版本信息，勿修改---------------------------*/
#define DRIVER_VERSION_INFO		"LR-Modem_Driver_V1.1.6_1907023_ICA_Release"
/*=====================================END======================================*/

/*-------------------------各种状态、引脚、返回值的枚举-------------------------*/
/**
 * 模块的各种工作状态
 */
typedef enum {
	command, 	///< 命令模式
	transparent,///< 透传模式
	wakeup,		///< 唤醒模块
	sleep,		///< 休眠模块
} node_status_t;

/**
 * 模块的各种功能引脚
 */
typedef enum {
	mode,	///< 模式引脚(切换命令和透传)
	wake,	///< 唤醒引脚(切换休眠和唤醒)
	stat,	///< 模块通信状态引脚
	busy,	///< 模块是否为忙引脚
} node_gpio_t;

/**
 * GPIO电平状态
 */
typedef enum {
	low,	///< 低电平
	high,	///< 高电平
	unknow,	///< 未知电平
} gpio_level_t;

/**
 * 释放链表资源的等级
 */
typedef enum {
	all_list,	///< 释放所有链表中所有资源
	save_data,	///< 保留链表中data不为空的资源
	save_data_and_last,  ///< 保留链表中data不为空的资源和最后一条资源
} free_level_t;

/**
 * 发送数据的返回状态
 */
typedef enum {
	COMMUNICATION_SUCCESSS	= 0x01,		///< 通信成功
	NO_JOINED				= 0x02,		///< 模块未入网
	COMMUNICATION_FAILURE	= 0x04,		///< 确认帧未收到ACK
	NODE_BUSY				= 0x08,		///< 模块当前处于忙状态
	NODE_EXCEPTION			= 0x10,		///< 模块处于异常状态
	NODE_NO_RESPONSE		= 0x20,		///< 模块串口无响应
	PACKET_LENGTH_ERROR		= 0x40		///< 数据包长度错误
}execution_status_t;

/**
 * 终端主动复位模块的信号标致
 */
typedef union node_reset_single
{
    uint8_t value;

    struct
    {
		uint8_t frame_type_modify      	 : 1;	///< 该位用于设置帧类型
		uint8_t RFU          			 : 7;	///< 其它位暂时保留，供以后使用
    }Bits;
}node_reset_single_t;


/**
 * 本次通信息的下行信息
 */
typedef struct down_info {
	uint8_t size;		///< 业务数据长度
	uint8_t *business_data;	///< 业务数据。
#ifdef USE_NODE_STATUS
	uint8_t result_ind;	///< 该字段详见模块使用说明书	
	int8_t snr;			///< 下行SNR，若无下行，则该值为0
	int8_t rssi;		///< 下行RSSI，若无下行，则该值为0
	uint8_t datarate;	///< 本次下行对应的上行速率
#endif
} down_info_t;

/**
 * 下行信息的链表结构
 */
typedef struct list {
	down_info_t down_info;   ///< 下行信息结构
	struct list *next;
}down_list_t;

/*=====================================END======================================*/

/*------------------------------提供给用户的接口函数----------------------------*/
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
/*--------------------------------提供给用户的变量------------------------------*/
extern bool node_join_successfully;
/*=====================================END======================================*/

#endif
