#ifndef _UART_MID__H_
#define _UART_MID__H_

#include <stdint.h>
#include "adt.h"

#define UART0_RXBUFSIZE 255
#define UART1_RXBUFSIZE 255
#define LPUART_RXBUFSIZE 512

void SysTick_init(void);
uint32_t getTick(void);
void Uart0_RxIntCallback(void);
void Uart1_RxIntCallback(void);
void Lpuart_RxIntCallback(void);
void Uart0_ErrIntCallback(void);
void Uart1_ErrIntCallback(void);
void Lpuart_ErrIntCallback(void);
void Uart0_send_string(uint8_t *str);
void Uart1_send_string(uint8_t *str);
void Lpuart_send_string(uint8_t *str);
void Lpuart_send_data(uint8_t *pdata, uint16_t Length);
void Uart0_init(void);
void Uart1_init(void);
void Lpuart1_init(void);
void LptInt(void);
void Lptimer_Init(void);
void Lora_reset(void);
void Gpio_IRQHandler(uint8_t u8Param);
void Gpio_vdetect_init(void);
void Gpio_key_init(void);
void Gpio_lora_Init(void);
void PcaInt(void);
void Wdt_init(void);
void WdtCallback(void);
void Pca_led_init(void);
void Pca_Timer_Init(void);
void Adtimer_init(void);
	
typedef enum 
{
	ON,  //µ„¡¡
	OFF,  //œ®√
	BREATH,  //10s∫ÙŒ¸
	BLINK,  //50ms…¡À∏“ª¥Œ
	QUICKFLASH,  //500msøÏ…¡
	SLOWFALSH,  //1s¬˝…¡
}ledStatType;

#endif
