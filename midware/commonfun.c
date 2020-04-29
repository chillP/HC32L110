/*!
 * @file       commonfun.c
 * @brief      应用层常用函数的实现
 * @details    将应用层常用功能封装为函数，方便上层调用
		   
 * @copyright  Revised BSD License, see section LICENSE. \ref LICENSE
 * @author     Lierda-WSN-LoRaWAN-Team
 * @date       2020-03-20
 * @version    V1.0
 */
 
 
 /* Includes ------------------------------------------------------------------*/
#include "commonfun.h"
#include "LSD_LoRaWAN_ICA_Driver.h"

extern bool timeout_start_flag;
 
/**
  * @简介：十六进制数转十进制数         
  * @参数：   
  * @返回值：无
  */
long hexToDec(char *source)
{
    long sum = 0;
    long t = 1;
    int i, len;
 
    len = strlen(source);
    //len = 2;
	for(i=len-1; i>=0; i--)
    {
        sum += t * getIndexOfSigns(*(source + i));
        t *= 16;
    }  
 
    return sum;
}
 
/**
  * @简介：返回ch字符在sign数组中的序号         
  * @参数：   
  * @返回值：无
  */
int getIndexOfSigns(char ch)
{
    if(ch >= '0' && ch <= '9')
    {
        return ch - '0';
    }
    if(ch >= 'A' && ch <='F') 
    {
        return ch - 'A' + 10;
    }
    if(ch >= 'a' && ch <= 'f')
    {
        return ch - 'a' + 10;
    }
    return -1;
}
 
/**
  * @简介：整型转字符串。              
  * @参数：   
  * @返回值：无
  */
void IntToStr(uint8_t* str, int32_t intnum)
{
    uint32_t i, Div = 1000000000, j = 0, Status = 0;
    
    if(intnum < 0)
    {
        intnum = intnum*(-1);
        str[j++] = '-';
    }
    
    for (i = 0; i < 10; i++)
    {
        str[j++] = (intnum / Div) + 48; /* '0' */
        
        intnum = intnum % Div;
        Div /= 10;
        if ((str[j-1] == '0') & (Status == 0))
        {
            j = 0;
        }
        else
        {
            Status++;
        }
    }
}

/**
  * @简介：连接两个字符串。              
  * @参数：   
  * @返回值：无
  */
uint8_t *StringConcat2(uint8_t *str, const uint8_t *string)
{
    uint8_t *s = str;
    
    while(*s)
    {
        s++;
    }
    
    while(*string)
    {
        *s++ = *string++;
    }
    
    return str;        
}

/**
  * @简介：连接两个字符串并在最后加上回车。              
  * @参数：   
  * @返回值：无
  */
uint8_t *StringConcat(uint8_t *str, const uint8_t *string)
{
    uint8_t *s = str;
    
    while(*s)
    {
        s++;
    }
    
    while(*string)
    {
        *s++ = *string++;
    }
    
    *s++ = '\r';
    *s++ = '\n';
    *s = '\0';
    
    return str;     
}

/**
  * @简介：该函数用于在M90工作在Mini RF模式下，通过AT指令设置Mini RF的射频参数。              
  * @参数： 详见说明手册
  * @返回值：0配置正确，-1配置错误
  */
uint32_t LoRaNode_SetP2P(uint32_t f,uint8_t a,uint8_t b,uint8_t c,uint8_t d,uint8_t e,uint8_t ff,uint8_t g,uint16_t h)
{
    uint8_t SetDebug[50] = "AT+RADIO=";
    uint8_t buf[10] = {0}; 
    uint8_t buf1[10] = {0}; 
    uint8_t buf2[10] = {0}; 
    uint8_t buf3[10] = {0}; 
    uint8_t buf4[10] = {0}; 
    uint8_t buf5[10] = {0}; 
    uint8_t buf6[10] = {0}; 
    uint8_t buf7[10] = {0}; 
    uint8_t buf8[10] = {0}; 
    
    uint8_t dou[2] = ",";
    char *temp = "OK";
	
	uint8_t *pMsgBuf = NULL;
    
    IntToStr(buf, f);
    StringConcat2(SetDebug, buf);
    StringConcat2(SetDebug, dou);
    
    IntToStr(buf1, a);
    StringConcat2(SetDebug, buf1);
    StringConcat2(SetDebug, dou);
    
    IntToStr(buf2, b);
    StringConcat2(SetDebug, buf2);
    StringConcat2(SetDebug, dou);
    
    IntToStr(buf3, c);
    StringConcat2(SetDebug, buf3);
    StringConcat2(SetDebug, dou);
    
    IntToStr(buf4, d);
    StringConcat2(SetDebug, buf4);
    StringConcat2(SetDebug, dou);
    
    IntToStr(buf5, e);
    StringConcat2(SetDebug, buf5);
    StringConcat2(SetDebug, dou);
    
    IntToStr(buf6, ff);
    StringConcat2(SetDebug, buf6);
    StringConcat2(SetDebug, dou);
    
    IntToStr(buf7, g);
    StringConcat2(SetDebug, buf7);
    StringConcat2(SetDebug, dou);
    
    IntToStr(buf8, h);
    StringConcat(SetDebug, buf8);
    
	node_gpio_set(wake, wakeup);
	node_gpio_set(mode, command);
	UART_WRITE_DATA((uint8_t*)SetDebug, strlen((const char *)SetDebug));
	
	UART_RECEIVE_FLAG = 0;
	UART_RECEIVE_LENGTH = 0;
	memset(UART_RECEIVE_BUFFER, 0, sizeof(UART_RECEIVE_BUFFER));
	
	timeout_start_flag = true;
	while(UART_RECEIVE_FLAG == 0)
	{
		if(true == time_out_break_ms(2000))
		{
			break;
		}
	}
	
	UART_RECEIVE_FLAG = 0;	
	if(find_string(UART_RECEIVE_BUFFER, (uint8_t *)"OK")!=NULL) return 1;
	return 0;
}
