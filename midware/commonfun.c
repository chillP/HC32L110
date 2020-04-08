/*!
 * @file       commonfun.c
 * @brief      Ӧ�ò㳣�ú�����ʵ��
 * @details    ��Ӧ�ò㳣�ù��ܷ�װΪ�����������ϲ����
		   
 * @copyright  Revised BSD License, see section LICENSE. \ref LICENSE
 * @author     Lierda-WSN-LoRaWAN-Team
 * @date       2020-03-20
 * @version    V1.0
 */
 
 
 /* Includes ------------------------------------------------------------------*/
 #include "commonfun.h"
 

 
/**
  * @��飺ʮ��������תʮ������         
  * @������   
  * @����ֵ����
  */
long hexToDec(char *source)
{
    long sum = 0;
    long t = 1;
    int i, len;
 
    len = strlen(source);
    for(i=len-1; i>=0; i--)
    {
        sum += t * getIndexOfSigns(*(source + i));
        t *= 16;
    }  
 
    return sum;
}
 
/**
  * @��飺����ch�ַ���sign�����е����         
  * @������   
  * @����ֵ����
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
 
