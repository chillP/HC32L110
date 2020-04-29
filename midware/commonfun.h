#ifndef _COMMONFUN__H_
#define _COMMONFUN__H_

#include "string.h"
#include <stdint.h>

long hexToDec(char *source);
int getIndexOfSigns(char ch);
void IntToStr(uint8_t* str, int32_t intnum);
uint8_t *StringConcat2(uint8_t *str, const uint8_t *string);
uint8_t *StringConcat(uint8_t *str, const uint8_t *string);
uint32_t LoRaNode_SetP2P(uint32_t f,uint8_t a,uint8_t b,uint8_t c,uint8_t d,uint8_t e,uint8_t ff,uint8_t g,uint16_t h);

#endif

