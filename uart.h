#ifndef __UART_H_
#define __UART_H_
#include <stdbool.h>
void initUart();
bool uartIsAvailable();
void uartFlush();
char uartGetChar();
bool uartGetLine(char *s,int len);
bool uartGetNumber(char *prompt,uint32_t *n,uint32_t min,uint32_t max,bool showDefault);
void dump(uint8_t buf[], int size, int rowLen);
#endif