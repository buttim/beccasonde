#include <stdio.h>
#include <stdlib.h>
#include "tremo_uart.h"
#include "tremo_wdg.h"
#include "tremo_gpio.h"
#include "uart.h"

void initUart() {
    // uart0
    gpio_set_iomux(GPIOB, GPIO_PIN_0, 1);
    gpio_set_iomux(GPIOB, GPIO_PIN_1, 1);

    uart_config_t uart_config;
    uart_config_init(&uart_config);

    uart_config.baudrate = UART_BAUDRATE_115200;
    uart_init(CONFIG_DEBUG_UART, &uart_config);
    uart_cmd(CONFIG_DEBUG_UART, ENABLE);
}

bool uartIsAvailable() {
    return !uart_get_flag_status(CONFIG_DEBUG_UART, UART_FLAG_RX_FIFO_EMPTY);
}

void uartFlush() {
    while (uartIsAvailable()) uartGetChar();
}

char uartGetChar() {
    wdg_stop();
    char ch=uart_receive_data(CONFIG_DEBUG_UART);
    wdg_reload();
    return ch;
}

bool uartGetLine(char *s,int len) {
    int i=0;
    char ch;
    
    while (true) {
	ch=uartGetChar();
	if (ch=='\x1B') {
	    *s=0;
	    printf("\n\rANNULLATO\n\r");
	    return false;
	}
	if (ch=='\b' || ch=='\x7F') {
	    if (i>0) {	//backspace/del
		printf("\b \b");
		--i;
	    }
	    continue;
	}
	if (ch=='\n' || ch=='\r')
	    break;
	printf("%c",ch);
	if (i<len) s[i++]=ch;
    }
    s[i]=0;
    return true;
}

bool uartGetNumber(char *prompt,uint32_t *n,uint32_t min,uint32_t max,bool showDefault) {
    long t;
    char s[20];
    
    while (true) {
	printf("\n\r%s",prompt);
	if (showDefault) printf(" (%ld)",*n);
	printf(": ");
	if (!uartGetLine(s,sizeof s)) return false;
	if (*s==0 && showDefault) return true;
	
	t=atol(s);
	if (t<min || t>=max)
	    printf("\n\rValore fuori dai limiti (%ld)\n\r",t);
	else
	    break;
    }
    *n=t;
    return true;
}

void dump(uint8_t buf[], int size, int rowLen) {
    for (int i = 0; i < size; i++)
        printf("%02X%s", buf[i], i % rowLen == (rowLen-1) ? "\n\r":" ");
    if (size % rowLen != 0) printf("\n\r");
}
