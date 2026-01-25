#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include "tremo_uart.h"
#include "tremo_rcc.h"
#include "tremo_gpio.h"
#include "tremo_delay.h"
#include "tremo_pwr.h"
#include "tremo_flash.h"
#include "tremo_rcc.h"
#include "tremo_uart.h"
#include "radio.h"
#include "timer.h"
#include "rtc-board.h"
#include "sx126x-board.h"
#include "sx1262.h"
#include "radioif.h"
#include "rs41.h"
#include "m10.h"
#include "m20.h"
#include "dfm.h"
#include "rd41.h"
#include "common.h"
#include "uart.h"

uint32_t freq = 405950;
int currentSonde = 0, rssi, batt;
char version[]="1.4";
static volatile uint32_t _millis=0;
Packet packet= {
    .frame=0,
    .lat=0,
    .lng=0,
    .alt=0,
    .hVel=0,
    .vVel=0,
    .encrypted=false,
    .serial="",
};

Sonde *sondes[] = { &rs41, &m20, &m10, &dfm09, &dfm17, &rd41 };

// clang-format off
const uint8_t flipByte[] = {
    0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0,
    0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8,
    0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4,
    0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC,
    0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2,
    0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
    0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6,
    0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
    0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
    0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9,
    0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
    0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
    0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
    0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
    0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7,
    0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
};
// clang-format on

uint32_t millis() {
    return _millis;
}

void SysTick_Handler(void) {
    _millis++;
}

void red(bool on) {
    gpio_write(RED_PORT, RED_LED, on?GPIO_LEVEL_HIGH:GPIO_LEVEL_LOW);
}

void green(bool on) {
    gpio_write(GREEN_PORT, GREEN_LED, on?GPIO_LEVEL_HIGH:GPIO_LEVEL_LOW);
}

void blue(bool on) {
    gpio_write(BLUE_PORT, BLUE_LED, on?GPIO_LEVEL_HIGH:GPIO_LEVEL_LOW);
}

void white(bool on) {
    gpio_write(WHITE_PORT, WHITE_LED, on?GPIO_LEVEL_HIGH:GPIO_LEVEL_LOW);
}

void yellow(bool on) {
    gpio_write(YELLOW_PORT, YELLOW_LED, on?GPIO_LEVEL_HIGH:GPIO_LEVEL_LOW);
}

void printPacket() {
    printf(">%ld,%s,%s,%d,%.6f,%.6f,%.1f,%.1f,%.1f\n",
	freq,sondes[currentSonde]->name,
	packet.serial,packet.frame,packet.lat,packet.lng,packet.alt,packet.vVel,packet.hVel);
}

int sondeTypeFromSondeName(const char *name) {
    for (int i=0;i<sizeof sondes/sizeof(*sondes);i++)
	if (strcmp(name,sondes[i]->name)==0)
	    return i;
    return -1;
}

typedef struct Settings_s {
    uint32_t freq;
    uint8_t sondeType;
} Settings;

void loadSettings() {
    Settings s;
    
    memcpy(&s,(void*)(0x08020000-sizeof s),sizeof s);
    if (s.sondeType<0 || s.sondeType>sizeof sondes/sizeof *sondes)
	s.sondeType=0;
    if (s.freq<400000UL || s.freq>=406000UL)
	s.freq=403000UL;
    
    freq=s.freq;
    currentSonde=s.sondeType;
}

void saveSettings() {
    Settings s={ .freq=freq, .sondeType=currentSonde };
    const int size=sizeof s, addr=0x08020000-size;
    int err=flash_erase_page(addr);
    if (err!=ERRNO_OK) {
	printf("errore cancellazione flash\n\r");
	return;
    }
    flash_program_bytes(addr, (uint8_t*)&s, sizeof s);
}

int main(void) {
    uint32_t tLedOn=0, tLastPoll=0;

    rcc_set_adc_clk_source(RCC_ADC_CLK_SOURCE_RCO48M);
    rcc_enable_oscillator(RCC_OSC_XO32K, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_UART0, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOD, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_PWR, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_RTC, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_LORA, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_SAC, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_RNGC, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_CRC, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_WDG, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_SYSCFG, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_ADC, true);
    
    gpio_init(GPIOA, GPIO_PIN_2, GPIO_MODE_INPUT_PULL_DOWN);
    gpio_set_iomux(BLUE_PORT,BLUE_LED,0);
    gpio_init(RED_PORT, RED_LED, GPIO_MODE_OUTPUT_PP_LOW);
    gpio_init(GREEN_PORT, GREEN_LED, GPIO_MODE_OUTPUT_PP_LOW);
    gpio_init(BLUE_PORT, BLUE_LED, GPIO_MODE_OUTPUT_PP_LOW);
    gpio_init(WHITE_PORT, WHITE_LED, GPIO_MODE_OUTPUT_PP_LOW);
    gpio_init(YELLOW_PORT, YELLOW_LED, GPIO_MODE_OUTPUT_PP_LOW);

    initUart();
    delay_init();

    delay_ms(100);
    pwr_xo32k_lpm_cmd(true);
    
    loadSettings();

    for (int i=0;i<3;i++) {
	blue(true);
	delay_ms(100);
	blue(false);
	delay_ms(400);
    }

    RtcInit();

    NVIC_EnableIRQ(GPIO_IRQn);
    NVIC_SetPriority(GPIO_IRQn, 2);
    
    initRadio();
    while (true) {
	int i, n;
	uint32_t f;
	char s[10];
	
	if (uartIsAvailable()) {
	    char ch=uartGetChar();
	    switch (ch) {
	    case '?':
		printPacket();
		break;
	    case '!':
		f=0;
		n=-1;
		for (i=0;i<6;i++) {
		    ch=uartGetChar();
		    if (!isdigit(ch)) {
			printf("mi aspettavo una cifra\n");
			break;
		    }
		    f*=10;
		    f+=ch-'0';
		    //TODO:
		}
		if (f<400000UL || f>=406000) {
		    printf("frequenza non valida: %ld\n",f);
		    break;
		}
		if (uartGetChar()!=',') {
		    printf("mi aspettavo virgola\n");
		    break;
		}
		for (i=0;i<sizeof s;i++) {
		    s[i]=uartGetChar();
		    if (s[i]=='\n'  || s[i]=='\r') {
			s[i]=0;
			n=sondeTypeFromSondeName(s);
			break;
		    }
		}
		if (n==-1) {
		    printf("sonda '%s' sconosciuta\n",s);
		    break;
		}
		freq=f;
		currentSonde=n;
		saveSettings();
		initRadio();
		break;
	    case '\r':
	    case '\n':
		break;
	    default:
		printf("???\n");
		break;
	    }
	}
	
        if (loopRadio()) {
            green(true);
            tLedOn=millis();
	    printPacket();
        }
        if (tLedOn!=0 && millis()-tLastPoll>50) {
            green(false);
            tLedOn=0;
        }
    }
}
