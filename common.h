#ifndef __COMMON_H
#define __COMMON_H
#include <stdbool.h>

#define LORA_FREQ		434650000UL
#define LORA_MIN_FREQ		433050000
#define LORA_MAX_FREQ		434790000
#define LORA_BANDWIDTH             1         // 250 kHz,
#define LORA_SPREADING_FACTOR      12        // [SF7..SF12]
#define LORA_PREAMBLE_LENGTH       8
#define LORA_SYMBOL_TIMEOUT        5         // Symbols
#define LORA_CODINGRATE            1         // 4/5
#define LORA_SYMBOL_TIMEOUT        5
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON       false
#define QRG_ENTRIES		5
#define SERIAL_LENGTH 		12
#define MAX_LORA_PACKET		256	//TODO: calcolare
#define BOOT_MODE_REG_BIT       (1UL<<29)

#define RED_LED 		GPIO_PIN_5
#define RED_PORT		GPIOA
#define GREEN_LED 		GPIO_PIN_4
#define GREEN_PORT		GPIOA
#define BLUE_LED 		GPIO_PIN_7
#define BLUE_PORT		GPIOA
#define WHITE_LED 		GPIO_PIN_14
#define WHITE_PORT		GPIOA
#define YELLOW_LED 		GPIO_PIN_15
#define YELLOW_PORT		GPIOA

typedef struct LoraConfig_t {
    uint32_t freq;
    int bandwidth,
	spreadingFactor,
	codingRate;
} LoraConfig;

typedef struct QRGentry_t {
    uint32_t freq;
    char serial[SERIAL_LENGTH];
    double lat,lng;
    float alt;
    uint8_t sondeType;
} QRGentry;

typedef struct __attribute__((packed)) ResetRequest_t {
    char cmd;
    bool OTA;
} ResetRequest;

typedef struct __attribute__((packed)) AddRequest_t {
    char cmd;
    uint32_t freq;
    uint8_t sondeType;
} AddRequest;

typedef struct __attribute__((packed)) RemoveRequest_t {
    char cmd;
    uint8_t row;
} RemoveRequest;

typedef struct __attribute__((packed)) AddRemoveAnswer_t {
    char check;
    bool result;
} AddRemoveAnswer;

typedef struct __attribute__((packed)) QRGAnswer_t {
    char check;
    uint8_t battery;
    int nEntries;
    QRGentry QRG[];
} QRGAnswer;

typedef struct __attribute__((packed)) ResetAnswer_t {
    char check;
} ResetAnswer;

typedef struct __attribute__((packed)) OTARequest_t {
    char cmd;
    uint16_t seq;
    uint8_t data[];
} OTARequest;

typedef struct __attribute__((packed)) OTAAnswer_t {
    char check;
    uint16_t seq;
} OTAAnswer;

extern LoraConfig loraConfig;
void loadLoraConfigFromFlash();
#endif