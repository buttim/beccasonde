#ifndef __RADIOIF_H__
#define __RADIOIF_H__
#include <stdbool.h>
#include "sx1262.h"
#include "sx126x_regs.h"
#include "sx126x_hal.h"
#include "sx126x_long_pkt.h"

#define PACKET_LENGTH RS41AUX_PACKET_LENGTH  //longest packet length
#define SERIAL_LENGTH 12
#define SYNCWORD_SIZE 8

typedef struct Sonde_s {
  const char *name;
  unsigned bitRate, afcBandWidth,
    frequencyDeviation, bandwidthHz;
  int packetLength,
    partialPacketLength,  //Minimum # of bytes to be read to determine actual packet length
    preambleLengthBytes;
  uint8_t syncWordLen;
  bool flipBytes;
  uint8_t syncWord[SYNCWORD_SIZE];

  int (*processPartialPacket)(uint8_t buf[]);  //To be called after partialPacketLength bytes
                                               //have been read to determine the actual packet length
  bool (*processPacket)(uint8_t buf[]);
} Sonde;

typedef struct __attribute__((packed)) Packet_t {
  int frame;
  double lat, lng;
  float alt, hVel, vVel;
  uint16_t bkTime;
  uint8_t bkStatus, cpuTemp, radioTemp;
  bool encrypted;
  char serial[SERIAL_LENGTH + 1];
} Packet;

extern Sonde *sondes[];
extern uint32_t freq;
extern int currentSonde, rssi;
extern const uint8_t flipByte[];
extern Packet packet;

sx126x_gfsk_preamble_detector_t getPreambleLength(unsigned lengthInBytes);
sx126x_gfsk_bw_t getBandwidth(unsigned bandwidth);

uint32_t millis();

void initRadio();
bool loopRadio();
void sleepRadio();
#endif