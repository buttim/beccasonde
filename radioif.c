#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include "tremo_delay.h"
#include "tremo_regs.h"
#include "lora_config.h"
#include "sx126x.h"
#include "sx126x-board.h"
#include "radioif.h"
#include "rs41.h"

#define min(a,b) ((a)<(b)?(a):(b))

uint8_t buf[RS41AUX_PACKET_LENGTH];
int nBytesRead = 0;

struct sx126x_long_pkt_rx_state pktRxState;

sx126x_gfsk_preamble_detector_t getPreambleLength(unsigned lengthInBytes) {
  sx126x_gfsk_preamble_detector_t tab[] = {
    SX126X_GFSK_PREAMBLE_DETECTOR_MIN_8BITS,
    SX126X_GFSK_PREAMBLE_DETECTOR_MIN_16BITS,
    SX126X_GFSK_PREAMBLE_DETECTOR_MIN_24BITS,
    SX126X_GFSK_PREAMBLE_DETECTOR_MIN_32BITS
  };
  if (lengthInBytes <= 0 || lengthInBytes - 1 > sizeof tab / sizeof(*tab))
    return SX126X_GFSK_PREAMBLE_DETECTOR_OFF;
  return tab[lengthInBytes - 1];
}

sx126x_gfsk_bw_t getBandwidth(unsigned bandwidth) {
  // clang-format off
  sx126x_gfsk_bw_t tab[] = {
    SX126X_GFSK_BW_4800,SX126X_GFSK_BW_5800,SX126X_GFSK_BW_7300,
    SX126X_GFSK_BW_9700,SX126X_GFSK_BW_11700,SX126X_GFSK_BW_14600,
    SX126X_GFSK_BW_19500,SX126X_GFSK_BW_23400,SX126X_GFSK_BW_29300,
    SX126X_GFSK_BW_39000,SX126X_GFSK_BW_46900,SX126X_GFSK_BW_58600,
    SX126X_GFSK_BW_78200,SX126X_GFSK_BW_93800,SX126X_GFSK_BW_117300,
    SX126X_GFSK_BW_156200,SX126X_GFSK_BW_187200,SX126X_GFSK_BW_234300,
    SX126X_GFSK_BW_312000,SX126X_GFSK_BW_373600,SX126X_GFSK_BW_467000,
  };
  unsigned  limits[] = {
    4800,5800,7300,9700,11700,14600,19500,23400,29300,39000,46900,
    58600,78200,93800,117300,156200,187200,234300,312000,373600,467000,
  };
  // clang-format on
  for (int i = 0; i < sizeof limits / sizeof *limits - 1; i++)
    if (bandwidth < (limits[i] + limits[i + 1]) / 2) return tab[i];
  return SX126X_GFSK_BW_467000;
}

void initRadio() {
  *packet.serial = '\0';
  packet.lat = packet.lng = packet.alt = 0;
    
	NVIC_DisableIRQ(LORA_IRQn);
  
	LORAC->CR0 = 0x00000200;

  LORAC->SSP_CR0 = 0x07;
  LORAC->SSP_CPSR = 0x02;

  //wakeup lora 
  //avoid always waiting busy after main reset or soft reset
  if(LORAC->CR1 != 0x80) {
      delay_us(20);
      LORAC->NSS_CR = 0;
      delay_us(20);
      LORAC->NSS_CR = 1;
  }

  LORAC->SSP_CR1 = 0x02;
  gpio_set_iomux(GPIOD, CONFIG_LORA_RFSW_CTRL_PIN, 3);
  SX126xAntSwOn();
  
  sx126x_reset(NULL);
  sx126x_wakeup(NULL );
  sx126x_set_standby(NULL, SX126X_STANDBY_CFG_RC);
  sx126x_set_dio2_as_rf_sw_ctrl(NULL, true);

  sx126x_set_reg_mode( NULL,SX126X_REG_MODE_DCDC);

  uint8_t syncWord[SYNCWORD_SIZE];
  for (int i = 0; i < SYNCWORD_SIZE; i++)
    syncWord[i] = sondes[currentSonde]->flipBytes ? flipByte[sondes[currentSonde]->syncWord[i]] : sondes[currentSonde]->syncWord[i];

  sx126x_mod_params_gfsk_t modParams = {
    .br_in_bps = sondes[currentSonde]->bitRate,
    .fdev_in_hz = sondes[currentSonde]->frequencyDeviation,
    .pulse_shape = SX126X_GFSK_PULSE_SHAPE_OFF,
    .bw_dsb_param = getBandwidth(sondes[currentSonde]->bandwidthHz),
  };
  sx126x_pkt_params_gfsk_t pktParams = {
    .preamble_len_in_bits = 0,
    .preamble_detector = getPreambleLength(sondes[currentSonde]->preambleLengthBytes),
    .sync_word_len_in_bits = (uint8_t)sondes[currentSonde]->syncWordLen,
    .address_filtering = SX126X_GFSK_ADDRESS_FILTERING_DISABLE,
    .header_type = SX126X_GFSK_PKT_FIX_LEN,
    .pld_len_in_bytes = (uint8_t)min(255, sondes[currentSonde]->packetLength),
    .crc_type = SX126X_GFSK_CRC_OFF,
    .dc_free = SX126X_GFSK_DC_FREE_OFF
  };
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  sx126x_status_t res = sx126x_reset(NULL);
#pragma GCC diagnostic pop
  //~ printf("sx126x_reset %d\n",res);
  //res = sx126x_set_dio3_as_tcxo_ctrl(NULL, SX126X_TCXO_CTRL_1_7V, 5<<6);
  //printf("sx126x_set_dio3_as_tcxo_ctrl %d\n",res);

  delay_ms(1);
  res = sx126x_set_standby(NULL, SX126X_STANDBY_CFG_RC);
  //~ printf("sx126x_set_standby %d\n", res);
  SX126xCheckDeviceReady( );

  res = sx126x_set_reg_mode(NULL, SX126X_REG_MODE_DCDC);
  //~ printf("sx126x_set_reg_mode %d\n", res);

  res = sx126x_set_pkt_type(NULL, SX126X_PKT_TYPE_GFSK);
  //~ printf("sx126x_set_pkt_type %d\n", res);
  res = sx126x_set_rf_freq(NULL, freq * 1000UL);
  //~ printf("sx126x_set_rf_freq %d\n", res);
  res = sx126x_set_gfsk_mod_params(NULL, &modParams);
  //~ printf("sx126x_set_gfsk_mod_params %d\n", res);

  if (sondes[currentSonde]->packetLength > 255) {
    res = sx126x_long_pkt_rx_set_gfsk_pkt_params(NULL, &pktParams);
    //~ printf("sx126x_long_pkt_rx_set_gfsk_pkt_params %d\n", res);
    res = sx126x_set_dio_irq_params(NULL, SX126X_IRQ_SYNC_WORD_VALID, SX126X_IRQ_SYNC_WORD_VALID, SX126X_IRQ_NONE, SX126X_IRQ_NONE);
    //~ printf("sx126x_set_dio_irq_params %d\n", res);
  } else {
    res = sx126x_set_gfsk_pkt_params(NULL, &pktParams);
    //~ printf("sx126x_rx_set_gfsk_pkt_params %d\n", res);
    res = sx126x_set_dio_irq_params(NULL, SX126X_IRQ_RX_DONE, SX126X_IRQ_RX_DONE, SX126X_IRQ_NONE, SX126X_IRQ_NONE);
    //~ printf("sx126x_set_dio_irq_params %d\n", res);
  }

  res = sx126x_set_gfsk_sync_word(NULL, syncWord, sizeof syncWord);
  //~ printf("sx126x_set_gfsk_sync_word %d\n", res);

  res = sx126x_cal_img(NULL, 0x6B, 0x6F);
  uint8_t val = 0x96;
  res = sx126x_write_register(NULL, SX126X_REG_RXGAIN, &val, 1);

  res = sx126x_clear_device_errors(NULL);

  if (sondes[currentSonde]->packetLength > 255) {
    res = sx126x_long_pkt_set_rx_with_timeout_in_rtc_step(NULL, &pktRxState, SX126X_RX_CONTINUOUS);
    //~ printf("sx126x_long_pkt_set_rx _with_timeout_in_rtc_step %d\n", res);
  } else {
    res = sx126x_set_rx_with_timeout_in_rtc_step(NULL, 0);  //SX126X_RX_CONTINUOUS);
    //~ printf("sx126x_set_rx_with_timeout_in_rtc_step %d\n", res);
  }
}


bool loopRadio() {
  static uint64_t tLastRead = 0, tLastPacket = 0, tLastRSSI = 0;
  static int16_t actualPacketLength = 0;
  bool validPacket = false;

  sx126x_status_t res;
  sx126x_pkt_status_gfsk_t pktStatus;
  sx126x_rx_buffer_status_t bufStatus;
  
  if (sondes[currentSonde]->packetLength > 255) {
    if (LORAC->SR&(1<<5)) {//digitalRead(RADIO_DIO_1) == HIGH) {
      //~ printf("SYNC\r\n");
      tLastPacket = tLastRead = millis();
      nBytesRead = 0;
      actualPacketLength = sondes[currentSonde]->packetLength;
      res = sx126x_clear_irq_status(NULL, SX126X_IRQ_SYNC_WORD_VALID);
      res = sx126x_get_gfsk_pkt_status_raw(NULL, &pktStatus);
      rssi = (uint8_t)pktStatus.rssi_sync;
    }
    if (tLastRead != 0 && millis() - tLastRead > 1000) {
      res = sx126x_long_pkt_rx_prepare_for_last(NULL, &pktRxState, 0);
      tLastRead = 0;
      nBytesRead = 0;
    }
    if (tLastRead != 0 && millis() - tLastRead > 300) {
      tLastRead = millis();
      uint8_t read;
      res = sx126x_long_pkt_rx_get_partial_payload(NULL, &pktRxState, buf + nBytesRead, sizeof buf - nBytesRead, &read);
      if (read == 0) {
        tLastRead = 0;
        nBytesRead = 0;
        validPacket = false;
      }
      //~ printf("  READ %d\r\n", read);
      if (sondes[currentSonde]->processPartialPacket != NULL && nBytesRead < sondes[currentSonde]->partialPacketLength && nBytesRead + read >= sondes[currentSonde]->partialPacketLength)
        actualPacketLength = sondes[currentSonde]->processPartialPacket(buf);
      nBytesRead += read;
      if (actualPacketLength - nBytesRead <= 255) {
        res = sx126x_long_pkt_rx_prepare_for_last(NULL, &pktRxState, actualPacketLength - nBytesRead);
        if (res) {}///////////elimina warning
        //~ printf("prepare for last... nBytesRead:%d, actualPacketLength: %d\r\n",nBytesRead,actualPacketLength);
      }

      if (nBytesRead == actualPacketLength) {
        //~ printf("fine\r\n");
        sx126x_long_pkt_rx_complete(NULL);
        sx126x_long_pkt_set_rx_with_timeout_in_rtc_step(NULL, &pktRxState, SX126X_RX_CONTINUOUS);
        tLastRead = 0;
        nBytesRead = 0;

        //dump(buf, actualPacketLength);
        validPacket = sondes[currentSonde]->processPacket(buf);
      }
    }
  } else {
    if (LORAC->SR&(1<<5)) {//digitalRead(RADIO_DIO_1) == HIGH) {
      printf("PKT\n\r");
      tLastPacket = millis();
      res = sx126x_clear_irq_status(NULL, SX126X_IRQ_RX_DONE);
      res = sx126x_set_rx_with_timeout_in_rtc_step(NULL, 0);
      res = sx126x_get_rx_buffer_status(NULL, &bufStatus);
      res = sx126x_read_buffer(NULL, bufStatus.buffer_start_pointer, buf, bufStatus.pld_len_in_bytes);
      res = sx126x_get_gfsk_pkt_status_raw(NULL, &pktStatus);
      rssi = (uint8_t)pktStatus.rssi_sync;
      //printf("PKT %d bytes\n", bufStatus.pld_len_in_bytes);
      //dump(buf, PACKET_LENGTH);
      validPacket = sondes[currentSonde]->processPacket(buf);
    }
  }

  if (validPacket) {
    //~ printf("invio notifica pacchetto\n\r");
    //~ BLENotifyPacket();
  } else {
    if ((tLastPacket == 0 || millis() - tLastPacket > 3000) && (tLastRSSI == 0 || millis() - tLastRSSI > 500)) {
      uint8_t t;
      sx126x_get_rssi_inst_raw(NULL, &t);
      rssi = t;
      //printf("rssi: %d\n", rssi);
      tLastRSSI = millis();
    }
  }
  return validPacket;
}

void sleepRadio() {
  sx126x_set_sleep(NULL, SX126X_SLEEP_CFG_COLD_START);
}
