#include "sx126x_hal.h"
#include "sx126x-board.h"
#include "tremo_regs.h"
#include "tremo_delay.h"

uint16_t SpiInOut( uint16_t outData );


sx126x_hal_status_t sx126x_hal_write(const void* context, const uint8_t* command, const uint16_t command_length,
                                     const uint8_t* data, const uint16_t data_length) {
    int i;

    SX126xCheckDeviceReady( );
    LORAC->NSS_CR = 0;
    for (i = 0; i < command_length; i++)
    SpiInOut(command[i]);
    for (i = 0; i < data_length; i++)
	SpiInOut(data[i]);
    LORAC->NSS_CR = 1;
    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_read(const void* context, const uint8_t* command, const uint16_t command_length,
                                    uint8_t* data, const uint16_t data_length) {
    int i;

    SX126xCheckDeviceReady( );

    LORAC->NSS_CR = 0;

    for (i = 0; i < command_length; i++)
	SpiInOut( ( uint8_t )command[i]);
    for (i = 0; i < data_length; i++)
	data[i] = SpiInOut( 0x00 );

    LORAC->NSS_CR = 1;

    SX126xWaitOnBusy( );  return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_reset(const void* context) {
    LORAC->CR1 &= ~(1<<5);  //nreset
    delay_us(100);
    LORAC->CR1 |= 1<<5;    //nreset release
    LORAC->CR1 &= ~(1<<7); //por release
    LORAC->CR0 |= 1<<5; //irq0
    LORAC->CR1 |= 0x1;  //tcxo
    
    while((LORAC->SR & 0x100));  
    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup(const void* context) {
    __disable_irq();

    LORAC->NSS_CR = 0;
    delay_us(20);

    SpiInOut( RADIO_GET_STATUS );
    SpiInOut( 0x00 );

    LORAC->NSS_CR = 1;

    // Wait for chip to be ready.
    SX126xWaitOnBusy( );

    __enable_irq();
    return SX126X_HAL_STATUS_OK;
}
