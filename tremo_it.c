#include "tremo_gpio.h"
#include "tremo_wdg.h"
#include "tremo_it.h"
#include <stdio.h>

extern void RtcOnIrq(void);

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void) {
    printf("NMI_Handler\n\r");
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void) {
    gpio_write(GPIOA, GPIO_PIN_4, GPIO_LEVEL_HIGH);
    gpio_write(GPIOA, GPIO_PIN_5, GPIO_LEVEL_HIGH);
    printf("******HardFault_Handler\n\r");
    /* Go to infinite loop when Hard Fault exception occurs */
    while(1)
	;
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void) {
    printf("******MemManage_Handler\r\n");
    gpio_write(GPIOA, GPIO_PIN_4, GPIO_LEVEL_HIGH);
    /* Go to infinite loop when Memory Manage exception occurs */
    while(1)
	;
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void) {
    gpio_write(GPIOA, GPIO_PIN_5, GPIO_LEVEL_HIGH);
    printf("******BusFault_Handler\n\r");
    /* Go to infinite loop when Bus Fault exception occurs */
    while(1)
	;
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void) {
    gpio_write(GPIOA, GPIO_PIN_7, GPIO_LEVEL_HIGH);
    printf("******UsageFault_Handler\n\r");
    /* Go to infinite loop when Usage Fault exception occurs */
    while(1)
	;
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void) {
    printf("SVC_Handler\n\r");
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void) {
    printf("PendSV_Handler\n\r");
}

/**
  * @brief  This function handles PWR Handler.
  * @param  None
  * @retval None
  */
void PWR_IRQHandler() {
    printf("PWR_IRQHandler\n\r");
}

void RTC_IRQHandler(void) {
    RtcOnIrq();
}

/******************************************************************************/
/*                 Tremo Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_cm4.S).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void) {
}*/

