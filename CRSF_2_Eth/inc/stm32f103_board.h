#ifndef __STM32F103_BOARD_H
#define __STM32F103_BOARD_H


#define ETHERNET_DEBUG 
/*=============================================================*/
#include "stm32f1xx_hal.h"

#define DATA_BUF_SIZE	128
#define UDP_DEST_PORT	3001
#define UDP_PORT		3000
#define MAX_SOCK_NUM    8

#define LED_PIN                    GPIO_PIN_13
#define LED_PORT                   GPIOC
#define W5500_RST_PORT             GPIOB

#define LED_TEST_PIN               GPIO_PIN_3
#define LED_TEST_PORT              GPIOA
 
#define SPIx                       SPI1
#define SPIx_CLK_ENABLE()          __HAL_RCC_SPI1_CLK_ENABLE()

#define SPIx_GPIO_PORT			    GPIOA
#define SPIx_CS_PIN			    	GPIO_PIN_4
#define W5500_RST_PIN			    GPIO_PIN_0
#define SPIx_SCK_PIN                GPIO_PIN_5
#define SPIx_MISO_PIN               GPIO_PIN_6
#define SPIx_MOSI_PIN               GPIO_PIN_7

#define SPIx_TIMEOUT_MAX            1000

#define TIMx                        TIM3
#define TIMx_CLK_ENABLE()           __HAL_RCC_TIM3_CLK_ENABLE()
#define TIMx_IRQn                   TIM3_IRQn
#define TIMx_IRQHandler             TIM3_IRQHandler

#define USARTx                      USART1
#define USARTx_CLK_ENABLE()         __HAL_RCC_USART1_CLK_ENABLE();
#define USARTx_FORCE_RESET()        __HAL_RCC_USART1_FORCE_RESET()
#define USARTx_RELEASE_RESET()      __HAL_RCC_USART1_RELEASE_RESET()
/* Definition for USARTx Pins */
#define USARTx_TX_PIN               GPIO_PIN_9
#define USARTx_GPIO_PORT         	GPIOA
#define USARTx_RX_PIN               GPIO_PIN_10

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART1_IRQn
#define USARTx_IRQHandler                USART1_IRQHandler

#define DELAY_TIM_FREQUENCY_US 1000000

void tm_init(void);
//void pwm_init(void);
void wdt_refresh(void);
void wdt_init(void);
void BSP_GPIO_init(void);
void BSP_LED_On(void);
void BSP_LED_Off(void);
void BSP_LED_Toggle(void);
void BSP_LedTestOn(void);
void BSP_LedTestOff(void);
#ifdef ETHERNET_DEBUG
void w5500_init(void);
void SPIx_init(void);
#endif
void on_error(void);
void UART_init(void);
//HAL_StatusTypeDef board_UART_rx(USART_TypeDef *uart_regs , uint8_t *pData, uint32_t Timeout);
#endif /* __STM32F1XX_NUCLEO_H */


