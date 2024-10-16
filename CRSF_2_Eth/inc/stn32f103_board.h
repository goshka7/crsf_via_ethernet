#ifndef __STM32F103_BOARD_H
#define __STM32F103_BOARD_H

/*=============================================================*/
#include "stm32f1xx_hal.h"

#define TM_VIEW_VIDEO							 100
//#define CH58_MAX									 47
#define CH58_ROW									 8
#define CH12_MAX									 12
#define LED0_PIN                   GPIO_PIN_9
#define LED1_PIN                   GPIO_PIN_8
#define LED2_PIN                   GPIO_PIN_7
#define LED3_PIN                   GPIO_PIN_6
#if 0
#define LED4_PIN                   GPIO_PIN_5
#define LED5_PIN                   GPIO_PIN_4
#define LED6_PIN                   GPIO_PIN_3
#else
#define LED4_PIN                   GPIO_PIN_14
#define LED5_PIN                   GPIO_PIN_13
#define LED6_PIN                   GPIO_PIN_12
#endif
#define LED_GPIO_PORT              GPIOB

#define LED_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()  
#define LED_GPIO_CLK_DISABLE()     __HAL_RCC_GPIOB_CLK_DISABLE()  

#define SPIx                       SPI1
#define SPIx_CLK_ENABLE()          __HAL_RCC_SPI1_CLK_ENABLE()

#define SPIx_GPIO_PORT             GPIOA
#define W5500_GPIO_Port				     GPIOA
#define SPIx_NSS_PIN               GPIO_PIN_4
#define W5500_CS_Pin							 GPIO_PIN_4
#define W5500_INT_pin							 GPIO_PIN_3
#define W5500_RST_pin							 GPIO_PIN_2
#define SPIx_SCK_PIN               GPIO_PIN_5
#define SPIx_MISO_PIN              GPIO_PIN_6
#define SPIx_MOSI_PIN              GPIO_PIN_7
#define SPIx_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPIx_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOA_CLK_DISABLE()

#define SPIx_TIMEOUT_MAX           1000


#define TIMx                           TIM3
#define TIMx_CLK_ENABLE()              __HAL_RCC_TIM3_CLK_ENABLE()
#define TIMx_IRQn                      TIM3_IRQn
#define TIMx_IRQHandler                TIM3_IRQHandler

void tm_init(void);
void wdt_refresh(void);
void wdt_init(void);
void BSP_GPIO_init(void);
void BSP_LED_On(char Led);
void BSP_LED_Off(char Led);
void BSP_LED_Toggle(char Led);
void w5500_init(void);
void SPIx_init(void);


#endif /* __STM32F1XX_NUCLEO_H */


