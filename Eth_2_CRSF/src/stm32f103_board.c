#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f103_board.h"
#include "w5500.h"
#include "wizchip_conf.h"
/*=============================================================*/
#define NET_PULSE_TM	 100
#define PWM_PERIOD	 	 20 

static SPI_HandleTypeDef hspi;
TIM_HandleTypeDef TimHandle;
TIM_HandleTypeDef tim4_hndl;
extern UART_HandleTypeDef UartHandle;
uint32_t tm_cnt;
//static IWDG_HandleTypeDef IwdgHandle;
void        SPIx_init(void);
extern uint32_t crsf_rate_cnt;
extern __IO ITStatus HZ_100;
/*=============================================================*/
void BSP_GPIO_init(void)
{
	GPIO_InitTypeDef  gpioinitstruct;  
	__HAL_RCC_GPIOA_CLK_ENABLE();
//	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	SPIx_CLK_ENABLE();		
	TIMx_CLK_ENABLE();
	
	gpioinitstruct.Pin    = LED_PIN;
	gpioinitstruct.Mode   = GPIO_MODE_OUTPUT_PP;
	gpioinitstruct.Pull   = GPIO_NOPULL;
	gpioinitstruct.Speed  = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_PORT, &gpioinitstruct);
	HAL_GPIO_WritePin(LED_PORT, gpioinitstruct.Pin, GPIO_PIN_RESET);	
	gpioinitstruct.Pin    =  W5500_RST_PIN;
	HAL_GPIO_Init(W5500_RST_PORT, &gpioinitstruct);
	HAL_GPIO_WritePin(W5500_RST_PORT, gpioinitstruct.Pin, GPIO_PIN_RESET);	

	gpioinitstruct.Pin    =  LED_TEST_PIN;
	HAL_GPIO_Init(LED_TEST_PORT, &gpioinitstruct);
	BSP_LedTestOff();	
	
	/* Configure SPI SCK */
	gpioinitstruct.Pin        = SPIx_SCK_PIN | SPIx_MOSI_PIN | SPIx_MOSI_PIN;
	gpioinitstruct.Mode       = GPIO_MODE_AF_PP;
	gpioinitstruct.Pull       = GPIO_PULLDOWN;
	gpioinitstruct.Speed      = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(W5500_GPIO_PORT , &gpioinitstruct);
	
	gpioinitstruct.Pin = W5500_CS_PIN;
	gpioinitstruct.Mode = GPIO_MODE_OUTPUT_PP;
	gpioinitstruct.Pull = GPIO_NOPULL;
	gpioinitstruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(W5500_GPIO_PORT, &gpioinitstruct);	
	HAL_GPIO_WritePin(W5500_GPIO_PORT, W5500_CS_PIN, GPIO_PIN_SET);	
	SPIx_CLK_ENABLE();
}
/*=============================================================*/
void BSP_LED_On(void)
{
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); 
}
/*=============================================================*/
void BSP_LED_Off(void)
{
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET); 
}

/*=============================================================*/
void BSP_LED_Toggle(void)
{
  HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
}
/*=============================================================*/
void BSP_LedTestOn(void)
{
  HAL_GPIO_WritePin(LED_TEST_PORT, LED_TEST_PIN, GPIO_PIN_SET);	
}
/*=============================================================*/
void BSP_LedTestOff(void)
{
  HAL_GPIO_WritePin(LED_TEST_PORT, LED_TEST_PIN, GPIO_PIN_RESET);	
}
/*=============================================================*/
void SPIx_init(void)
{
  hspi.Instance = SPIx;	
  if(HAL_SPI_GetState(&hspi) != HAL_SPI_STATE_RESET)
		return;
    hspi.Init.BaudRatePrescaler  = SPI_BAUDRATEPRESCALER_16;
    hspi.Init.Direction          = SPI_DIRECTION_2LINES;
    hspi.Init.CLKPhase           = SPI_PHASE_1EDGE;
    hspi.Init.CLKPolarity        = SPI_POLARITY_LOW;
    hspi.Init.CRCCalculation     = SPI_CRCCALCULATION_DISABLE;
    hspi.Init.CRCPolynomial      = 7;
    hspi.Init.DataSize           = SPI_DATASIZE_8BIT;
    hspi.Init.FirstBit           = SPI_FIRSTBIT_MSB;
    hspi.Init.NSS                = SPI_NSS_SOFT;
    hspi.Init.TIMode             = SPI_TIMODE_DISABLE;
    hspi.Init.Mode               = SPI_MODE_MASTER;
    HAL_SPI_Init(&hspi);
}
/*=============================================================*/
void on_error(void)
{
  while (1)
		continue;
}
/*=============================================================*/
/*void wdt_init(void)
{
  IwdgHandle.Instance = IWDG;
  IwdgHandle.Init.Prescaler = IWDG_PRESCALER_256;
  IwdgHandle.Init.Reload    = 0x07ffff;
	HAL_IWDG_Init(&IwdgHandle);
}	*/
/*=============================================================*/
/*void wdt_refresh(void)
{
	HAL_IWDG_Refresh(&IwdgHandle);
}*/
/*=============================================================*/
void tm_init(void)
{
	uint32_t prescaler;
	/* Compute the prescaler value to have TIMx counter clock equal to 1MHz */
  prescaler = (uint32_t)(SystemCoreClock / 1000000) - 1;
  tm_cnt= 0;
  /* Set TIMx instance */
  HAL_NVIC_SetPriority(TIMx_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(TIMx_IRQn);	
	
  /* Initialize TIMx peripheral as follows:
       + Period = 1000 - 1
       + Prescaler = (SystemCoreClock/1000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  TimHandle.Instance = TIM3;	
  TimHandle.Init.Period            = 170;
  TimHandle.Init.Prescaler         = prescaler;
  TimHandle.Init.ClockDivision     = 0;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_DOWN;
  TimHandle.Init.RepetitionCounter = 0;
  TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
    on_error(); 
  prescaler = (uint32_t)(SystemCoreClock / 1000) - 1;  /*  1KHz */
  tim4_hndl.Instance = TIM4;  
  tim4_hndl.Init.Period            = 220;
  tim4_hndl.Init.Prescaler         = prescaler;
  tim4_hndl.Init.ClockDivision     = 0;
  tim4_hndl.Init.CounterMode       = TIM_COUNTERMODE_DOWN;
  tim4_hndl.Init.RepetitionCounter = 0;
  tim4_hndl.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&tim4_hndl) != HAL_OK)
    on_error();
  __HAL_RCC_TIM4_CLK_ENABLE();
  HAL_NVIC_SetPriority(TIM4_IRQn, 2, 1);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);	  
  if (HAL_TIM_Base_Init(&tim4_hndl) != HAL_OK)
		on_error();  
  if (HAL_TIM_Base_Start_IT(&tim4_hndl) != HAL_OK)
		on_error();	  
}
/*=============================================================*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM4)
	{
			tm_cnt++;	
			HZ_100 = SET;		
	}
}	
/*=============================================================*/
void W5500_Select(void) 
{
    HAL_GPIO_WritePin(W5500_GPIO_PORT, W5500_CS_PIN, GPIO_PIN_RESET);
}
/*=============================================================*/
void W5500_Unselect(void) 
{
    HAL_GPIO_WritePin(W5500_GPIO_PORT, W5500_CS_PIN, GPIO_PIN_SET);
}
/*=============================================================*/
void W5500_ReadBuff(uint8_t* buff, uint16_t len) 
{
	HAL_StatusTypeDef status;
	status= HAL_SPI_Receive(&hspi, buff, len, SPIx_TIMEOUT_MAX);
	if(status != HAL_OK)
		on_error();
}
/*=============================================================*/
void W5500_WriteBuff(uint8_t* buff, uint16_t len) 
{
	HAL_StatusTypeDef status;
	status= HAL_SPI_Transmit(&hspi, buff, len, SPIx_TIMEOUT_MAX);
	if(status != HAL_OK)
		on_error();	
}
/*=============================================================*/
uint8_t W5500_ReadByte(void) {
    uint8_t byte;	
    W5500_ReadBuff(&byte, sizeof(byte));		
    return byte;
}
/*=============================================================*/
void W5500_WriteByte(uint8_t byte) 
	{
    W5500_WriteBuff(&byte, sizeof(byte));	
}
/*=============================================================*/
void w5500_init(void) 
{
    uint8_t rx_tx_buff_sizes[] = {2, 2, 2, 2, 2, 2, 2, 2};
	uint8_t W5500_AdrSet[2][8] = { { 2, 2, 2, 2, 2, 2, 2, 2 }, { 2, 2, 2, 2, 2, 2, 2, 2 } };//for 5500
	uint8_t tmpstr[6];
	int8_t r;
//		wiz_PhyConf phy;
    wiz_NetInfo net_info = {
        .mac  = { 0xEA, 0x11, 0x22, 0x33, 0x44, 0x55 },
        .dhcp = NETINFO_STATIC,
				.ip   = {192,168,1,111},
				.sn   = {255,255,255,0},
				.gw   = {192,168,1,110}
    };		
	HAL_Delay(100);
	HAL_GPIO_WritePin(W5500_RST_PORT, W5500_RST_PIN, GPIO_PIN_SET);
    reg_wizchip_cs_cbfunc(W5500_Select, W5500_Unselect);
    reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte);
    reg_wizchip_spiburst_cbfunc(W5500_ReadBuff, W5500_WriteBuff);
    if((r= wizchip_init(rx_tx_buff_sizes, rx_tx_buff_sizes)) != 0)
		on_error();
    setSHAR(net_info.mac);
    wizchip_setnetinfo(&net_info);
	if( (r= ctlwizchip(CW_RESET_PHY,(void*)tmpstr)) != 0)
		on_error();		
//	ctlwizchip(CW_GET_PHYCONF, &phy);
    if( (r= ctlwizchip(CW_INIT_WIZCHIP,(void*)W5500_AdrSet)) != 0)
		on_error();			
}
/*=============================================================*/  
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{  
	GPIO_InitTypeDef  GPIO_InitStruct; 
	USARTx_CLK_ENABLE(); 
	GPIO_InitStruct.Pin       = USARTx_TX_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_PULLUP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(USARTx_GPIO_PORT, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = 		USARTx_RX_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_INPUT;
	HAL_GPIO_Init(USARTx_GPIO_PORT, &GPIO_InitStruct);
	HAL_NVIC_SetPriority(USARTx_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(USARTx_IRQn);
}
/*=============================================================*/
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
	USARTx_FORCE_RESET();
	USARTx_RELEASE_RESET();
	HAL_GPIO_DeInit(USARTx_GPIO_PORT, USARTx_TX_PIN | USARTx_RX_PIN);
	HAL_NVIC_DisableIRQ(USARTx_IRQn);
}
/*=============================================================*/
void UART_init(void)
{
    static DMA_HandleTypeDef hdma_tx;	
	
	DMAx_CLK_ENABLE();
	
	UartHandle.Instance        = USARTx;
	UartHandle.Init.BaudRate   = 400000;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits   = UART_STOPBITS_1;
	UartHandle.Init.Parity     = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode       = UART_MODE_TX_RX;
	if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
		on_error(); 
	if(HAL_UART_Init(&UartHandle) != HAL_OK)
		on_error();
	
	hdma_tx.Instance                 = USARTx_TX_DMA_CHANNEL;
	hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
	hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
	hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
	hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
	hdma_tx.Init.Mode                = DMA_NORMAL;
	hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;

	HAL_DMA_Init(&hdma_tx);

	/* Associate the initialized DMA handle to the UART handle */
	__HAL_LINKDMA(&UartHandle, hdmatx, hdma_tx);	
	HAL_NVIC_SetPriority(USARTx_DMA_TX_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(USARTx_DMA_TX_IRQn);
	
}
#if 0
HAL_StatusTypeDef board_UART_rx(USART_TypeDef *uart_regs , uint8_t *pData, uint32_t Timeout)
{
	uint32_t tm_start;
	uint32_t tm_cur;
	tm_start= tm_cnt;
	do
	{
		if( USART1->SR & UART_FLAG_RXNE)
		{	
			*pData= (uint8_t)USART1->DR;			
//		HAL_GPIO_TogglePin(LED_TEST_PORT, LED_TEST_PIN);		
			return HAL_OK;		
		}		
		tm_cur= tm_cnt - tm_start;
	} while(tm_cur < 2);
	return HAL_TIMEOUT;
}
#endif
