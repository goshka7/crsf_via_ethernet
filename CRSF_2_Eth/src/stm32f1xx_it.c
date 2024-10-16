/*=============================================================*/
#include "stm32f1xx_it.h"
#include "stm32f1xx_hal.h"
#include "stm32f103_board.h"
extern TIM_HandleTypeDef  TimHandle;
extern TIM_HandleTypeDef  tim4_hndl;
extern UART_HandleTypeDef UartHandle;
static uint32_t	uart_rx_cnt= 0;	
extern uint32_t	uart_rx_num;	
static uint32_t uart_cur_buf= 0;
extern uint8_t uart_rx_buf[DATA_BUF_SIZE];
/*=============================================================*/
void HardFault_Handler(void)
{
  while (1)
		continue;
}
/*=============================================================*/
void MemManage_Handler(void)
{
  while (1)
		continue;
}
/*=============================================================*/
void BusFault_Handler(void)
{
  while (1)
		continue;
}
/*=============================================================*/
void UsageFault_Handler(void)
{
  while (1)
		continue;
}
/*=============================================================*/
void SVC_Handler(void)
{
}
/*=============================================================*/
void DebugMon_Handler(void)
{
}
/*=============================================================*/
void PendSV_Handler(void)
{
}
/*=============================================================*/
void SysTick_Handler(void)
{
  HAL_IncTick();
}
/*=============================================================*/
void TIM3_IRQHandler(void)
{
  if(uart_rx_cnt)
  {
	  if(!uart_cur_buf)
	  {
		  uart_rx_num= uart_rx_cnt;
	  }
	  uart_cur_buf= (uart_cur_buf)? 0 : 1;
  }
	uart_rx_cnt= 0;  
  HAL_TIM_IRQHandler(&TimHandle);
}
/*=============================================================*/
void TIM4_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&tim4_hndl);
}
/*=============================================================*/
void USARTx_IRQHandler(void)
{
	if( USART1->SR & (UART_FLAG_RXNE | UART_FLAG_ORE))
	{
		uint8_t ch;
		HAL_TIM_Base_Stop_IT(&TimHandle);
		ch= (uint8_t)USART1->DR;
		if( !uart_cur_buf)
			uart_rx_buf[uart_rx_cnt]= ch;
		uart_rx_cnt++;
		if(uart_rx_cnt > 128)
			uart_rx_cnt= 0;		
		TimHandle.Instance->CNT= TimHandle.Instance->ARR;
		HAL_TIM_Base_Start_IT(&TimHandle);
	}
}
