/*=============================================================*/
#include "stm32f1xx_it.h"
#include "stm32f1xx_hal.h"
#include "stm32f103_board.h"
extern TIM_HandleTypeDef  TimHandle;
extern TIM_HandleTypeDef  tim4_hndl;
extern UART_HandleTypeDef UartHandle;
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
	HAL_UART_IRQHandler(&UartHandle);
}
/*=============================================================*/
void USARTx_DMA_TX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(UartHandle.hdmatx);
}
/*=============================================================*/
