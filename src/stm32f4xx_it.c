// Interrupt Service Routines.

#include "stm32f4xx_hal.h"

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
// This function handles Non maskable interrupt.
void NMI_Handler(void)
{
  while (1)
  {}
}

// This function handles Hard fault interrupt.
void HardFault_Handler(void)
{
  while (1)
  {}
}

// This function handles Memory management fault.
void MemManage_Handler(void)
{
  while (1)
  {}
}

// This function handles Pre-fetch fault, memory access fault.
void BusFault_Handler(void)
{
  while (1)
  {}
}

// This function handles Undefined instruction or illegal state.
void UsageFault_Handler(void)
{
  while (1)
  {}
}

// This function handles System service call via SWI instruction.
void SVC_Handler(void)
{}

// This function handles Debug monitor.
void DebugMon_Handler(void)
{}

// This function handles Pendable request for system service.
void PendSV_Handler(void)
{}

// This function handles System tick timer.
void SysTick_Handler(void)
{
  HAL_IncTick();
}


extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart2;



/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}