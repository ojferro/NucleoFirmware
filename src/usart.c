#include "usart.h"
#include "string.h"
#include <stdarg.h>
#include <stdio.h>

UART_HandleTypeDef huart2;

// Pins A2 and A3 are used by the USB port. This lets us write to serial console.
#define USART2_TX_PIN GPIO_PIN_2
#define USART2_RX_PIN GPIO_PIN_3

void USART2_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);
}

// Overrides the HAL_UART_MspInit() in the HAL, which is a no-op.
// This is necessary to configure which pins UART2 uses.
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
  if(uartHandle->Instance==USART2)
  {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(uartHandle->Instance==USART2)
    {
      __HAL_RCC_USART2_CLK_ENABLE();

      __HAL_RCC_GPIOA_CLK_ENABLE();
      /**USART2 GPIO Configuration
      PA2     ------> USART2_TX
      PA3     ------> USART2_RX
      */
      GPIO_InitStruct.Pin = USART2_TX_PIN|USART2_RX_PIN;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

      HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
      HAL_NVIC_EnableIRQ(USART2_IRQn);
    }
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART2)
  {
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, USART2_TX_PIN|USART2_RX_PIN);

    HAL_NVIC_DisableIRQ(USART2_IRQn);
  }
}

void debugLog(const char* str)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 10);
}

void debugLogFmt(const char* format, ...)
{
    va_list args;
    va_start(args, format);

    char buffer[256];
    vsnprintf(buffer, sizeof(buffer), format, args);

    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 10);

    va_end(args);
}