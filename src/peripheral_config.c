#include "peripheral_config.h"
#include <stdbool.h>

///////////////////////////////////////////////////
/////////////////// GPIO //////////////////////////
///////////////////////////////////////////////////
void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (CAN_CS_GPIO_Port == GPIOA)
         __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (CAN_CS_GPIO_Port == GPIOB)
        __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (CAN_CS_GPIO_Port == GPIOC)
        __HAL_RCC_GPIOC_CLK_ENABLE();

    HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = CAN_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CAN_CS_GPIO_Port, &GPIO_InitStruct);
}

///////////////////////////////////////////////////
/////////////////// SPI ///////////////////////////
///////////////////////////////////////////////////

SPI_HandleTypeDef hspi1;

bool SPI1_Init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    return HAL_SPI_Init(&hspi1) == HAL_OK;
}

///////////////////////////////////////////////////////
/////////////////////// UART //////////////////////////
///////////////////////////////////////////////////////

UART_HandleTypeDef huart2;

bool USART2_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  return HAL_UART_Init(&huart2) == HAL_OK;
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

///////////////////////////////////////////////////////
////////////////// CONFIGURE ALL //////////////////////
///////////////////////////////////////////////////////

bool InitializeHardware(){
    GPIO_Init();
    const bool spi1_status  = SPI1_Init();
    const bool uart2_status = USART2_Init();

    return spi1_status && uart2_status;
}