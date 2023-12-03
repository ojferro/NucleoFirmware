#ifndef __PERIPHERAL_CONFIG_H__
#define __PERIPHERAL_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdbool.h>


///// GPIO //////////////////////////
#define CAN_CS_GPIO_Port    GPIOB
#define CAN_CS_Pin          GPIO_PIN_6

void GPIO_Init(void);
/////////////////////////////////////

///////////// I2C ///////////////////
extern I2C_HandleTypeDef hi2c1;
bool I2C1_Init(void);
/////////////////////////////////////

///////////// SPI ///////////////////
extern SPI_HandleTypeDef hspi1;
bool SPI1_Init(void);
/////////////////////////////////////


////////////// DMA //////////////////
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
void DMA_Init(void);
/////////////////////////////////////

//////////////// USART //////////////
extern UART_HandleTypeDef huart2;

// Pins A2 and A3 are used by the USB port. This lets us write to serial console.
#define USART2_TX_PIN GPIO_PIN_2
#define USART2_RX_PIN GPIO_PIN_3

bool USART2_Init(void);
/////////////////////////////////////

///// CONFIGURE ALL PERIPHERALS /////
bool InitializeHardware();
/////////////////////////////////////

#ifdef __cplusplus
}
#endif
#endif /*__PERIPHERAL_CONFIG_H__ */
