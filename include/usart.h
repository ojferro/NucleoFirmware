#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart2;

void USART2_Init(void);

void debugLog(const char* str);
void debugLogFmt(const char* str, ...);

#ifdef __cplusplus
}
#endif