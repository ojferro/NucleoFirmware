#ifndef __ERROR_HANDLER_H
#define __ERROR_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f4xx_hal.h"

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {}
}

#ifdef __cplusplus
}
#endif

#endif // __ERROR_HANDLER_H