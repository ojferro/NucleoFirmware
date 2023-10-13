#include "stm32f4xx_hal.h"
#include "mcp2515.hpp"
#include "ODrive.hpp"
#include "string.h"
#include "Logger.h"
#include "peripheral_config.h"
#include "error_handler.h"
#include <stdio.h>
#include <math.h>
#include <cstring>
#include <cmath>

void SystemClock_Config(void);

void handleCANRx(CANFrame& rxMsg){
    uint32_t odrvID  = rxMsg.id >> 5;
    uint32_t odrvCmd = rxMsg.id & 0b11111;

    if (odrvID == 3 && odrvCmd == ODrive::AxisCommand::ENCODER_ESTIMATES)
    {
        const auto encoderPos = can_getSignal<float>(rxMsg, 0, 32, true, 1, 0);
        const auto encoderVel = can_getSignal<float>(rxMsg, 4, 32, true, 1, 0);

        debugLogFmt("encoder_position:%.3f\n", encoderPos);
        debugLogFmt("encoder_velocity:%f\n", encoderVel);

        // Note: this is an "async" call. Axis will reply with Voltage+Current right after
        // If HAL_Delay() is set too long in the loop, new CAN messages will overwrite this one
        // axis0.getBusVoltageCurrent();
    }

    if (odrvID == 3 && odrvCmd == ODrive::AxisCommand::GET_BUS_VOLTAGE_CURRENT)
    {
        const auto busVoltage = can_getSignal<float>(rxMsg, 0, 32, true, 1, 0);
        // const auto busCurrent = can_getSignal<float>(rxMsg, 4, 32, true, 1, 0);

        debugLogFmt("bus_voltage:%.3f\n", busVoltage);
    }
}


// TODO: Remove this ugly global function, needed for DMA right now.

#define RxBuf_SIZE 16 // TODO: TOMORROW!!! This goes crazy when you increase it past 16...??? Investigate what's happening
#define MainBuf_SIZE 32

uint8_t RxBuf[RxBuf_SIZE];
uint8_t MainBuf[MainBuf_SIZE];
bool receivedData = false;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size){
    memcpy(MainBuf, RxBuf, RxBuf_SIZE);
    receivedData = true;

    // TODO: Without these 2 lines, it goes crazy. Investigate why.
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuf, RxBuf_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
// {
//     receivedData = true;
//     debugLog("dbg_msg:HAL_UART_RxCpltCallback\n");
//     HAL_UART_Receive_DMA(&huart2, RxBuf, 4);
//     HAL_UART_Transmit(&huart2, RxBuf, RxBuf_SIZE, 100);
// }

// void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
// { 
//   if (huart->Instance == USART2)
//   {
//       huart2.gState = HAL_UART_STATE_READY;
//      __NOP();
//   }
// }
 
// extern uint8_t rx_buffer[];
 
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//   if (huart->Instance == USART2)
//   {
//     HAL_UART_Receive_DMA(&huart2, rx_buffer, 4);
//     __NOP();
//   }
// }
 
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    __NOP();
  }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    // Initialize all configured peripherals
    if (!InitializeHardware())
    {
        Error_Handler();
    }


    auto mcp2515 = MCP2515(&hspi1);

    MCP2515::ERROR _e;
    _e = mcp2515.reset();
    _e = mcp2515.setBitrate(CAN_250KBPS, MCP_8MHZ);
    // _e = mcp2515.setLoopbackMode();
    _e = mcp2515.setNormalMode();

    if (_e != MCP2515::ERROR_OK)
    {
        debugLog("Error!\n");
    }

    // const auto requestBusVoltageCmd = uint32_t{0x017};
    // const auto setAxisRequestedStateCmd = uint32_t{0x007};
    // const auto setControllerModes = uint32_t{0x00B};

    // const auto idleState = uint32_t{0x001};
    // const auto calibrationSequence = uint32_t{0x003};
    // const auto closedLoopControl = uint32_t{0x008};

    // const auto velocityControl = uint32_t{0x002};
    // const auto positionControl = uint32_t{0x003};

    // Request Bus Voltage
    ODrive::Axis axis0(0x3, mcp2515);
    axis0.setRequestedState(ODrive::AxisState::IDLE);
    // axis0.getBusVoltageCurrent();
    // auto txMsg_GetBusVoltage = CANFrame{};
    // txMsg_GetBusVoltage.id = axisCANID << 5 | requestBusVoltageCmd | CAN_RTR_FLAG;
    // txMsg_GetBusVoltage.dlc = 0;

    // auto txMsg_SetAxisRequestedStateCmd = CANFrame{};
    // txMsg_SetAxisRequestedStateCmd.id = axisCanID << 5 | setAxisRequestedStateCmd;
    // txMsg_SetAxisRequestedStateCmd.dlc = 4;
    // uint8_t *ptrToFloat;
    // ptrToFloat = (uint8_t *)&idleState;
    // txMsg_SetAxisRequestedStateCmd.data[0] = ptrToFloat[0];
    // txMsg_SetAxisRequestedStateCmd.data[1] = ptrToFloat[1];
    // txMsg_SetAxisRequestedStateCmd.data[2] = ptrToFloat[2];
    // txMsg_SetAxisRequestedStateCmd.data[3] = ptrToFloat[3];

    // if (mcp2515.sendMessage(&txMsg_SetAxisRequestedStateCmd) != MCP2515::ERROR_OK)
    //     debugLog("Tx Error. Did not send calibration sequence.\n");
    // else
    //     debugLog("Sent calibration sequence.\n");

    // debugLog("dbg_msg:Initial print\n");
    HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"dbg_msg:Initial PRINT\n", 22);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuf, RxBuf_SIZE);
    // __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    CANFrame canFrameRx;
    uint8_t  uartFrameRx[16];
    while (1)
    {
        if (receivedData)
        {
            receivedData = false;
            std::string strRx = "dbg_msg:"; // initialize empty string
            for (const auto& element : RxBuf) {
                strRx += static_cast<char>(element); // convert each element to its ASCII representation and append to string
            }
            strRx+= "\n";
            debugLog(strRx.c_str());
            // debugLog("dbg_msg:Received data!\n");
        }

        if (HAL_UART_Receive_DMA(&huart2, uartFrameRx, 4)  == HAL_OK)
            debugLog("dbg_msg:Received data!\n");
        if (mcp2515.readMessage(&canFrameRx) == MCP2515::ERROR_OK)
            handleCANRx(canFrameRx);

        // if (HAL_UART_Receive(&huart2, uartFrameRx, 10, 1) == HAL_OK)
        // {
        //     std::string strRx = "dbg_msg:"; // initialize empty string
        //     for (const auto& element : uartFrameRx) {
        //         strRx += static_cast<char>(element); // convert each element to its ASCII representation and append to string
        //     }
        //     strRx+= "\n";
        //     debugLog(strRx.c_str());
        // }

        // debugLog("dbg_msg:.");

        // CANFrame txMsg;
        // txMsg.id = 27;
        // txMsg.dlc = 1;
        // txMsg.data[0] = 'a';
        // mcp2515.sendMessage(&txMsg);


        // debugLog("Sent CAN message.\n");
        // HAL_Delay(5);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
