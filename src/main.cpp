#include "main.h"
#include "mcp2515.h"
#include "string.h"
#include "usart.h"
#include "can_helpers.h"
#include <stdio.h>
#include <math.h>
#include <cstring>
#include <cmath>

SPI_HandleTypeDef hspi1;

#define SPI_GPIO_PORT GPIOB;
#define SPI_CS_PIN GPIO_PIN_6;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    // Initialize all configured peripherals
    MX_GPIO_Init();
    MX_SPI1_Init();
    USART2_Init();

    auto mcp2515 = MCP2515(&hspi1);

    MCP2515::ERROR _e;
    _e = mcp2515.reset();
    _e = mcp2515.setBitrate(CAN_250KBPS, MCP_8MHZ);
    _e = mcp2515.setLoopbackMode();
    // _e = mcp2515.setNormalMode();

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

    // const auto axisCanID = uint32_t{0x3};

    // // Request Bus Voltage
    // auto txMsg_GetBusVoltage = can_frame{};
    // txMsg_GetBusVoltage.can_id = axisCanID << 5 | requestBusVoltageCmd | CAN_RTR_FLAG;
    // txMsg_GetBusVoltage.can_dlc = 0;

    // auto txMsg_SetAxisRequestedStateCmd = can_frame{};
    // txMsg_SetAxisRequestedStateCmd.can_id = axisCanID << 5 | setAxisRequestedStateCmd;
    // txMsg_SetAxisRequestedStateCmd.can_dlc = 4;
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

    while (1)
    {
        can_frame rxMsg;
        if (mcp2515.readMessage(&rxMsg) == MCP2515::ERROR_OK)
        {
            uint8_t dlc;
            uint32_t id;
            id = rxMsg.can_id;
            dlc = rxMsg.can_dlc;

            debugLog("Received CAN message.\n");
            debugLogFmt("id: %d, dlc: %d, data[0]: %d \n", id, dlc, rxMsg.data[0]);

            // uint8_t odrv_id, odrv_cmd;
            // odrv_id = id >> 5;
            // odrv_cmd = id & 0b11111;

            // if (odrv_id == 3 && odrv_cmd == 0x09)
            // {
            //     can_Message_t rxMsgOdrv;
            //     rxMsgOdrv.id = odrv_id;
            //     rxMsgOdrv.len = dlc;

            //     std::memcpy(rxMsgOdrv.buf, rxMsg.data, dlc);
            //     const auto encoderPos = can_getSignal<float>(rxMsgOdrv, 0, 32, true, 1, 0);
            //     const auto encoderVel = can_getSignal<float>(rxMsgOdrv, 4, 32, true, 1, 0);

            //     debugLogFmt("encoder_position:%.3f\n", encoderPos);
            //     debugLogFmt("encoder_velocity:%f\n", encoderVel);

            //     mcp2515.sendMessage(&txMsg_GetBusVoltage);
            // }

            // if (odrv_id == 3 && odrv_cmd == 0x017)
            // {
            //     can_Message_t rxMsgOdrv;
            //     rxMsgOdrv.id = odrv_id;
            //     rxMsgOdrv.len = dlc;

            //     std::memcpy(rxMsgOdrv.buf, rxMsg.data, dlc);
            //     const auto busVoltage = can_getSignal<float>(rxMsgOdrv, 0, 32, true, 1, 0);

            //     debugLogFmt("bus_voltage:%.3f\n", busVoltage);
            // }
        }

        can_frame txMsg;
        txMsg.can_id = 27;
        txMsg.can_dlc = 1;
        txMsg.data[0] = 'a';
        mcp2515.sendMessage(&txMsg);

        debugLog("Sent CAN message.\n");
        HAL_Delay(100);
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

static void MX_SPI1_Init(void)
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
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {}
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
