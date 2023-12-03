#include "stm32f4xx_hal.h"
#include "mcp2515.hpp"
#include "mpu6050.hpp"
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
        // TODO: There's something wrong with the scaling in getSignal I think....
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

struct CommandHandler{

    CommandHandler(): m_controlMode(ODrive::ControlMode::POSITION_CONTROL){}

    ODrive::ControlMode m_controlMode;

    void handleMasterCmd(const std::string& strRx, ODrive::Axis& axis0){
        // Control State
        if (strRx.compare("calib_rtn") == 0)
        {
            axis0.clearErrors();
            axis0.setRequestedState(ODrive::AxisState::FULL_CALIBRATION_SEQUENCE);

            const auto dbg_msg = "dbg_msg: Sent "+strRx+" command\n";
            debugLog(dbg_msg.c_str());
        }
        else if (strRx.compare("ClLp_ctrl") == 0)
        {
            axis0.setRequestedState(ODrive::AxisState::CLOSED_LOOP_CONTROL);

            const auto dbg_msg = "dbg_msg: Sent "+strRx+" command\n";
            debugLog(dbg_msg.c_str());
        }
        else if (strRx.compare("idle_ctrl") == 0)
        {
            axis0.setRequestedState(ODrive::AxisState::IDLE);

            const auto dbg_msg = "dbg_msg: Sent "+strRx+" command\n";
            debugLog(dbg_msg.c_str());
        }

        // Control Modes
        else if (strRx.compare("posn_ctrl") == 0)
        {
            m_controlMode = ODrive::ControlMode::POSITION_CONTROL;

            axis0.setControllerModes(m_controlMode);
            const auto dbg_msg = "dbg_msg: Sent "+strRx+" command\n";
            debugLog(dbg_msg.c_str());
        }
        else if (strRx.compare("velo_ctrl") == 0)
        {
            m_controlMode = ODrive::ControlMode::VELOCITY_CONTROL;
            axis0.setControllerModes(m_controlMode);

            const auto dbg_msg = "dbg_msg: Sent "+strRx+" command\n";
            debugLog(dbg_msg.c_str());
        }
        else if (strRx.compare("torq_ctrl") == 0)
        {
            m_controlMode = ODrive::ControlMode::TORQUE_CONTROL;
            axis0.setControllerModes(m_controlMode);

            const auto dbg_msg = "dbg_msg: Sent "+strRx+" command\n";
            debugLog(dbg_msg.c_str());
        }
        else if (strRx.compare("volt_ctrl") == 0)
        {
            m_controlMode = ODrive::ControlMode::VOLTAGE_CONTROL;
            axis0.setControllerModes(m_controlMode);

            const auto dbg_msg = "dbg_msg: Sent "+strRx+" command\n";
            debugLog(dbg_msg.c_str());
        }

        else if (strRx.find("sp:")!=std::string::npos)
        {
            const auto sp = strRx.substr(3);
            const auto setpoint = std::stof(sp);

            if (m_controlMode == ODrive::ControlMode::POSITION_CONTROL)
            {
                axis0.setInputPos(setpoint, 0, 0);
            }
            else if (m_controlMode == ODrive::ControlMode::VELOCITY_CONTROL)
            {
                axis0.setInputVel(setpoint, 0.0f);
            }
            else if (m_controlMode == ODrive::ControlMode::TORQUE_CONTROL)
            {
                axis0.setInputTorque(setpoint);
            }
            else if (m_controlMode == ODrive::ControlMode::VOLTAGE_CONTROL)
            {
                debugLogFmt("dbg_msg: Voltage control not implemented yet\n");
            }

            debugLogFmt("dbg_msg: Setpoint %f command received\n", setpoint);
        }
        
        else
        {
            std::string feedback = "dbg_msg:Unknown cmd " + strRx + "\n";
            debugLog(feedback.c_str());
        }
    }
};


// TODO: Remove this ugly global function, needed for DMA right now.

#define RxBuf_SIZE 9 // TODO: TOMORROW!!! This goes crazy when you increase it past 16...??? Investigate what's happening
                      // Figured it out. Sort of. This needs to be EXACTLY the size of the msg being sent by the publisher
                      // i.e. if this is bigger, wonky. if this is smaller, wonky.
                      // Look into this https://github1s.com/MaJerle/stm32-usart-uart-dma-rx-tx/blob/main/projects/usart_rx_idle_line_irq_F4/Src/main.c#L83
                      // I think that's what he's doing with the circular buffer, keeping track of which was the last byte he read, and wrapping around.
#define MainBuf_SIZE 18

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

    // Ensure MPU6050 is up and running
    debugLog("dbg_msg:Initializing MPU6050\n");
    while (MPU6050_Init(&hi2c1) == 1);
    debugLog("dbg_msg:MPU6050 online\n");

    auto mpu6050Data = MPU6050_t{};

    // Initialize MCP2515
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
    debugLog("dbg_msg:MCP2515 online\n");

    // Request Bus Voltage
    ODrive::Axis axis0(0x3, mcp2515);
    axis0.setRequestedState(ODrive::AxisState::IDLE);
    axis0.setControllerModes(ODrive::ControlMode::POSITION_CONTROL);
    axis0.setLimits(20.0f, 10.0f);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuf, RxBuf_SIZE);
    CANFrame canFrameRx;

    debugLog("dbg_msg:STM32 online\n");

    auto cmdHandler = CommandHandler();
    while (1)
    {
        if (receivedData)
        {
            receivedData = false;
            std::string strRx = "";
            for (const auto& element : RxBuf) {
                // convert each element to its ASCII representation and append to string
                strRx += static_cast<char>(element);
            }
            
            cmdHandler.handleMasterCmd(strRx, axis0);           
        }

        if (mcp2515.readMessage(&canFrameRx) == MCP2515::ERROR_OK)
        {
            handleCANRx(canFrameRx);
            // axis0.getBusVoltageCurrent();
        }

        MPU6050_Read_All(&hi2c1, &mpu6050Data);
        // debugLogFmt("g_x:%.3f, g_y:%.3f, g_z:%.3f\n", mpu6050Data.Gx, mpu6050Data.Gy, mpu6050Data.Gz);
        // debugLogFmt("a_x:%.3f, a_y:%.3f, a_z:%.3f\n", mpu6050Data.Ax, mpu6050Data.Ay, mpu6050Data.Az);
        debugLogFmt("K_x:%.3f, K_y:%.3f\n", mpu6050Data.KalmanAngleX, mpu6050Data.KalmanAngleY);

        // debugLog("dbg_msg:Hello World\n");
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
