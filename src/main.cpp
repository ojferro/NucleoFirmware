#include "stm32f4xx_hal.h"
#include <algorithm>
#include "LQR.h"
#include "Controller.h"
#include "mcp2515.hpp"
#include "mpu6050.hpp"
// #include "EKF.h"
#include "ComplementaryFilter.h"
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

// Define CAN IDs of the Axes. Not to be confused with the encoder SPI CS ports.
#define ODRV_AXIS0_CAN_ID 0x0 // ID of the Left axis
#define ODRV_AXIS1_CAN_ID 0x1 // ID of the Right axis

float enc_pos_0 = 0.0f;
float enc_vel_0 = 0.0f;
float enc_pos_1 = 0.0f;
float enc_vel_1 = 0.0f;

void handleCANRx(CANFrame& rxMsg){
    uint32_t odrvID  = rxMsg.id >> 5;
    uint32_t odrvCmd = rxMsg.id & 0b11111;

    if (odrvCmd == ODrive::AxisCommand::ENCODER_ESTIMATES)
    {
        const auto encoderPos = can_getSignal<float>(rxMsg, 0, 32, true, 1, 0);
        const auto encoderVel = can_getSignal<float>(rxMsg, 4, 32, true, 1, 0);

        if (odrvID == ODRV_AXIS0_CAN_ID)
        {
            // Update state variables
            enc_pos_0 = encoderPos;
            enc_vel_0 = encoderVel;

            // Transmit for visualization
            const float wheelRadius = 0.03f;
            debugLogFmt("enc_pos_0:%.3f\n", encoderPos * M_TWOPI * wheelRadius);
            debugLogFmt("enc_vel_0:%f\n", encoderVel * M_TWOPI * wheelRadius);
        }
        else if (odrvID == ODRV_AXIS1_CAN_ID)
        {
            // Update state variables
            enc_pos_1 = encoderPos;
            enc_vel_1 = encoderVel;

            // Transmit for visualization
            const float wheelRadius = 0.03f;
            debugLogFmt("enc_pos_1:%.3f\n", -encoderPos * M_TWOPI * wheelRadius);
            debugLogFmt("enc_vel_1:%f\n", -encoderVel * M_TWOPI * wheelRadius);
        }
    }

    if (odrvCmd == ODrive::AxisCommand::GET_BUS_VOLTAGE_CURRENT)
    {
        const auto busVoltage  = can_getSignal<float>(rxMsg, 0, 32, true, 1, 0);
        const auto busCurrent  = can_getSignal<float>(rxMsg, 4, 32, true, 1, 0);
        debugLogFmt("bus_voltage:%.3f\n", busVoltage);
        debugLogFmt("bus_current:%.3f\n", busCurrent);
    }

    if (odrvCmd == ODrive::AxisCommand::ODRIVE_HEARTBEAT_MESSAGE)
    {
        const auto axisError = can_getSignal<uint32_t>(rxMsg, 0, 32, true);
        if (axisError != 0)
        {
            if (odrvID == ODRV_AXIS0_CAN_ID)
                debugLogFmt("dbg_msg:Axis0 Err Code: %d\n", axisError);

            if (odrvID == ODRV_AXIS1_CAN_ID)
                debugLogFmt("dbg_msg:Axis1 Err Code: %d\n", axisError);
        }
    }
}

struct CommandHandler{

    CommandHandler(): m_controlMode(ODrive::ControlMode::POSITION_CONTROL){}

    ODrive::ControlMode m_controlMode;

    void handleMasterCmd(const std::string& strRx, ODrive::Axis& axis, bool flipSetpoint = false){
        // Control State
        if (strRx.compare("calib_rtn") == 0)
        {
            axis.clearErrors();
            axis.setRequestedState(ODrive::AxisState::FULL_CALIBRATION_SEQUENCE);

            const auto dbg_msg = "dbg_msg: Sent "+strRx+" command\n";
            debugLog(dbg_msg.c_str());
        }
        else if (strRx.compare("clear_err") == 0)
        {
            axis.clearErrors();

            const auto dbg_msg = "dbg_msg: Sent "+strRx+" command\n";
            debugLog(dbg_msg.c_str());
        }
        else if (strRx.compare("ClLp_ctrl") == 0)
        {
            axis.setRequestedState(ODrive::AxisState::CLOSED_LOOP_CONTROL);

            const auto dbg_msg = "dbg_msg: Sent "+strRx+" command\n";
            debugLog(dbg_msg.c_str());
        }
        else if (strRx.compare("idle_ctrl") == 0)
        {
            axis.setRequestedState(ODrive::AxisState::IDLE);

            const auto dbg_msg = "dbg_msg: Sent "+strRx+" command\n";
            debugLog(dbg_msg.c_str());
        }
        else if (strRx.compare("auto_ctrl") == 0)
        {
            // Initialize wheels to 0
            axis.setRequestedState(ODrive::AxisState::CLOSED_LOOP_CONTROL);
            axis.setControllerModes(ODrive::ControlMode::POSITION_CONTROL);
            axis.setInputPos(0, 0, 0);

            HAL_Delay(1000);
            debugLog("dbg_msg: Zeroing wheels done.\n");

            axis.setRequestedState(ODrive::AxisState::IDLE);

            debugLog("dbg_msg: Starting closed loop torque control.\n");
            HAL_Delay(500);

            m_controlMode = ODrive::ControlMode::TORQUE_CONTROL;
            axis.setControllerModes(m_controlMode);
            axis.setRequestedState(ODrive::AxisState::CLOSED_LOOP_CONTROL);

            axis.setInputTorque(0);

            debugLog("dbg_msg: Torque control active.\n");
        }

        // Control Modes
        else if (strRx.compare("posn_ctrl") == 0)
        {
            m_controlMode = ODrive::ControlMode::POSITION_CONTROL;

            axis.setControllerModes(m_controlMode);
            const auto dbg_msg = "dbg_msg: Sent "+strRx+" command\n";
            debugLog(dbg_msg.c_str());
        }
        else if (strRx.compare("velo_ctrl") == 0)
        {
            m_controlMode = ODrive::ControlMode::VELOCITY_CONTROL;
            axis.setControllerModes(m_controlMode);

            const auto dbg_msg = "dbg_msg: Sent "+strRx+" command\n";
            debugLog(dbg_msg.c_str());
        }
        else if (strRx.compare("torq_ctrl") == 0)
        {
            m_controlMode = ODrive::ControlMode::TORQUE_CONTROL;
            axis.setControllerModes(m_controlMode);

            const auto dbg_msg = "dbg_msg: Sent "+strRx+" command\n";
            debugLog(dbg_msg.c_str());
        }
        else if (strRx.compare("volt_ctrl") == 0)
        {
            m_controlMode = ODrive::ControlMode::VOLTAGE_CONTROL;
            axis.setControllerModes(m_controlMode);

            const auto dbg_msg = "dbg_msg: Sent "+strRx+" command\n";
            debugLog(dbg_msg.c_str());
        }
        
        // Setpoint
        else if (strRx.find("sp:")!=std::string::npos)
        {
            const auto sp = strRx.substr(3);
            auto setpoint = std::stof(sp);
            setpoint = flipSetpoint ? -setpoint : setpoint;

            if (m_controlMode == ODrive::ControlMode::POSITION_CONTROL)
            {
                debugLogFmt("dbg_msg: Setting to POSITION_CONTROL\n");
                axis.setInputPos(setpoint, 0, 0);
            }
            else if (m_controlMode == ODrive::ControlMode::VELOCITY_CONTROL)
            {
                debugLogFmt("dbg_msg: Setting to VELOCITY_CONTROL\n");
                axis.setInputVel(setpoint, 0.0f);
            }
            else if (m_controlMode == ODrive::ControlMode::TORQUE_CONTROL)
            {
                debugLogFmt("dbg_msg: Setting to TORQUE_CONTROL\n");
                axis.setInputTorque(setpoint);
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

template <typename T>
T clamp(const T& value, const T& lower, const T& upper) {
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}


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
    while (MPU6050_Init(&hi2c1) == 1)
    {
        debugLog("dbg_msg:MPU6050 offline\n");
        HAL_Delay(200);
    }
    debugLog("dbg_msg:MPU6050 online\n");

    // auto mpu6050Data = MPU6050_t{};

    debugLog("dbg_msg:STM32 online\n");

    // Initialize MCP2515
    auto mcp2515 = MCP2515(&hspi1);

    MCP2515::ERROR _e;
    _e = mcp2515.reset();
    _e = mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    _e = mcp2515.setNormalMode();

    if (_e != MCP2515::ERROR_OK)
    {
        debugLog("Error!\n");
    }
    debugLog("dbg_msg:MCP2515 online\n");

    // Set up safely, to be idle and in position control mode
    ODrive::Axis axis0(ODRV_AXIS0_CAN_ID, mcp2515);
    axis0.setRequestedState(ODrive::AxisState::IDLE);
    axis0.setControllerModes(ODrive::ControlMode::POSITION_CONTROL);
    axis0.getBusVoltageCurrent();
    axis0.setLimits(200.0f, 10.0f);
    axis0.clearErrors();
    axis0.setPositionGain(20.0f);
    axis0.setVelGains(0.05f, 0.001f);

    ODrive::Axis axis1(ODRV_AXIS1_CAN_ID, mcp2515);
    axis1.setRequestedState(ODrive::AxisState::IDLE);
    axis1.setControllerModes(ODrive::ControlMode::POSITION_CONTROL);
    axis1.getBusVoltageCurrent();
    axis1.setLimits(200.0f, 10.0f);
    axis1.clearErrors();
    axis1.setPositionGain(20.0f);
    axis1.setVelGains(0.05f, 0.001f);

    debugLog("dbg_msg:ODrive online\n");

    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuf, RxBuf_SIZE);
    CANFrame canFrameRx;

    // const auto ekfConfig = EKFConfig{};
    // auto qEst = Quaternion{};
    // auto filteredSensorData = FilteredSensorData{};
    // auto eulerEst = Euler{};
    // auto ekf = EKF(&hi2c1, ekfConfig);

    auto cf = ComplementaryFilter(&hi2c1);

    // Control
    auto controller = LQR();
    Controller::U u = {0.0f, 0.0f};
    Controller::State state = {0.0f, 0.0f, 0.0f, 0.0f};

    auto cmdHandler = CommandHandler();
    while (1)
    {
        if (receivedData)
        {
            receivedData = false;
            std::string strRx = "";
            for (const auto& element : RxBuf) {
                // Convert each element to its ASCII representation and append to string
                strRx += static_cast<char>(element);
            }
            
            cmdHandler.handleMasterCmd(strRx, axis0);
            cmdHandler.handleMasterCmd(strRx, axis1, true);

            // axis0.getBusVoltageCurrent();
            // axis1.getBusVoltageCurrent();
        }

        if (mcp2515.readMessage(&canFrameRx) == MCP2515::ERROR_OK)
        {
            handleCANRx(canFrameRx);
        }


        // ekf.StepEKFLoop(qEst, filteredSensorData);
        // ekf.QuaternionToEuler(qEst, eulerEst);

        cf.Step();
        // debugLogFmt("imu_r:%.6f\n", (eulerEst.roll - 1.488) * 180.0f / M_PI);
        debugLogFmt("imu_r:%.6f\n", 0.0f);
        debugLogFmt("imu_p:%.6f\n", cf.GetPitchAngle() * 180.0f / M_PI);
        debugLogFmt("imu_y:%.6f\n", 0.0f);

        // Run control loop

        if (cmdHandler.m_controlMode == ODrive::ControlMode::TORQUE_CONTROL)
        {
            // Populate state
            const auto wheelRadius = 0.03f; // meters
            state[Controller::StateIndex::X] = enc_pos_0 * M_TWOPI * wheelRadius;
            state[Controller::StateIndex::THETA] = cf.GetPitchAngle();
            state[Controller::StateIndex::X_DOT] = enc_vel_0 * M_TWOPI * wheelRadius;
            state[Controller::StateIndex::THETA_DOT] = cf.GetPitchAngleDot();

            // Failsafe in the event of wheels lifting (nothing to counteract torque)
            // or robot falling over
            if (abs(state[Controller::StateIndex::THETA] * 180.0 * M_1_PI) > 45.0f)
            {
                axis0.setInputTorque(0);
                axis1.setInputTorque(0);

                debugLog("dbg_msg:Robot is falling over. Stopping motors.\n");
                continue;
            }
            else if (abs(state[Controller::StateIndex::X_DOT]) > 5.0f)
            {
                axis0.setInputTorque(0);
                axis1.setInputTorque(0);

                debugLog("dbg_msg:Max vel limit. Stopping motors.\n");
                continue;
            }

            // Run controller
            controller.iterate(u, state);

            // Send the control signal to the ODrive
            const auto maxTorque = 0.05f;
            axis0.setInputTorque(clamp(u[0], -maxTorque, maxTorque));
            axis1.setInputTorque(clamp(-u[1], -maxTorque, maxTorque));

            // Transmit data for visualization
            debugLogFmt("ctrl_u_0:%.6f\n", clamp(u[0], -maxTorque, maxTorque));
        }
        

        // debugLogFmt("imu_w:%.3f\n", qEst.w);
        // debugLogFmt("imu_x:%.3f\n", qEst.x);
        // debugLogFmt("imu_y:%.3f\n", qEst.y);
        // debugLogFmt("imu_z:%.3f\n", qEst.z);

        // debugLogFmt("dbg_msg:quaternion,%.3f,%.3f,%.3f,%.3f,\n", qEst.w, qEst.x, qEst.y, qEst.z);


        // MPU6050_Read_All(&hi2c1, &mpu6050Data);
        // debugLogFmt("g_x:%.3f, g_y:%.3f, g_z:%.3f\n", mpu6050Data.Gx, mpu6050Data.Gy, mpu6050Data.Gz);
        // debugLogFmt("a_x:%.3f, a_y:%.3f, a_z:%.3f\n", mpu6050Data.Ax, mpu6050Data.Ay, mpu6050Data.Az);

        // debugLogFmt("acc_x,%.16f\n", mpu6050Data.Ax);
        // debugLogFmt("acc_y,%.16f\n", mpu6050Data.Ay);
        // debugLogFmt("acc_z,%.16f\n", mpu6050Data.Az);
        // debugLogFmt("gyr_x,%.16f\n", mpu6050Data.Gx);
        // debugLogFmt("gyr_y,%.16f\n", mpu6050Data.Gy);
        // debugLogFmt("gyr_z,%.16f\n", mpu6050Data.Gz);
        // debugLogFmt("K_x:%.3f, K_y:%.3f\n", mpu6050Data.KalmanAngleX, mpu6050Data.KalmanAngleY);

        // HAL_Delay(100);
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
