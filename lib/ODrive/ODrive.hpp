#ifndef ODRIVE_ODRIVE_H_
#define ODRIVE_ODRIVE_H_

#include <stdint.h>
#include "CANHelpers.h"
#include "can.hpp"
#include "mcp2515.hpp"

struct ODrive {

	enum AxisCommand : uint8_t {
		ODRIVE_HEARTBEAT_MESSAGE = 0x001,

		// TODO: Add the rest of the commands, like EStop, etc.
		SET_AXIS_NODE_ID		 = 0x006,
		SET_AXIS_REQUESTED_STATE = 0x007,
		ENCODER_ESTIMATES		 = 0x009,
		GET_ENCODER_COUNT		 = 0x00A,
		SET_CONTROLLER_MODES	 = 0x00B,
		SET_INPUT_POS			 = 0x00C,
		SET_INPUT_VEL			 = 0x00D,
		SET_INPUT_TORQUE		 = 0x00E,
		SET_LIMITS				 = 0x00F,
		GET_IQ					 = 0x014,
		REBOOT_ODRIVE			 = 0x016,
		GET_BUS_VOLTAGE_CURRENT	 = 0x017,
		CLEAR_ERRORS			 = 0x018,
		SET_POSITION_GAIN		 = 0x01A,
		SET_VEL_GAINS			 = 0x01B
	};

	enum AxisState : uint8_t {
		UNDEFINED						  = 0x0,
		IDLE							  = 0x1,
		STARTUP_SEQUENCE				  = 0x2,
		FULL_CALIBRATION_SEQUENCE		  = 0x3,
		MOTOR_CALIBRATION				  = 0x4,
		ENCODER_INDEX_SEARCH			  = 0x6,
		ENCODER_OFFSET_CALIBRATION		  = 0x7,
		CLOSED_LOOP_CONTROL				  = 0x8,
		LOCKIN_SPIN						  = 0x9,
		ENCODER_DIR_FIND				  = 0xA,
		HOMING							  = 0xB,
		ENCODER_HALL_POLARITY_CALIBRATION = 0xC,
		ENCODER_HALL_PHASE_CALIBRATION	  = 0xD
	};

	enum ControlMode : uint8_t {
		VOLTAGE_CONTROL  = 0x0,
		TORQUE_CONTROL   = 0x1,
		VELOCITY_CONTROL = 0x2,
		POSITION_CONTROL = 0x3
	};

	enum InputMode : uint8_t {
		INACTIVE     = 0x0,
		PASSTHROUGH  = 0x1,
		VEL_RAMP     = 0x2,
		POS_FILTER   = 0x3,
		MIX_CHANNELS = 0x4,
		TRAP_TRAJ    = 0x5,
		TORQUE_RAMP  = 0x6,
		MIRROR       = 0x7,
		TUNING       = 0x8
	};

	//Axis Parameters
	class Axis
	{
		public:
			Axis(uint16_t axisID, MCP2515& mcp2515): axisID(axisID), m_mcp2515(mcp2515) {}

			uint32_t axisID;
			float 	 axisEncoderPos;
			float 	 axisEncoderVel;
			int32_t	 axisEncoderCPR;
			int32_t	 axisEncoderShadow;
			float	 axisBusVoltage;
			float	 axisBusCurrent;
			float	 axisIqSetpoint;
			float	 axisIqMeasured;
			uint32_t axisError;
			uint8_t	 axisCurrentState;
			uint8_t	 controllerStatus;

			void setRequestedState(AxisState state);

			// Set the Control Mode and Input Mode
			void setControllerModes(ControlMode controlMode, InputMode inputMode = InputMode::PASSTHROUGH);

			// Set the desired position of the axis as well as the feed-forward velocity
			// and feed-forward torque
			void setInputPos(float inputPos, int velFF, int torqueFF);

			// Sends a CAN Data Packet with the required Axis ID and Command ID to set the desired
			// velocity of the axis and the the feed-forward torque of the torque controller
			void setInputVel(float vel, float torqueFF);

			//Set the desired torque of the axis.
			void setInputTorque(float torque);

			// Set the gain for the position controller
			void setPositionGain(float posGain);

			void setVelGains(float velGain, float velIntGain);

			// Update the given axis' Node Id
			void setNodeID(uint32_t nodeID);

			// Set velocity and current limits
			void setLimits(float velLim, float currLim);

			// Sends a CAN RTR Frame with required ID to the ODrive to request Encoder Shadow Count
			// and Encoder Count in CPR.
			// Note: Function only sends the RTR frame. Data will be received and variables will be updated
			// via the CallBack function when an reception interrupt is triggered.
			void getEncoderCount();

			// Sends a CAN RTR Frame with required ID to the ODrive to request Bus Voltage and Bus Current.
			// Note: Function only sends the RTR frame. Data will be received and variables will be updated
			// via the CallBack function when an reception interrupt is triggered.
			void getBusVoltageCurrent();

			// Sends a CAN RTR Frame with required ID to the ODrive to request Iq Setpoint and Iq Measured.
			// Note: Function only sends the RTR frame. Data will be received and variables will be updated
			// via the CallBack function when an reception interrupt is triggered.
			void getIQ();

			// Clear all errors of this Axis including contained submodules
			void clearErrors();

			// Sends a CAN Data Packet with the required Axis ID and Command ID to reboot
			// the controller without saving the current configuraiton.
			void rebootODrive();

		// private:
			MCP2515 m_mcp2515;

			//Used to set the CAN TX Struct Parameters such as datalength, frametype, idtype, ID
			CANFrame odriveCANFrame(AxisCommand cmd, uint8_t dataLength, bool isExt = false, bool isRTR = false);
	};
};

#endif /* ODRIVE_ODRIVE_H_ */