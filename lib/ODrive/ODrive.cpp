#include "ODrive.hpp"
#include "mcp2515.hpp"

CANFrame ODrive::Axis::odriveCANFrame(AxisCommand commandID, uint8_t dataLength, bool isExt, bool isRTR){
	CANFrame canFrame{
		(axisID << 5) | commandID,
		dataLength,
		{0,0,0,0,0,0,0,0}
	};

	if (isExt)
		canFrame.id |= CAN_EFF_FLAG;
	if (isRTR)
		canFrame.id |= CAN_RTR_FLAG;

	return canFrame;
}

void ODrive::Axis::setRequestedState(AxisState state){
	auto canFrame = odriveCANFrame(AxisCommand::SET_AXIS_REQUESTED_STATE, 4);
	uint8_t requestedState = state;
	uint8_t *ptrToFloat;

	ptrToFloat = (uint8_t *)&requestedState;
	canFrame.data[0] = ptrToFloat[0];
	canFrame.data[1] = ptrToFloat[1];
	canFrame.data[2] = ptrToFloat[2];
	canFrame.data[3] = ptrToFloat[3];

	m_mcp2515.sendMessage(&canFrame);
}

void ODrive::Axis::setControllerModes(ControlMode controlMode, InputMode inputMode){
	auto canFrame = odriveCANFrame(AxisCommand::SET_CONTROLLER_MODES, 8);

	const uint8_t *ptrControl = (uint8_t *)&controlMode;
	const uint8_t *ptrInput = (uint8_t *)&inputMode;

	canFrame.data[0] = ptrControl[0];
	canFrame.data[1] = ptrControl[1];
	canFrame.data[2] = ptrControl[2];
	canFrame.data[3] = ptrControl[3];
	canFrame.data[4] = ptrInput[0];
	canFrame.data[5] = ptrInput[1];
	canFrame.data[6] = ptrInput[2];
	canFrame.data[7] = ptrInput[3];
	
	m_mcp2515.sendMessage(&canFrame);
}

void ODrive::Axis::setInputPos(float inputPos, int velFF, int torqueFF){
	auto canFrame = odriveCANFrame(AxisCommand::SET_INPUT_POS, 8);
	uint8_t *ptrPos = (uint8_t *)&inputPos;
	uint8_t *ptrVel = (uint8_t *)&velFF;
	uint8_t *ptrTor = (uint8_t *)&torqueFF;

	canFrame.data[0] = ptrPos[0];
	canFrame.data[1] = ptrPos[1];
	canFrame.data[2] = ptrPos[2];
	canFrame.data[3] = ptrPos[3];

	canFrame.data[4] = ptrVel[0];
	canFrame.data[5] = ptrVel[1];

	canFrame.data[6] = ptrTor[0];
	canFrame.data[7] = ptrTor[1];

	m_mcp2515.sendMessage(&canFrame);
}

void ODrive::Axis::setInputVel(float vel, float torqueFF){
	auto canFrame = odriveCANFrame(AxisCommand::SET_INPUT_VEL, 8);

	uint8_t *ptrVel = (uint8_t *)&vel;
	uint8_t *ptrTor = (uint8_t *)&torqueFF;

	canFrame.data[0] = ptrVel[0];
	canFrame.data[1] = ptrVel[1];
	canFrame.data[2] = ptrVel[2];
	canFrame.data[3] = ptrVel[3];

	canFrame.data[4] = ptrTor[0];
	canFrame.data[5] = ptrTor[1];
	canFrame.data[6] = ptrTor[2];
	canFrame.data[7] = ptrTor[3];

	m_mcp2515.sendMessage(&canFrame);
}

void ODrive::Axis::setInputTorque(float torque){
	auto canFrame = odriveCANFrame(AxisCommand::SET_INPUT_TORQUE, 4);

	uint8_t *ptrTor = (uint8_t *)&torque;
	canFrame.data[0] = ptrTor[0];
	canFrame.data[1] = ptrTor[1];
	canFrame.data[2] = ptrTor[2];
	canFrame.data[3] = ptrTor[3];

	m_mcp2515.sendMessage(&canFrame);
}

void ODrive::Axis::setPositionGain(float posGain){
	auto canFrame = odriveCANFrame(AxisCommand::SET_POSITION_GAIN, 4);

	uint8_t *ptrPos = (uint8_t *)&posGain;

	canFrame.data[0] = ptrPos[0];
	canFrame.data[1] = ptrPos[1];
	canFrame.data[2] = ptrPos[2];
	canFrame.data[3] = ptrPos[3];

	m_mcp2515.sendMessage(&canFrame);
}

void ODrive::Axis::setVelGains(float velGain, float velIntGain){
	auto canFrame = odriveCANFrame(AxisCommand::SET_VEL_GAINS, 8);

	uint8_t *ptrVelGain = (uint8_t *)&velGain;
	uint8_t *ptrVelIntGain = (uint8_t *)&velIntGain;

	canFrame.data[0] = ptrVelGain[0];
	canFrame.data[1] = ptrVelGain[1];
	canFrame.data[2] = ptrVelGain[2];
	canFrame.data[3] = ptrVelGain[3];

	canFrame.data[4] = ptrVelIntGain[0];
	canFrame.data[5] = ptrVelIntGain[1];
	canFrame.data[6] = ptrVelIntGain[2];
	canFrame.data[7] = ptrVelIntGain[3];

	m_mcp2515.sendMessage(&canFrame);
}
void ODrive::Axis::setNodeID(uint32_t nodeID){
	auto canFrame = odriveCANFrame(AxisCommand::SET_AXIS_NODE_ID, 4);

	uint8_t *ptrNodeId = (uint8_t *)&nodeID;

	canFrame.data[0] = ptrNodeId[0];
	canFrame.data[1] = ptrNodeId[1];
	canFrame.data[2] = ptrNodeId[2];
	canFrame.data[3] = ptrNodeId[3];
	
	m_mcp2515.sendMessage(&canFrame);
}

void ODrive::Axis::setLimits(float velLim, float currLim){
	auto canFrame = odriveCANFrame(AxisCommand::SET_LIMITS, 8);

	uint8_t *ptrVelLim = (uint8_t *)&velLim;
	uint8_t *ptrCurrLim = (uint8_t *)&currLim;

	canFrame.data[0] = ptrVelLim[0];
	canFrame.data[1] = ptrVelLim[1];
	canFrame.data[2] = ptrVelLim[2];
	canFrame.data[3] = ptrVelLim[3];

	canFrame.data[4] = ptrCurrLim[0];
	canFrame.data[5] = ptrCurrLim[1];
	canFrame.data[6] = ptrCurrLim[2];
	canFrame.data[7] = ptrCurrLim[3];

	m_mcp2515.sendMessage(&canFrame);
}

void ODrive::Axis::getEncoderCount(){
	auto canFrame = odriveCANFrame(AxisCommand::GET_ENCODER_COUNT, 0, false, true);
	m_mcp2515.sendMessage(&canFrame);
}

void ODrive::Axis::getBusVoltageCurrent(){
	auto canFrame = odriveCANFrame(AxisCommand::GET_BUS_VOLTAGE_CURRENT, 0, false, true);
	m_mcp2515.sendMessage(&canFrame);
}

void ODrive::Axis::getIQ(){
	auto canFrame = odriveCANFrame(AxisCommand::GET_IQ, 0, false, true);
	m_mcp2515.sendMessage(&canFrame);
}

void ODrive::Axis::clearErrors(){
	auto canFrame = odriveCANFrame(AxisCommand::CLEAR_ERRORS, 0);
	m_mcp2515.sendMessage(&canFrame);
}


void ODrive::Axis::rebootODrive(){
	auto canFrame = odriveCANFrame(AxisCommand::REBOOT_ODRIVE, 0);
	m_mcp2515.sendMessage(&canFrame);
}