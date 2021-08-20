#ifndef __G15_SERVO_H__
#define __G15_SERVO_H__

#include <stdint.h>

#include "G15_Constants.h"
#include "G15_Error_Handler.h"

#define ConvertAngleToPos(angle) (uint16_t)((uint16_t)(angle)*1088UL / 360UL)
#define ConvertPosToAngle(position) (float)(position*360.0f / 1088.0f)
#define ConvertTime(time) (uint16_t)(time * 10UL)
#define ConvertRPMToVal(speed) (uint16_t)( (float)(speed/112.83) * 1023UL )
#define ConvertValToRPM(val) (float)((val/1023.0f) * 112.83f)

#define SerialTimeOut 100L

#define CW 1
#define CCW 0
#define ON 1
#define OFF 0

class G15_HAL;

class G15_Servo
{
public:
	G15_Servo(G15_HAL&, G15_Error_Handler* error_handler = nullptr, bool check_response_packet = true);
	~G15_Servo();

	void begin(uint32_t baudrate, uint32_t timeout=SerialTimeOut);
	void end(void);

	//*=========Wheel Mode=====================================================================================
	//360 degree continous rotation. change CW and CCW Angle Limits to same value
	uint16_t setWheelMode(uint8_t servoID);
	uint16_t exitWheelMode(uint8_t servoID);
	uint16_t setWheelSpeed(uint8_t servoID, uint16_t speed, uint8_t direction);

	//*=========Normal Positioning Mode========================================================================
	//(Rotation limited by Angle Limit and Direction of Rotation determined by operation section of Angle Limit)
	uint16_t setPos(uint8_t servoID, uint16_t position);
	uint16_t setPosAngle(uint8_t servoID, uint16_t angle);
	uint16_t setPosSpeed(uint8_t servoID, uint16_t position, uint16_t speed);

	//*========Direction Positioning Mode======================================================================
	//(Rotation direction and angle is NOT limited by Angle Limit Control Register value)
	uint16_t rotateCW(uint8_t servoID, uint16_t position);
	uint16_t rotateCCW(uint8_t servoID, uint16_t position);

	//*=======Torque Enable and Speed Control==================================================================
	uint16_t setTorqueOnOff(uint8_t servoID, uint8_t onOff);
	uint16_t setSpeed(uint8_t servoID, uint16_t speed);
	uint16_t setTimeToGoal(uint8_t servoID, uint16_t time);

	//*=======Set Maximum Limits===============================================================================
	uint16_t setAngleLimit(uint8_t servoID, uint16_t cwAngle, uint16_t ccwAngle);
	uint16_t setTorqueLimit(uint8_t servoID, uint16_t torqueLimit); //in RAM area
	uint16_t setTemperatureLimit(uint8_t servoID, uint8_t temperature);
	uint16_t setVoltageLimit(uint8_t servoID, uint8_t voltageLow, uint8_t voltageHigh);

	uint16_t setID(uint8_t servoID, uint8_t newID);

	uint16_t setLED(uint8_t servoID, uint8_t onOff);
	uint16_t setAlarmLED(uint8_t servoID, uint8_t alarmLED);
	uint16_t setAlarmShutDown(uint8_t servoID, uint8_t alarm);

	//*========Servo Positioning Control Parameters============================================================
	uint16_t setMarginSlopePunch(uint8_t servoID, uint8_t CWMargin, uint8_t CCWMargin, uint8_t CWSlope, uint8_t CCWSlope, uint16_t punch);

	uint16_t setBaudRate(uint8_t servoID, uint32_t baudrate);
	uint16_t factoryReset(uint8_t servoID);
	uint16_t ping(uint8_t servoID, uint8_t *data);

	uint16_t getPos(uint8_t servoID, uint8_t *data);
	uint16_t getSpeed(uint8_t servoID, uint8_t *data);
	uint16_t getLoad(uint8_t servoID, uint8_t *data);
	uint16_t getVoltage(uint8_t servoID, uint8_t *data);
	uint16_t getTemperature(uint8_t servoID, uint8_t *data);
	uint16_t getTorqueOnOff(uint8_t servoID, uint8_t *data);
	uint16_t isMoving(uint8_t servoID, uint8_t *data);

	uint16_t writeInstruction(uint8_t servoID, uint8_t address, uint8_t data[], uint8_t length);
	uint16_t readInstruction(uint8_t servoID, uint8_t address, uint8_t data[], uint8_t length);
	
	uint16_t regWriteInstruction(uint8_t servoID, uint8_t address, uint8_t data[], uint8_t length);
	uint16_t actionInstruction(uint8_t servoID = BROADCAST_ID);

	void syncWrite(uint8_t address, uint8_t data[], uint8_t data_length);

	uint16_t convertToRegisterValue(double inputQuantity, double maxInputQuantity, uint16_t maxRegisterValue);
	double convertFromRegisterValue(uint16_t registerValue, uint16_t maxRegisterValue, double maxOutputQuanity);

protected:
	uint16_t sendPacket(uint8_t servoID, uint8_t instruction, uint8_t *data, uint8_t parameterLength);

	void _sendInstruction(uint8_t servoID, uint8_t instruction, uint8_t address, uint8_t data[], uint8_t length);
	uint16_t _readResponse(uint8_t servoID, uint8_t data[] = nullptr, uint8_t length = 0);

	uint16_t _checkAndReadResponse(uint8_t servoID);

	G15_HAL& hal_;

	G15_Error_Handler* error_handler_;
	bool own_error_handler_;

	bool check_response_packet_;
};

#endif