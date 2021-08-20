
#ifndef __G15_CONSTANTS_H__
#define __G15_CONSTANTS_H__

// Instructions
#define iPING 0x01		 // Obtain a status packet
#define iREAD_DATA 0x02	 // Read Control Table values
#define iWRITE_DATA 0x03 // Write Control Table values
#define iREG_WRITE 0x04	 // Write and wait for ACTION instruction
#define iACTION 0x05	 // Triggers REG_WRITE instruction
#define iRESET 0x06		 // Set factory defaults
#define iSYNC_WRITE 0x83 // Simultaneously control multiple actuators

// Device Error codes
#define ALARM_INST 0x40
#define ALARM_OVERLOAD 0x20
#define ALARM_CHECKSUM 0x10
#define ALARM_RANGE 0x08
#define ALARM_OVERHEAT 0x04
#define ALARM_ANGLELIMIT 0x02
#define ALARM_VOLTAGE 0x01

// Broadcast ID
#define BROADCAST_ID 0xFE

// Driver Error codes
#define NO_RESPONSE_ERROR 0xFFFF
#define RECEIVED_LENGTH_ERROR 0x0100
#define RECEIVED_HEADER_ERROR 0x0200
#define RECEIVED_ID_ERROR 0x0400
#define RECEIVED_CHECKSUM_ERROR 0x0800

#define POSITION_MAX_REGISTER 1088
#define POSITION_MAX_DEGREES 360.0

#define SPEED_MAX_REGISTER 1023
#define SPEED_MAX_RPM 112.83

// Control register addresses for G15 parameters
enum
{
	MODEL_NUMBER_L,			// 0x00
	MODEL_NUMBER_H,			// 0x01
	VERSION,				// 0x02
	ID,						// 0x03
	BAUD_RATE,				// 0x04
	RETURN_DELAY_TIME,		// 0x05
	CW_ANGLE_LIMIT_L,		// 0x06
	CW_ANGLE_LIMIT_H,		// 0x07
	CCW_ANGLE_LIMIT_L,		// 0x08
	CCW_ANGLE_LIMIT_H,		// 0x09
	RESERVED1,				// 0x0A
	LIMIT_TEMPERATURE,		// 0x0B
	DOWN_LIMIT_VOLTAGE,		// 0x0C
	UP_LIMIT_VOLTAGE,		// 0x0D
	MAX_TORQUE_L,			// 0x0E
	MAX_TORQUE_H,			// 0x0F
	STATUS_RETURN_LEVEL,	// 0x10
	ALARM_LED,				// 0x11
	ALARM_SHUTDOWN,			// 0x12
	RESERVED2,				// 0x13
	DOWN_CALIBRATION_L,		// 0x14
	DOWN_CALIBRATION_H,		// 0x15
	UP_CALIBRATION_L,		// 0x16
	UP_CALIBRATION_H,		// 0x17
	TORQUE_ENABLE,			// 0x18
	LED,					// 0x19
	CW_COMPLIANCE_MARGIN,	// 0x1A
	CCW_COMPLIANCE_MARGIN,	// 0x1B
	CW_COMPLIANCE_SLOPE,	// 0x1C
	CCW_COMPLIANCE_SLOPE,	// 0x1D
	GOAL_POSITION_L,		// 0x1E
	GOAL_POSITION_H,		// 0x1F
	MOVING_SPEED_L,			// 0x20
	MOVING_SPEED_H,			// 0x21
	TORQUE_LIMIT_L,			// 0x22
	TORQUE_LIMIT_H,			// 0x23
	PRESENT_POSITION_L,		// 0x24
	PRESENT_POSITION_H,		// 0x25
	PRESENT_SPEED_L,		// 0x26
	PRESENT_SPEED_H,		// 0x27
	PRESENT_LOAD_L,			// 0x28
	PRESENT_LOAD_H,			// 0x29
	PRESENT_VOLTAGE,		// 0x2A
	PRESENT_TEMPERATURE,	// 0x2B
	REGISTERED_INSTRUCTION, // 0x2C
	RESERVE3,				// 0x2D
	MOVING,					// 0x2E
	LOCK,					// 0x2F
	PUNCH_L,				// 0x30
	PUNCH_H					// 0x31
};

#endif