#include "G15_Servo.h"
#include "G15_HAL.h"
#include "G15_Constants.h"

G15_Servo::G15_Servo(G15_HAL& hal, G15_Error_Handler* error_handler, bool check_response_packet)
: hal_(hal),
  error_handler_(error_handler),
  own_error_handler_(false),
  check_response_packet_(check_response_packet)
{ 
	if (!error_handler_)
	{
		own_error_handler_ = true;
		error_handler_ = new G15_Error_Handler();
	}
}

G15_Servo::~G15_Servo()
{
	if (own_error_handler_)
	{
		delete error_handler_;
	}
}

void G15_Servo::begin(uint32_t baudrate, uint32_t timeout)
{
	hal_.begin(baudrate, timeout);
}

void G15_Servo::end(void)
{
	hal_.end();
}

// Send packet
// Caution: At least 2 bytes of data array need to be passed into the function
uint16_t G15_Servo::sendPacket(uint8_t servoID, uint8_t instruction, uint8_t *data, uint8_t parameterLength)
{
	uint8_t readCount = 0;
	uint8_t i;
	uint8_t packetLength = 0;
	uint8_t txBuffer[16];
	uint8_t status[16];
	uint8_t checksum = 0; // Checksum = ~(ID + Length + Instruction + Parameter1 + ... + Parameter n)
	uint16_t error = 0;

	hal_.setTxMode();

	txBuffer[0] = 0xFF; // Header is 0xFF
	txBuffer[1] = 0xFF; // Header is not included in checksum
	txBuffer[2] = servoID;
	checksum += txBuffer[2]; // 0-254, 0xFE = broadcast id
	txBuffer[3] = parameterLength + 2;
	checksum += txBuffer[3]; // Instruction + parameters (start add + values) + checksum                                                                                                        //0xFF and ID not included
	txBuffer[4] = instruction;
	checksum += txBuffer[4];

	for (i = 0; i < parameterLength; i++)
	{
		txBuffer[i + 5] = data[i];
		checksum += txBuffer[i + 5];
	}
	txBuffer[i + 5] = ~checksum;

	packetLength = txBuffer[3] + 4; // Number of bytes for the whole packet

	hal_.writeData(txBuffer, packetLength);

	// G15 only response if it was not broadcast command
	if ((ID != 0xFE) || (instruction == iPING))
	{
		if (instruction == iREAD_DATA)
		{
			parameterLength = data[1];
			packetLength = data[1] + 6;
		}
		else
		{
			packetLength = 6;
		}

		hal_.setRxMode();

		readCount = hal_.readData(status, packetLength);

		hal_.setTxMode();

		if (readCount == 0)
		{
			error = 0xFFFF;
		}
		else
		{
			error = 0;
			if (readCount != packetLength)
			{
				error |= 0x0100;
			}
			if ((status[0] != 0xFF) || (status[1] != 0xFF))
			{
				error |= 0x0200;
			}
			if (status[2] != servoID)
			{
				error |= 0x0400;
			}
			if (status[4] != 0)
			{
				error |= status[4];
			}

			// Calculate checksum
			checksum = 0;
			for (i = 2; i < packetLength; i++)
			{
				checksum += status[i];
			}
			if (checksum != 0xFF)
			{
				error |= 0x0800;
			}
			if (status[4] == 0x00 && (error & 0x0100) == 0x00) // Copy data only if there is no packet error
			{
				if (instruction == iPING)
				{
					data[0] = status[2];
				}
				else if (instruction == iREAD_DATA)
				{
					for (i = 0; i < parameterLength; i++)
					{
						data[i] = status[i + 5];
					}
				}
			}
		}
	}

	return (error);
}

void G15_Servo::_sendInstruction(uint8_t servoID, uint8_t instruction, uint8_t address, uint8_t data[], uint8_t length)
{
	uint8_t header[6];
	uint8_t header_length = 6;
	uint8_t checksum = 0; // Checksum = ~(ID + Length + Instruction + Parameter1 + ... + Parameter n)

	hal_.setTxMode();

	header[0] = 0xFF; // Header is 0xFF
	header[1] = 0xFF; // Header is not included in checksum
	header[2] = servoID;
	checksum += header[2]; // 0-254, 0xFE = broadcast id                                                                                                      //0xFF and ID not included
	header[4] = instruction;
	checksum += header[4];
	if (instruction != iACTION)
	{
		header[3] = length + 3;
		header[5] = address;
		checksum += header[5];
	}
	else
	{
		header[3] = length + 2;
		header_length = 5;
	}
	checksum += header[3];

	for (uint8_t i = 0; i < length; i++)
	{
		checksum += data[i];
	}
	checksum = ~checksum;

	hal_.writeData(header, data, checksum, header_length, length);

	error_handler_->registerSentPacket(header, checksum, data, header_length, length);
}

uint16_t G15_Servo::_readResponse(uint8_t servoID, uint8_t data[], uint8_t length)
{
	uint16_t error = 0;
	uint8_t expected_length = 6 + length;

	hal_.setRxMode();

	uint8_t header[5] = {0};
	uint8_t checksum_packet = 0;

	uint8_t checksum = 0;

	uint8_t actual_length = hal_.readData(header, data, checksum_packet, length);

	hal_.setTxMode();

	if (actual_length == 0)
	{
		error = 0xFFFF;
	}
	else
	{
		error = 0;
		if (actual_length != expected_length)
		{
			error |= 0x0100;
		}
		if ((header[0] != 0xFF) || (header[1] != 0xFF))
		{
			error |= 0x0200;
		}
		if (header[2] != servoID)
		{
			error |= 0x0400;
		}
		if (header[4] != 0)
		{
			error |= header[4];
		}

		// Calculate checksum
		for (uint8_t i = 2; i < 5; i++)
		{
			checksum += header[i];
		}
		for (uint8_t i = 0; i < length; i++)
		{
			checksum += data[i];
		}

		checksum = ~checksum;
		if (checksum != checksum_packet)
		{
			error |= 0x0800;
		}
	}

	error_handler_->registerReceivedPacket(servoID, header, checksum_packet, data, expected_length, actual_length, checksum, error);
	return error;
}

uint16_t G15_Servo::_checkAndReadResponse(uint8_t servoID)
{
	if (servoID!=BROADCAST_ID && check_response_packet_)
	{
		return _readResponse(servoID);
	}
	else
	{
		return 0;
	}
}


uint16_t G15_Servo::setWheelMode(uint8_t servoID)
{
	uint16_t error = 0;
	error = setAngleLimit(servoID, 0, 0); // Enable wheel mode
	if (error != 0)
	{
		return (error);
	}

	error = setTorqueOnOff(servoID, ON); // Enable torque

	return (error);
}

uint16_t G15_Servo::exitWheelMode(uint8_t servoID)
{
	return (setAngleLimit(servoID, 0, 1087)); // Reset to default angle limit
}

uint16_t G15_Servo::setWheelSpeed(uint8_t servoID, uint16_t speed, uint8_t direction)
{
	speed = speed & 0x03FF; // Eliminate bits which are non speed
	if (direction == CW)
	{
		speed = speed | 0x0400;
	}
	return (setSpeed(servoID, speed));
}

//******************************************************************
//*	SET GOAL POSITION
//* 	eg:	dat[0] = 0xC8;				// Position lower byte
//*			dat[1] = 0x00;				// Position upper byte
//*			SetPos(dat,iWRITE_DATA);	// Send to servo 2 & action!
//******************************************************************
uint16_t G15_Servo::setPos(uint8_t servoID, uint16_t position)
{
	uint8_t txBuffer[3];

	txBuffer[0] = GOAL_POSITION_L;	 // Control Starting Address
	txBuffer[1] = position & 0x00FF; // Goal pos bottom 8 bits
	txBuffer[2] = position >> 8;	 // Goal pos top 8 bits

	return (sendPacket(servoID, iWRITE_DATA, txBuffer, 3));
}

uint16_t G15_Servo::setPosAngle(uint8_t servoID, uint16_t angle)
{
	uint8_t txBuffer[3];
	uint16_t position = ConvertAngleToPos(angle);

	txBuffer[0] = GOAL_POSITION_L;	 // Control Starting Address
	txBuffer[1] = position & 0x00FF; // Goal pos bottom 8 bits
	txBuffer[2] = position >> 8;	 // Goal pos top 8 bits

	return (sendPacket(servoID, iWRITE_DATA, txBuffer, 3));
}

uint16_t G15_Servo::setPosSpeed(uint8_t servoID, uint16_t position, uint16_t speed)
{
	uint8_t txBuffer[5];

	txBuffer[0] = GOAL_POSITION_L;	 // Control Starting Address
	txBuffer[1] = position & 0x00FF; // Goal pos bottom 8 bits
	txBuffer[2] = position >> 8;	 // Goal pos top 8 bits
	txBuffer[3] = speed & 0x00FF;	 // Speed bottom 8 bits
	txBuffer[4] = speed >> 8;		 // Speed top 8 bits

	return (sendPacket(servoID, iWRITE_DATA, txBuffer, 5));
}

uint16_t G15_Servo::rotateCW(uint8_t servoID, uint16_t position)
{
	position = position | 0xC000;

	return (setPos(servoID, position));
}

uint16_t G15_Servo::rotateCCW(uint8_t servoID, uint16_t position)
{
	position = position | 0x8000;
	position = position & 0xBFFF;

	return (setPos(servoID, position));
}

//******************************************************************
//*	SET TORQUE ON OFF
//* 	eg:	SetTorqueOnOff(1,iREG_WRITE);	// Turn on torque of servo 2
//******************************************************************
uint16_t G15_Servo::setTorqueOnOff(uint8_t servoID, uint8_t onOff)
{
	uint8_t txBuffer[2];

	txBuffer[0] = TORQUE_ENABLE;
	txBuffer[1] = onOff; // ON = 1, OFF = 0

	return (sendPacket(servoID, iWRITE_DATA, txBuffer, 2));
}
//******************************************************************
//*	SET SPEED
//* 	eg:	dat[0] = 0x0A;				// Speed lower byte
//*			dat[1] = 0x02;				// Speed upper byte
//*			SetSpeed(dat,iREG_WRITE);	// Save data in servo 2 register &
//*										// wait for action command
//******************************************************************
uint16_t G15_Servo::setSpeed(uint8_t servoID, uint16_t speed)
{
	uint8_t txBuffer[3];

	txBuffer[0] = MOVING_SPEED_L;
	txBuffer[1] = speed & 0x00FF; // Speed bottom 8 bits
	txBuffer[2] = speed >> 8;	  // Speed top 8 bits

	return (sendPacket(servoID, iWRITE_DATA, txBuffer, 3));
}

//********************************************************************
uint16_t G15_Servo::setTimeToGoal(uint8_t servoID, uint16_t time)
{
	time = time & 0x0FFF;
	time = time | 0x8000; // Bit 15 represents the time to goal pos mode

	return (setSpeed(servoID, time));
}

//******************************************************************
//*	SET ANGLE LIMIT
//* byte SetAngleLimit(word CW_angle, word CCW_angle)
//*	CW_angle & CCW_angle are not in degree value
//*	Use ConverAngle to convert angle values if needed
//*
//*
//******************************************************************
uint16_t G15_Servo::setAngleLimit(uint8_t servoID, uint16_t CW_angle, uint16_t CCW_angle)
{
	uint8_t txBuffer[5];
	uint16_t error = 0;

	txBuffer[0] = CW_ANGLE_LIMIT_L;
	txBuffer[1] = CW_angle & 0x00FF;  // CW limit bottom 8 bits
	txBuffer[2] = CW_angle >> 8;	  // CW limit top 8 bits
	txBuffer[3] = CCW_angle & 0x00FF; // CCW limit bottom 8 bits
	txBuffer[4] = CCW_angle >> 8;	  // CCW limit top 8 bits

	error = sendPacket(servoID, iWRITE_DATA, txBuffer, 5);
	hal_.delayMilliseconds(10); // Delay for eeprom write
	return (error);
}

uint16_t G15_Servo::setTorqueLimit(uint8_t servoID, uint16_t TorqueLimit)
{
	uint8_t txBuffer[3];

	txBuffer[0] = TORQUE_LIMIT_L;
	txBuffer[1] = TorqueLimit & 0x00FF; // Torque limit bottom 8 bits
	txBuffer[2] = TorqueLimit >> 8;		// Torque limit top 8 bits

	return (sendPacket(servoID, iWRITE_DATA, txBuffer, 3));
}

uint16_t G15_Servo::setTemperatureLimit(uint8_t servoID, uint8_t temperature)
{
	uint8_t txBuffer[2];
	uint16_t error;

	txBuffer[0] = LIMIT_TEMPERATURE; //Starting Address
	txBuffer[1] = temperature;		 //temperature

	error = sendPacket(servoID, iWRITE_DATA, txBuffer, 2);
	hal_.delayMilliseconds(10); // Delay for eeprom write
	return (error);
}

uint16_t G15_Servo::setVoltageLimit(uint8_t servoID, uint8_t voltageLow, uint8_t voltageHigh)
{
	uint8_t txBuffer[3];
	uint16_t error;

	txBuffer[0] = DOWN_LIMIT_VOLTAGE;
	txBuffer[1] = voltageLow;  // Lower voltage limit
	txBuffer[2] = voltageHigh; // Higher voltage limit

	error = sendPacket(servoID, iWRITE_DATA, txBuffer, 3);
	hal_.delayMilliseconds(10); // Delay for eeprom write
	return (error);
}

//******************************************************************
//*	SET ID
//* 	eg:	SetID(MAIN,0xFE,3);	// Change the ID of any number to 3
//******************************************************************
uint16_t G15_Servo::setID(uint8_t servoID, uint8_t newID)
{
	uint8_t txBuffer[2];
	uint16_t error;

	txBuffer[0] = ID;
	txBuffer[1] = newID;

	error = sendPacket(servoID, iWRITE_DATA, txBuffer, 2);
	hal_.delayMilliseconds(10); // Delay for eeprom write
	return (error);
}

//******************************************************************
//*	SET LED
//* 	eg:	SetLED(1,iWRITE_DATA);	// Turn on LED of servo 2
//******************************************************************
uint16_t G15_Servo::setLED(uint8_t servoID, uint8_t onOff)
{
	uint8_t txBuffer[2];

	txBuffer[0] = LED;	 // Control Starting Address
	txBuffer[1] = onOff; // ON = 1, OFF = 0

	return (sendPacket(servoID, iWRITE_DATA, txBuffer, 2));
}

uint16_t G15_Servo::setAlarmLED(uint8_t servoID, uint8_t alarmLED)
{
	uint8_t alarmValue = 0x00;
	uint8_t txBuffer[2];
	uint16_t error;

	alarmValue = alarmValue | alarmLED;

	txBuffer[0] = ALARM_LED;
	txBuffer[1] = alarmValue; // Alarm value

	error = sendPacket(servoID, iWRITE_DATA, txBuffer, 2);
	hal_.delayMilliseconds(10);
	return (error);
}

uint16_t G15_Servo::setAlarmShutDown(uint8_t servoID, uint8_t alarm)
{
	uint8_t alarmValue = 0x00;
	uint8_t txBuffer[2];
	uint16_t error;

	alarmValue = alarmValue | alarm;

	txBuffer[0] = ALARM_SHUTDOWN; // Control Starting Address
	txBuffer[1] = alarmValue;	  // Alarm

	error = sendPacket(servoID, iWRITE_DATA, txBuffer, 2);
	hal_.delayMilliseconds(10); // Delay for eeprom write
	return (error);
}

uint16_t G15_Servo::setMarginSlopePunch(uint8_t servoID, uint8_t CWMargin, uint8_t CCWMargin, uint8_t CWSlope, uint8_t CCWSlope, uint16_t punch)
{
	uint8_t txBuffer[5];
	uint16_t error = 0;

	txBuffer[0] = CW_COMPLIANCE_MARGIN; //Control Starting Address
	txBuffer[1] = CWMargin;
	txBuffer[2] = CCWMargin;
	txBuffer[3] = CWSlope;
	txBuffer[4] = CCWSlope;

	error = sendPacket(servoID, iWRITE_DATA, txBuffer, 5);

	if (error != 0)
	{
		return (error);
	}

	txBuffer[0] = PUNCH_L;		  //Control Starting Address
	txBuffer[1] = punch & 0x00FF; //punch Lower 8 bits
	txBuffer[2] = punch >> 8;	  //punch Higher 8 bits

	error = sendPacket(servoID, iWRITE_DATA, txBuffer, 3);

	return (error);
}

//******************************************************************
//*	SET BAUDRATE
//* 	eg:	SetBaud(1,2,1);	// Turn on torque of servo 2
//******************************************************************
uint16_t G15_Servo::setBaudRate(uint8_t servoID, uint32_t baudrate)
{
	uint8_t txBuffer[2];
	uint16_t error;

	txBuffer[0] = BAUD_RATE;
	txBuffer[1] = (2000000 / baudrate) - 1; // Baudrate = 32M / (16*(n + 1)) = 2000000 / (n+1)

	error = sendPacket(servoID, iWRITE_DATA, txBuffer, 2);
	hal_.delayMilliseconds(10); // Delay for eeprom write
	return (error);
}

//******************************************************************
//*	RESET TO FACTORY SETTINGS
//* 	eg:	FactoryReset(1,1);// Reset servo 1
//******************************************************************
uint16_t G15_Servo::factoryReset(uint8_t servoID)
{
	uint8_t txBuffer[1]; // Dummy byte
	uint16_t error;

	error = sendPacket(servoID, iRESET, txBuffer, 0);
	hal_.delayMilliseconds(100); // Delay for eeprom write
	return (error);
}
//******************************************************************
//*	PING
//* 	eg:	Ping(MAIN,1,dat);	// Ping servo 1, dat is array's pointer
//******************************************************************
uint16_t G15_Servo::ping(uint8_t servoID, uint8_t *data)
{
	return (sendPacket(servoID, iPING, data, 0));
}

//******************************************************************
//*	GET CURRENT POSITION
//******************************************************************
uint16_t G15_Servo::getPos(uint8_t servoID, uint8_t *data)
{
	data[0] = PRESENT_POSITION_L; // Starting address where data to be read
	data[1] = 0x02;				  // No of bytes to be read

	return (sendPacket(servoID, iREAD_DATA, data, 2));
}

uint16_t G15_Servo::getSpeed(uint8_t servoID, uint8_t *data)
{
	data[0] = PRESENT_SPEED_L; // Starting address where data to be read
	data[1] = 0x02;			   // No of bytes to be read

	return (sendPacket(servoID, iREAD_DATA, data, 2));
}

uint16_t G15_Servo::getLoad(uint8_t servoID, uint8_t *data)
{
	data[0] = PRESENT_LOAD_L; // Starting address where data to be read
	data[1] = 0x02;			  // No of bytes to be read

	return (sendPacket(servoID, iREAD_DATA, data, 2));
}

uint16_t G15_Servo::getVoltage(uint8_t servoID, uint8_t *data)
{
	data[0] = PRESENT_VOLTAGE; // Starting address where data to be read
	data[1] = 0x01;			   // No of bytes to be read

	return (sendPacket(servoID, iREAD_DATA, data, 2));
}

uint16_t G15_Servo::getTemperature(uint8_t servoID, uint8_t *data)
{
	data[0] = PRESENT_TEMPERATURE; // Starting address where data to be read
	data[1] = 0x01;				   // No of bytes to be read

	return (sendPacket(servoID, iREAD_DATA, data, 2));
}

//******************************************************************
//*	GET TORQUE (ON/OFF?)
//******************************************************************
uint16_t G15_Servo::getTorqueOnOff(uint8_t servoID, uint8_t *data)
{
	data[0] = TORQUE_ENABLE; // Starting address where data to be read
	data[1] = 0x01;			 // No of bytes to be read

	return (sendPacket(servoID, iREAD_DATA, data, 2));
}

//******************************************************************
//*	IS MOTOR MOVING?
//******************************************************************
uint16_t G15_Servo::isMoving(uint8_t servoID, uint8_t *data)
{
	data[0] = MOVING; // Starting address where data to be read
	data[1] = 0x01;	  // No of bytes to be read

	return (sendPacket(servoID, iREAD_DATA, data, 2));
}

uint16_t G15_Servo::writeInstruction(uint8_t servoID, uint8_t address, uint8_t data[], uint8_t length)
{
	_sendInstruction(servoID, iWRITE_DATA, address, data, length);
	uint16_t error = _checkAndReadResponse(servoID);
	error_handler_->handleErrors();
	return error;
}

uint16_t G15_Servo::readInstruction(uint8_t servoID, uint8_t address, uint8_t data[], uint8_t length)
{
	_sendInstruction(servoID, iREAD_DATA, address, &length, 1);
	uint16_t error = _readResponse(servoID, data, length);
	error_handler_->handleErrors();
	return error;
}

uint16_t G15_Servo::regWriteInstruction(uint8_t servoID, uint8_t address, uint8_t data[], uint8_t length)
{
	_sendInstruction(servoID, iREG_WRITE, address, data, length);
	uint16_t error = _checkAndReadResponse(servoID);
	error_handler_->handleErrors();
	return error;
}

uint16_t G15_Servo::actionInstruction(uint8_t servoID)
{
	_sendInstruction(BROADCAST_ID, iACTION, 0, nullptr, 0);
	uint16_t error = _checkAndReadResponse(servoID);
	error_handler_->handleErrors();
	return error;
}

void G15_Servo::syncWrite(uint8_t address, uint8_t data[], uint8_t data_length)
{
	_sendInstruction(BROADCAST_ID, iSYNC_WRITE, address, data, data_length);
	error_handler_->handleErrors();
}

uint16_t G15_Servo::convertToRegisterValue(double inputQuantity, double maxInputQuantity, uint16_t maxRegisterValue)
{
	return (uint16_t)((inputQuantity/maxInputQuantity) * maxRegisterValue);
}

double G15_Servo::convertFromRegisterValue(uint16_t registerValue, uint16_t maxRegisterValue, double maxOutputQuanity)
{
	return (double)(((double)registerValue/maxRegisterValue) * maxOutputQuanity);
}

