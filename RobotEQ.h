// Copyright (C) 2013 Andy Kipp <kipp.andrew@gmail.com> 
// 
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
// 
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
// CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
// SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef ROBOTEQ_H
#define ROBOTEQ_H

#define ROBOTEQ_QUERY_CHAR 0x05
#define ROBOTEQ_ACK_CHAR 0x06
#define ROBOTEQ_DEFAULT_TIMEOUT 1000 

#include "Logging.h"
#include <HardwareSerial.h>

enum RobotEQFaultFlag {
	Overheat = 1,
	OverVoltage = 2,
	UnderVoltage = 3,
	Short = 4,
	EmergencyStop = 5,
	ScriptExecFault = 6,
	MOSFETFailure = 7,
	StartupConfigurationFault = 8
};

enum RobotEQStatusFlag {
	SerialMode = 1,
	PulseMode = 2,
	AnalogMode = 3,
	PowerStageOff = 4,
	StallDetected = 5,
	AtLimit = 6,
	ScriptRunning = 8
};

class RobotEQ {
	// Constructors
	public:
		RobotEQ(Stream *serial);

	// Destructors
	public:
		~RobotEQ(void);

	// Public Methods
	public:
		void setTimeout(uint32_t timeout);

		int isConnected(void);
		int isConnected(uint32_t timeout);

		int commandMotorPower(uint8_t ch, int16_t p);

		int queryMotorAmps(uint8_t ch);
		int queryMotorPower(uint8_t ch);

		int queryBatteryAmps(void);
		int queryBatteryVoltage(void);

		int queryFaultFlag(RobotEQFaultFlag flag);
		int queryStatusFlag(RobotEQStatusFlag flag);

		int queryFirmware(char* buf, size_t size);

	// Private Methods
	private:

		int sendQuery(const char *str, uint8_t *r, size_t rSize); 
		int sendQuery(const uint8_t *q, size_t qSize, uint8_t *r, size_t rSize);

		int sendCommand(const char *str);
		int sendCommand(const uint8_t *q, size_t qSize);
		
		int send(const uint8_t b);
		int send(const uint8_t * buf, size_t size);

		int readSerialUntilNewline(uint8_t * buf, size_t size);

	// Private Data
	private:
		uint32_t 	m_Timeout;
		Stream		*m_Serial; 
};

#endif

