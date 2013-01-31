#include "Arduino.h"
#include "RobotEQ.h"

#define ETIMEOUT -1 
#define EIO -2

RobotEQ::RobotEQ(Stream *serial) {
	m_Timeout = ROBOTEQ_DEFAULT_TIMEOUT;	
	m_Serial = serial; 
}

RobotEQ::~RobotEQ(void) {
}

void RobotEQ::setTimeout(uint32_t timeout) {
	m_Timeout = timeout;
}

int RobotEQ::isConnected() {
	return this->isConnected(this->m_Timeout);
}

int RobotEQ::isConnected(uint32_t timeout) {
	if (this->m_Serial == NULL) 
		return EIO;
	
	uint8_t inByte = 0;
	uint32_t startTime = millis();

	// send QRY 
	this->send(ROBOTEQ_QUERY_CHAR);

	while (millis() - startTime < timeout) {
		// wait for ACK
		if (m_Serial->available() > 0) {
			inByte = m_Serial->read();
			if (inByte == ROBOTEQ_ACK_CHAR) {
				return 0;
			}
		}
	   	//delay(100);	
	}
	return ETIMEOUT;
}
		
int RobotEQ::commandMotorPower(uint8_t ch, int16_t p) {
	char command[32];
	sprintf(command, "!G %02d %d\r", ch, p);
	return this->sendCommand(command);
}

int RobotEQ::queryFaultFlag(RobotEQFaultFlag flag) {
	uint8_t fault;
	if (this->sendQuery("?FF\r", &fault, 1) <= 0) {
		return -1;
	}
	return (fault & (1 << (int)flag) > 0);
}

int RobotEQ::queryStatusFlag(RobotEQStatusFlag flag) {
	uint8_t status;
	if (this->sendQuery("?FS\r", &status, 1) <= 0) {
		return -1;
	}
	return (status & (1 << (int)flag) > 0);
}

int RobotEQ::queryFirmware(char *buf, size_t size) {
	return this->sendQuery("?FID\r", (uint8_t*)buf, size);
}

int RobotEQ::sendCommand(const char *str) {
	return this->sendCommand((uint8_t*)str, strlen(str));
}
		
int RobotEQ::sendCommand(const uint8_t *q, size_t qSize) {
	if (this->m_Serial == NULL) 
		return EIO;

	uint8_t buf[32];

	// Write Command to Serial
	this->send(q, qSize);

	// Read Serial until timeout or newline
	if (this->readSerialUntilNewline(buf, 32) <= 0) {
		return -1;
	}	

	// Check Command Status
	if (strncmp((char*)buf, "+", 1) == 0) {
		return 0;
	} else {
		return 1;
	}
}

int RobotEQ::sendQuery(const char *str, uint8_t *r, size_t rSize) {
	return this->sendQuery((const uint8_t*)str, strlen(str), r, rSize);
}

int RobotEQ::sendQuery(const uint8_t *q, size_t qSize, uint8_t *r, size_t rSize) {
	if (this->m_Serial == NULL) 
		return EIO;

	// Write Query to Serial
	this->send(q, qSize);

	// Read Serial until timeout or newline
	return this->readSerialUntilNewline(r, rSize);
}

int RobotEQ::send(const uint8_t b) {
	//Log.Debug("sending to controller: %X"CR, (char)b);	
	this->m_Serial->write(b);
	this->m_Serial->flush();
}

int RobotEQ::send(const uint8_t * buf, size_t size) {
	//Log.Debug("sending to controller: %s"CR, (char*)buf);	
	this->m_Serial->write(buf, size);
	this->m_Serial->flush();
}

int RobotEQ::readSerialUntilNewline(uint8_t * buf, size_t size) {
	uint8_t inByte;
	size_t index = 0;
	uint32_t startTime = millis();	
	while (millis() - startTime < this->m_Timeout) {
		if (m_Serial->available() > 0) {
			inByte = m_Serial->read();
			//Log.Debug("read %X from controller"CR, inByte);
			buf[index++] = inByte;
			if (inByte == 0x0D) 
				return index;
		}
	}
	// timeout
	Log.Error("timeout reading controller"CR);
	return ETIMEOUT;
}
