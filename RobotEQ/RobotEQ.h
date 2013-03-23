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

#include <Stream.h>

// Logging Library available here:
// http://playground.arduino.cc/Code/Logging 

// Note: Uncomment this to enable debug logging
#define ROBOTEQ_DEBUG
#ifdef ROBOTEQ_DEBUG
    #include "Logging.h"
    #define LOGLEVEL LOG_LEVEL_DEBUG
#endif

#define ROBOTEQ_DEFAULT_TIMEOUT     1000
#define ROBOTEQ_BUFFER_SIZE         64
#define ROBOTEQ_COMMAND_BUFFER_SIZE 20

#define ROBOTEQ_QUERY_CHAR          0x05
#define ROBOTEQ_ACK_CHAR            0x06

#define ROBOTEQ_OK                  0
#define ROBOTEQ_TIMEOUT             -1
#define ROBOTEQ_ERROR               -2
#define ROBOTEQ_BAD_COMMAND         -3
#define ROBOTEQ_BAD_RESPONSE        -4
#define ROBOTEQ_BUFFER_OVER         -5

#define ROBOTEQ_FAULT_OVERHEAT      0x01
#define ROBOTEQ_FAULT_OVERVOLTAGE   0x02
#define ROBOTEQ_FAULT_UNDERVOLTAGE  0x04
#define ROBOTEQ_FAULT_SHORT         0x08
#define ROBOTEQ_FAULT_ESTOP         0x10
#define ROBOTEQ_FAULT_SCRIPT        0x20
#define ROBOTEQ_FAULT_MOSFET        0x40
#define ROBOTEQ_FAULT_CONFIG        0x80

#define ROBOTEQ_STATUS_SERIAL       0x01
#define ROBOTEQ_STATUS_PULSE        0x02
#define ROBOTEQ_STATUS_ANALOG       0x04
#define ROBOTEQ_STATUS_POWER_OFF    0x08
#define ROBOTEQ_STATUS_STALL        0x10
#define ROBOTEQ_STATUS_LIMIT        0x20
#define ROBOTEQ_SCRIPT_RUN          0x80

class RobotEQ {
    // Constructors
    public:
        RobotEQ(Stream *serial);

    // Destructors
    public:
        ~RobotEQ(void);

    // Public Methods
    public:
        /*
         * check if controller is connected
         *
         * @return ROBOTEQ_OK if connected
         */
        int isConnected(void);

        //*********************************************************************
        // Commands
        //*********************************************************************

        /*
         * send motor power command (!G)
         *
         * @param ch channel
         * @param p power level (-1000, 1000)
         * @return ROBOTEQ_OK if successful 
         */
        int commandMotorPower(uint8_t ch, int16_t p);

        /*
         * send emergency stop command (!EX)
         * note: you have to reset the controller after this sending command
         *
         * @return ROBOTEQ_OK if successful
         */
        int commandEmergencyStop(void);

        //*********************************************************************
        // Query
        //*********************************************************************

        /*
         * query controller firmware
         * 
         * @param
         * @param
         * @return
         */
        int queryFirmware(char *buf, size_t bufSize);

        /*
         * query motor amps
         * 
         * @param ch channel
         * @return motor amps * 10
         */
        int queryMotorAmps(uint8_t ch);

        /*
         * query battery amps
         * 
         * @return battery amps * 10
         */
        int queryBatteryAmps(void);

        /*
         * query battery amps
         * 
         * @param ch channel
         * @return battery amps * 10
         */
        int queryBatteryAmps(uint8_t ch);

        /*
         * query battery voltage
         * 
         * @return battery voltage * 10
         */ 
        int queryBatteryVoltage(void);

        /*
         * query motor voltage
         * 
         * @return motor voltage * 10
         */
        int queryMotorVoltage(void);

        /*
         * query internal temp 
         *
         * @return temp (in degrees C)
         */
        int queryInternalTemp(void);

        /*
         * query channel temp 
         * 
         * @param ch channel
         * @return temp (in degrees C)
         */
        int queryTemp(uint8_t ch);

        /*
         * query the motor power command
         * 
         * @param ch channel
         * @return motor power
         */
        int queryMotorPower(uint8_t ch);

        /*
         * query fault flags
         */
        int queryFaultFlag(void);

        /*
         * query status flags
         */
        int queryStatusFlag(void);

        /*
         * query encoder speed in RPM
         *
         * @param ch channel
         * @return rpm
         */
        int queryEncoderSpeed(uint8_t ch);

        /*
         * query encoder speed in RPM
         *
         * @param ch channel
         * @return rpm
         */
        int queryEncoderRelativeSpeed(uint8_t ch);

        /*
         * Query user variable
         *
         * @param var
         * @param value
         *
         * @return ROBOTEQ_OK if successful
         */
        int queryUserVariable(uint32_t var, int32_t *value);

        /*
         * Query user variable
         *
         * @param var
         * @param value
         *
         * @return ROBOTEQ_OK if successful
         */
        int queryUserVariable(uint32_t var, bool *value);

        //*********************************************************************
        // Configuration
        //*********************************************************************

        /*
         * set encoder pulse per rotation
         *
         * @param ch channel
         * @param ppr pulese per rotation
         *
         * @return ROBOTEQ_OK if successful
         */
        int setEncoderPulsePerRotation(uint8_t ch, uint16_t ppr);

        /*
         * get encoder pulse per rotation
         */
        int getEncoderPulsePerRotation(uint8_t ch);

        /*
         * set motor amp limit
         *
         * @param ch channel
         * @param a amps level (x10)
         *
         * @return ROBOTEQ_OK if successful
         */
        int setMotorAmpLimit(uint8_t ch, uint16_t a);

        /*
         * get motor amp limit
         */
        int getMotorAmpLimit(uint8_t ch);

        /*
         * load controller configuration
         *
         * @return ROBOTEQ_OK if successful
         */
        int loadConfiguration(void);

        /*
         * save controller configuration
         *
         * @return ROBOTEQ_OK if successful
         */
        int saveConfiguration(void);

        /*
         * set timeout
         */
        void setTimeout(uint16_t timeout);

    // Private Methods
    private:

        int sendQuery(const char *query, uint8_t *buf, size_t bufSize);
        int sendQuery(const char *query, size_t querySize, uint8_t *buf, size_t bufSize);

        int sendCommand(const char *command);
        int sendCommand(const char *command, size_t commandSize);

        int readResponse(uint8_t *buf, size_t bufSize);

    // Private Data
    private:
        uint16_t    m_Timeout;
        Stream      *m_Serial;

};

// Helper Functions
char* chomp(char* s);

#endif

