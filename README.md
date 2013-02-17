About
=====

The Arduino-RobotEQ library is an abstraction interface for the RobotEQ Motor
Controllers. It uses the serial command inteface to send and receive commands
from the RobotEQ controller using an Arduino.

Usage
=====

    #include <RobotEQ.h>

    #define CHANNEL_1 1

    // Configure Motor Controllers
    RobotEQ controller(&Serial);

    void setup() {
    }

    void loop() {
        int voltage;
        int amps;

        if (controller.isConnected()) {
            voltage = controller.queryBatteryVoltage();
            amps = controller.queryBatteryAmps();

            controller.commandMotorPower(CHANNEL_1, 1000);
        }
    }
