//
// Created by lokalmatador on 12.04.17.
//

#ifndef SQUIRREL_CONTROL_MOTOR_UTILITIES_H
#define SQUIRREL_CONTROL_MOTOR_UTILITIES_H

#include <mutex>
#include <cmath>
#include <error/throwControlError.h>

#include "squirrel_control/control_modes.h"

#include <dynamixel_sdk/PortHandler.h>
#include <dynamixel_sdk/Protocol2PacketHandler.h>
#include <dynamixel_sdk/dynamixel_tool.h>


namespace motor_control
{
    struct Motor {
        UINT8_T id;
        dynamixel_tool::DynamixelTool *tool;
    };


    class MotorUtilities
    {
    public:
        static constexpr int BAUD_RATE_ = 3000000;

	    static constexpr double CURRENT_TO_TORQUE_RATIO_ = 0.068267;

        MotorUtilities();

        ~MotorUtilities();

        bool initMotors(std::string motor_port, std::vector<int> motors);

        bool startMotors();

        bool stopMotors();

        bool motorsReady();

        bool enableTorque();

        bool disableTorque();

        bool torqueEnabled();

        std::vector<Motor> getMotors();

        void setMode(control_modes::ControlMode mode);

        control_modes::ControlMode getMode();

        /**
            Writes the high level control commands to the motor. this operation supports all three operating modes, i.e.,
                position mode, velocity mode, and torque mode

            @param commands, a vector of either (i) joint position, (ii) joint velocities, or (iii) joint torques
                depending on the current operation mode; the length of this vector is expected to be this.motors.size
            @return true in case of success
        */
        bool write(std::vector<double>commands);

        std::vector<double> read();


    private:

        bool motors_ready_;

        bool torque_enabled_;

        std::mutex motor_lock_;

        std::vector<Motor> motors_;

        control_modes::ControlMode current_mode_;

        ROBOTIS::PortHandler* port_handler_;

        ROBOTIS::Protocol2PacketHandler* packet_handler_;
    };

}

#endif //SQUIRREL_CONTROL_MOTOR_UTILITIES_H
