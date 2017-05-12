//
// Created by Philipp Zech on 12.04.17.
//

#include <squirrel_control/motor_utilities.h>


namespace motor_control {

    MotorUtilities::MotorUtilities() {
    }

    MotorUtilities::~MotorUtilities() {
        stopMotors();
        for(auto const& motor : motors_) {
            delete motor.tool;
        }
        delete port_handler_;
        delete packet_handler_;
    }


    void MotorUtilities::setMode(control_modes::ControlMode mode) {
	    UINT8_T error;
	    try {
		    motor_lock_.lock();
		    current_mode_ = mode;
		    for(const auto motor : motors_){
			    packet_handler_->Write1ByteTxRx(port_handler_, motor.id,
			                                        motor.tool->ctrl_table_["operating_mode"]->address,
			                                   current_mode_, &error);
			    throw_control_error(error,
			                        "Failed to switch motor " << motor.id << " (" << motor.tool->model_name_
			                                                  << ") into mode " << mode);
		    }
		    motor_lock_.unlock();
	    } catch (std::exception &ex) {
		    motor_lock_.unlock();
		    throw_control_error(true, ex.what());
	    }
    }


	control_modes::ControlMode MotorUtilities::getMode() {
        return current_mode_;
    }


    std::vector<Motor> MotorUtilities::getMotors() {
        return motors_;
    }


    bool MotorUtilities::motorsReady() {
        return motors_ready_;
    }


    bool MotorUtilities::torqueEnabled() {
        return torque_enabled_;
    }


    bool MotorUtilities::initMotors(std::string motor_port, std::vector<int> motors) {
        try {
	        port_handler_ = ROBOTIS::PortHandler::GetPortHandler(motor_port.c_str());
	        port_handler_->SetBaudRate(BAUD_RATE_);
	        port_handler_->OpenPort();
        } catch(const std::exception &e) {
            throw_control_error(true, e.what());
        }

        //do find the whole gang?
	    packet_handler_ = ROBOTIS::Protocol2PacketHandler::GetInstance();
        UINT8_T error = 0;
        for(auto const& motor: motors) {
	        packet_handler_->Ping(port_handler_, motor, &error);
            throw_control_error(error, "Failed to ping motor: " << motor);
        }

        //initialize them
        motors_ = std::vector<Motor>();
        for(auto const& motor: motors) {
            uint16_t model_id = packet_handler_->Read2ByteTx(port_handler_, motor, 0);
            dynamixel_tool::DynamixelTool tool = dynamixel_tool::DynamixelTool(motor, model_id, 2.0);
            Motor motor_;
            motor_.id = motor;
            motor_.tool = &tool;
            motors_.push_back(motor_);
        }
        return true;
    }


    bool MotorUtilities::startMotors() {
        UINT8_T error;
        for(auto const& motor: motors_) {
	        packet_handler_->Write2ByteTxRx(port_handler_, motor.id, motor.tool->ctrl_table_["external_port_data_1"]->address, 4095, &error);
            throw_control_error(error, "Failed to loosen brakes for motor " << motor.id << " (" << motor.tool->model_name_ << ")");
        }
	    motors_ready_ = true;
        return true;
    }


    bool MotorUtilities::stopMotors() {
        UINT8_T error;
        for(auto const& motor: motors_) {
	        packet_handler_->Write1ByteTxRx(port_handler_, motor.id, motor.tool->ctrl_table_["external_port_data_1"]->address, 0, &error);
            throw_control_error(error, "Failed to set brakes for motor " << motor.id << " (" << motor.tool->model_name_ << ")");
        }
	    motors_ready_ = false;
        return true;
    }


    bool MotorUtilities::enableTorque() {
        UINT8_T error;
        for(auto const& motor: motors_) {
	        packet_handler_->Write1ByteTxRx(port_handler_, motor.id, motor.tool->ctrl_table_["torque_enable"]->address, 1, &error);
            throw_control_error(error, "Failed to enable torque for motor " << motor.id << " (" << motor.tool->model_name_ << ")");
        }
	    torque_enabled_ = true;
        return true;
    }


    bool MotorUtilities::disableTorque() {
        UINT8_T error;
        for(auto const& motor: motors_) {
	        packet_handler_->Write1ByteTxRx(port_handler_, motor.id, motor.tool->ctrl_table_["torque_enable"]->address, 0, &error);
            throw_control_error(error, "Failed to enable torque for motor " << motor.id << " (" << motor.tool->model_name_ << ")");
        }
	    torque_enabled_ = false;
        return true;
    }


    //PID control should actually already be done in the motor
    //this only consider low level (HW) limits
    //conversion from joint pos to velocity/torque done in squirrel_hw_interface or here?
    bool MotorUtilities::write(std::vector<double> commands)
    {
        throw_control_error(commands.size() != motors_.size(), "Wrong number of commands! Got " << commands.size() << ", but expected " << motors_.size());

        int i = 0;
        UINT8_T error;

	    motor_lock_.lock();

        //we throw a bunch of own exceptions - let's catch them all and forward them to at all times release the lock
        try {
            for (auto const motor : motors_) {
                switch (current_mode_) {
	                case control_modes::ControlMode::POSITION_MODE: {
                        double rad_per_tick = motor.tool->max_radian_ / motor.tool->value_of_max_radian_position_;
                        UINT32_T goal_position = (UINT32_T) commands.at(i) / rad_per_tick;
                        UINT32_T homing_offset;
		                packet_handler_->Read4ByteTxRx(port_handler_, motor.id,
                                                           motor.tool->ctrl_table_["homing_offset"]->address,
                                                           &homing_offset, &error);
                        throw_control_error(error, "Failed to read homing offset for motor " << motor.id << " ("
                                                                                             << motor.tool->model_name_
                                                                                             << ")");
                        UINT32_T min_with_offset = homing_offset + motor.tool->value_of_min_radian_position_;
                        UINT32_T max_with_offset = homing_offset + motor.tool->value_of_max_radian_position_;
                        UINT32_T goal_position_with_offset = goal_position + homing_offset;
                        throw_control_error(goal_position_with_offset < min_with_offset || goal_position_with_offset > max_with_offset,
                                            "Motor " << motor.id << " (" << motor.tool->model_name_
                                                     << ") exceeds its limits [" << min_with_offset << "," << max_with_offset
                                                     << "] with goal position: " << goal_position);
		                packet_handler_->Write2ByteTxRx(port_handler_, motor.id,
                                                            motor.tool->ctrl_table_["goal_position"]->address,
                                                            goal_position_with_offset, &error);
                        }
                        break;

	                case control_modes::ControlMode::VELOCITY_MODE: {
                        UINT32_T velocity_limit;
		                packet_handler_->Read4ByteTxRx(port_handler_, motor.id,
                                                           motor.tool->ctrl_table_["velocity_limit"]->address,
                                                           &velocity_limit, &error);
                        throw_control_error(error, "Failed to read velocity limit for motor " << motor.id << " ("
                                                                                             << motor.tool->model_name_
                                                                                             << ")");
                        UINT32_T commanded_velocity = (UINT32_T)commands.at(i);
                        throw_control_error( commanded_velocity > velocity_limit,
                                            "Motor " << motor.id << " (" << motor.tool->model_name_
                                                     << ") exceeds its velocity [" << velocity_limit << "] with commanded velocity: " << commanded_velocity);
		                packet_handler_->Write2ByteTxRx(port_handler_, motor.id,
                                                            motor.tool->ctrl_table_["goal_velocity"]->address,
                                                            commanded_velocity, &error);
                        }
                        break;

                    case control_modes::ControlMode::TORQUE_MODE: {
                        UINT16_T torque_limit;
	                    packet_handler_->Read2ByteTxRx(port_handler_, motor.id,
                                                           motor.tool->ctrl_table_["torque_limit"]->address,
                                                           &torque_limit, &error);
                        throw_control_error(error, "Failed to read torque limit for motor " << motor.id << " ("
                                                                                              << motor.tool->model_name_
                                                                                              << ")");
                        UINT16_T commanded_torque = (UINT16_T)commands.at(i);
                        throw_control_error( commanded_torque > torque_limit,
                                             "Motor " << motor.id << " (" << motor.tool->model_name_
                                                      << ") exceeds its velocity [" << torque_limit << "] with commanded velocity: " << commanded_torque);
	                    packet_handler_->Write2ByteTxRx(port_handler_, motor.id,
                                                            motor.tool->ctrl_table_["goal_torque"]->address,
                                                            commanded_torque, &error);
                        }
                        break;

                    default:
                        throw_control_error(true, "Unknown mode: " << current_mode_);
                }
	            //this theoretically should never be evaluated, but for sake of completeness...
                throw_control_error(error,
                                    "Failed to command motor " << motor.id << " (" << motor.tool->model_name_ << ") in "
                                                               << current_mode_ << " mode");
                i++;
            }
        } catch(const std::exception &ex) {
	        motor_lock_.unlock();
	        throw_control_error(true, ex.what());
        }

	    try{
	        motor_lock_.unlock();
	    } catch(std::exception &ex) {
		    throw_control_error(true, ex.what());
		    motor_lock_.unlock();
		    return false;
	    }
        return true;
    }


    std::vector<double> MotorUtilities::read() {
	    UINT8_T error;
        switch(current_mode_) {
            case control_modes::ControlMode::POSITION_MODE:
                {
	                std::vector<double> positions = {};
					UINT16_T value;
	                for (auto const motor : motors_) {
		                packet_handler_->Read2ByteTxRx(port_handler_, motor.id,
		                                                   motor.tool->ctrl_table_["present_position"]->address,
		                                                   &value, &error);
		                throw_control_error(error, "Error reading position for motor " << motor.id << " (" << motor.tool->model_name_ << ")");
		                positions.push_back(double(value));
	                }
                    return positions;
                }
            case control_modes::ControlMode::VELOCITY_MODE:
                {
	                std::vector<double> velocities = {};
	                UINT16_T value;
	                for (auto const motor : motors_) {
		                packet_handler_->Read2ByteTxRx(port_handler_, motor.id,
		                                                   motor.tool->ctrl_table_["present_velocity"]->address,
		                                                   &value, &error);
		                throw_control_error(error, "Error reading velocity for motor " << motor.id << " (" << motor.tool->model_name_ << ")");
		                velocities.push_back(double(value));
	                }
	                return velocities;
                }
            case control_modes::ControlMode::TORQUE_MODE:
                {
	                //TODO adapt if necessary
	                std::vector<double> torques = {};
	                UINT16_T value;
	                for (auto const motor : motors_) {
		                packet_handler_->Read2ByteTxRx(port_handler_, motor.id,
		                                              motor.tool->ctrl_table_["present_current"]->address,
		                                              &value, &error);
		                throw_control_error(error, "Error reading torque for motor " << motor.id << " (" << motor.tool->model_name_ << ")");
		                torques.push_back(abs(double(value)*CURRENT_TO_TORQUE_RATIO_));
	                }
	                return torques;
                }
            default:
                throw_control_error(true, "Unknown mode: " << current_mode_);
        }
    }

}