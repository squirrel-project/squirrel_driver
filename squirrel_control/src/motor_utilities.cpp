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

    std::cout << "Switching to mode " << mode << std::endl;

    if (mode == control_modes::ControlMode::VELOCITY_MODE || mode == control_modes::ControlMode::TORQUE_MODE) {
      throw_control_error(true, "Controller currently only supports position control!");
    }

    disableTorque();
    
    UINT8_T error = 0;
    int comm = 0;
    try {
      motor_lock_.lock();
      current_mode_ = mode;
      for(const auto motor : motors_){
	comm = packet_handler_->Write1ByteTxRx(port_handler_, motor.id,
					motor.tool->ctrl_table_["operating_mode"]->address,
					(UINT8_T)current_mode_, &error);
	if(comm != 0) {
	  std::cout << "Failed to switch motor " << static_cast<int>(motor.id) << " (" << motor.tool->model_name_ << ") into mode " << mode << std::endl;
	}
      }
      motor_lock_.unlock();
    } catch (std::exception &ex) {
      motor_lock_.unlock();
      throw_control_error(true, ex.what());
    }

    enableTorque();
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
    port_handler_ = ROBOTIS::PortHandler::GetPortHandler(motor_port.c_str());
    if (!port_handler_->OpenPort()) {
      throw_control_error(true, "Failed to open motor port!");
    }		
    if (!port_handler_->SetBaudRate(BAUD_RATE_)) {
      throw_control_error(true, "Failed to set baud rate!");
    }
    
    //initialize the whole gang
    packet_handler_ = ROBOTIS::PacketHandler::GetPacketHandler(2.0);
    UINT8_T error = 0;
    motors_ = std::vector<Motor>();
    
    uint8_t dynamixel_error = 0;
    int dynamixel_id = 0;
    uint16_t dynamixel_num = 0;
    
    for(dynamixel_id = 1; dynamixel_id < 253; dynamixel_id++) {
      if (packet_handler_->Ping(port_handler_, dynamixel_id, &dynamixel_num, &dynamixel_error) == 0) {
	std::cout << "Found model: " << dynamixel_num << " with id " << dynamixel_id << std::endl;
	dynamixel_tool::DynamixelTool* tool = new dynamixel_tool::DynamixelTool(dynamixel_id, dynamixel_num, 2.0);
	Motor motor_;
	motor_.id = dynamixel_id;
	motor_.tool = tool;
	motors_.push_back(motor_);
      } 
    }	
    std::cout << "Found " << motors_.size() << " motors" << std::endl;
    return true;
  }
  
  
  bool MotorUtilities::startMotors() {
    UINT8_T error = 0;
    int comm = 0;
    enableTorque();
    for(auto const& motor: motors_) {
      std::cout << "Starting motor " << static_cast<int>(motor.id)<< std::endl;
      comm = packet_handler_->Write2ByteTxRx(port_handler_, motor.id, motor.tool->ctrl_table_["external_port_data_1"]->address, 4095, &error);
      if(comm != 0) {
	std::cout << "Failed to loosen brakes for motor " << static_cast<int>(motor.id) << " (" << motor.tool->model_name_ << ")" << std::endl;
      }
    }
    motors_ready_ = true;
    std::cout << "Motors started." << std::endl;
    return true;
  }
  
  
  bool MotorUtilities::stopMotors() {
    UINT8_T error = 0;
    int comm = 0;
    for(auto const& motor: motors_) {
      comm = packet_handler_->Write2ByteTxRx(port_handler_, motor.id, motor.tool->ctrl_table_["external_port_data_1"]->address, 0, &error);
      if(comm != 0) {
	std::cout << "Failed to set brakes for motor " << static_cast<int>(motor.id) << " (" << motor.tool->model_name_ << ")" << std::endl;
      }
    }
    motors_ready_ = false;
    disableTorque();
    return true;
  }
  
  
  bool MotorUtilities::enableTorque() {
    UINT8_T error = 0;
    int comm = 0;
    for(auto const& motor: motors_) {
      comm = packet_handler_->Write1ByteTxRx(port_handler_, motor.id, motor.tool->ctrl_table_["torque_enable"]->address, 1, &error);
      if(comm != 0) {
	std::cout << "Failed to enable torque for motor " << static_cast<int>(motor.id) << " (" << motor.tool->model_name_ << ")" << std::endl;
      }
    }
    torque_enabled_ = true;
    return true;
  }
  
  
  bool MotorUtilities::disableTorque() {
    UINT8_T error = 0;
    int comm = 0;
    for(auto const& motor: motors_) {
      comm = packet_handler_->Write1ByteTxRx(port_handler_, motor.id, motor.tool->ctrl_table_["torque_enable"]->address, 0, &error);
      if(comm != 0) {
	std::cout << "Failed to disable torque for motor " << static_cast<int>(motor.id) << " (" << motor.tool->model_name_ << ")" << std::endl;
      }
    }
    torque_enabled_ = false;
    return true;
  }
  
  
  bool MotorUtilities::write(std::vector<double> commands)
  {
    throw_control_error(commands.size() != motors_.size(), "Wrong number of commands! Got " << commands.size() << ", but expected " << motors_.size());
    
    int i = 0;
    UINT8_T error = 0;
    int comm = 0;
    
    motor_lock_.lock();
    
    //we throw a bunch of own exceptions - let's catch them all and forward them to at all times release the lock
    //Motor 3 has an offset of 228000 ticks - we do not have to treat that one, the dynamiel formware takes care of that
    try {
      for (auto const motor : motors_) {
	switch (current_mode_) {
	case control_modes::ControlMode::POSITION_MODE: {
	  double rad_per_tick = motor.tool->max_radian_ / motor.tool->value_of_max_radian_position_;
	  int goal_position =  static_cast<int>(commands.at(i) / rad_per_tick);
	  int min_ = motor.tool->value_of_min_radian_position_;
	  int max_ = motor.tool->value_of_max_radian_position_;
	  throw_control_error(goal_position < min_ || goal_position > max_,
			      "Motor " << static_cast<int>(motor.id) << " (" << motor.tool->model_name_
			      << ") exceeds its limits [" << min_ << "," << max_
			      << "] with goal position: " << goal_position);

	  comm = packet_handler_->Write4ByteTxRx(port_handler_, motor.id,
					  motor.tool->ctrl_table_["goal_position"]->address,
					  goal_position, &error);	  
	}
	  break;
	  
	case control_modes::ControlMode::VELOCITY_MODE: {
	  UINT32_T velocity_limit;
	  comm = packet_handler_->Read4ByteTxRx(port_handler_, motor.id,
					 motor.tool->ctrl_table_["velocity_limit"]->address,
					 &velocity_limit, &error);
	  if(comm != 0) {
	    std::cout << "Failed to read velocity limit for motor " << static_cast<int>(motor.id) << " (" << motor.tool->model_name_ << ")" << std::endl;
	  }
	  UINT32_T commanded_velocity = (UINT32_T)commands.at(i);
	  throw_control_error( commanded_velocity > velocity_limit,
			       "Motor " << static_cast<int>(motor.id) << " (" << motor.tool->model_name_
			       << ") exceeds its velocity [" << velocity_limit << "] with commanded velocity: " << commanded_velocity);
	  comm = packet_handler_->Write4ByteTxRx(port_handler_, motor.id,
					  motor.tool->ctrl_table_["goal_velocity"]->address,
					  commanded_velocity, &error);
	}
	  break;
	  
	case control_modes::ControlMode::TORQUE_MODE: {
	  UINT16_T torque_limit;
	  comm = packet_handler_->Read2ByteTxRx(port_handler_, motor.id,
					 motor.tool->ctrl_table_["torque_limit"]->address,
					 &torque_limit, &error);
	  if(comm != 0) {
	    std::cout << "Failed to read torque limit for motor " << static_cast<int>(motor.id) << " (" << motor.tool->model_name_ << ")" << std::endl;
	  }
	  UINT16_T commanded_torque = (UINT16_T)commands.at(i);
	  throw_control_error( commanded_torque > torque_limit,
			       "Motor " << static_cast<int>(motor.id) << " (" << motor.tool->model_name_
			       << ") exceeds its velocity [" << torque_limit << "] with commanded velocity: " << commanded_torque);
	  comm = packet_handler_->Write2ByteTxRx(port_handler_, motor.id,
					  motor.tool->ctrl_table_["goal_torque"]->address,
					  commanded_torque, &error);
	}
	  break;
	  
	default:
	  throw_control_error(true, "Unknown mode: " << current_mode_);
	}
	//this theoretically should never be evaluated, but for sake of completeness...
	if(comm != 0){
	  std::cout << "Failed to command motor " << static_cast<int>(motor.id) <<  " (" << motor.tool->model_name_ << ") in mode " << current_mode_ << std::endl;
	}
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
    UINT8_T error = 0;    
    int comm = 0;
    switch(current_mode_) {
    case control_modes::ControlMode::POSITION_MODE:
      {
	std::vector<double> positions = {};
	UINT32_T value;
	for (auto const motor : motors_) {
	  comm = packet_handler_->Read4ByteTxRx(port_handler_, motor.id,
					 motor.tool->ctrl_table_["present_position"]->address,
					 &value, &error);
	  if(comm != 0) {
	    std::cout << "Error reading position for motor " << static_cast<int>(motor.id) << " (" << motor.tool->model_name_ << ")" << std::endl;
	  }
	  double rad_per_tick = motor.tool->max_radian_ / motor.tool->value_of_max_radian_position_;
	  int real_value = static_cast<int>(value);
	  positions.push_back(double(real_value*rad_per_tick));
	}
	
	return positions;
      }
    case control_modes::ControlMode::VELOCITY_MODE:
      {
	std::vector<double> velocities = {};
	UINT32_T value;
	for (auto const motor : motors_) {
	  comm = packet_handler_->Read4ByteTxRx(port_handler_, motor.id,
					 motor.tool->ctrl_table_["present_velocity"]->address,
					 &value, &error);
	  if(comm != 0) {
	    std::cout << "Error reading velocity for motor " << static_cast<int>(motor.id) << " (" << motor.tool->model_name_ << ")" << std::endl;
	  }
	  velocities.push_back(double(value));
	}
	return velocities;
      }
    case control_modes::ControlMode::TORQUE_MODE:
      {
	std::vector<double> torques = {};
	UINT16_T value;
	for (auto const motor : motors_) {
	  comm = packet_handler_->Read2ByteTxRx(port_handler_, motor.id,
					 motor.tool->ctrl_table_["present_current"]->address,
					 &value, &error);
	  if(comm != 0) {
	    std::cout << "Error reading torque for motor " << static_cast<int>(motor.id) << " (" << motor.tool->model_name_ << ")" << std::endl;
	  }
	  torques.push_back(abs(double(value)*CURRENT_TO_TORQUE_RATIO_));
	}
	return torques;
      }
    default:
      throw_control_error(true, "Unknown mode: " << current_mode_);
    }
  }  
}
