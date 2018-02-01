#include <sstream>
#include <ros/ros.h>
#include <std_msgs/Bool.h> 
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <kclhand_control/Definitions.h>
#include <kclhand_control/ActuateHandAction.h>
#include <kclhand_control/kclhand_controller_new.h>

using namespace std;

JointSensor::JointSensor(const ros::NodeHandle &nh, const unsigned int &joint_id)
:nh_(nh),joint_id_(joint_id)
{
  stringstream param_name;
  
  param_name.clear();
  param_name << "joint_" << joint_id_ << "/sensor_calibration";
  if(!nh_.getParam(param_name.str(), joint_sensor_zero_value_))
    throw runtime_error("missing sensor_calibration");
  setJointValueZero(joint_sensor_zero_value_);


  param_name.str("");
  param_name << "joint_" << joint_id_ << "/sensor_direction";
  if(!nh_.getParam(param_name.str(), sensor_direction_))
    throw runtime_error("missing sensor_direction");

  param_name.str("");
  param_name << "joint_" << joint_id_ << "/sensor_range_min";
  if(!nh_.getParam(param_name.str(), joint_value_min_))
    throw runtime_error("missing sensor_range_min");

  param_name.str("");
  param_name << "joint_" << joint_id_ << "/sensor_range_max";
  if(!nh_.getParam(param_name.str(), joint_value_max_))
    throw runtime_error("missing sensor_range_max");

  joint_value_valid_ = true;

  //ROS_INFO("cali %d", sensor_direction_);
}



KCLHandController::	KCLHandController(std::string name)
{
  M_handle_ = 0;
  M_node_id_ = 1;
  M_error_code = 0;

  stringstream param_name;
  
  param_name.clear();
  param_name << "lower_workspace_open_conf";
  if(!nh_.getParam(param_name.str(), lower_workspace_open_conf_))
    throw runtime_error("missing hand lower workspace open configuration");

  param_name.str("");
  param_name << "lower_workspace_close_conf";
  if(!nh_.getParam(param_name.str(), lower_workspace_close_conf_))
    throw runtime_error("missing hand lower workspace close configuration");
  
  param_name.str("");
  param_name << "upper_workspace_open_conf";
  if(!nh_.getParam(param_name.str(), upper_workspace_open_conf_))
    throw runtime_error("missing hand upper workspace open configuration");
  
  param_name.str("");
  param_name << "upper_workspace_close_conf";
  if(!nh_.getParam(param_name.str(), upper_workspace_close_conf_))
    throw runtime_error("missing hand upper workspace close configuration");

  param_name.str("");
  param_name << "lower_to_upper_workspace_seq";
  if(!nh_.getParam(param_name.str(), lower_to_upper_workspace_seq_))
    throw runtime_error("missing hand switching configuration from lower to upper workspace");

  param_name.str("");
  param_name << "upper_to_lower_workspace_seq";
  if(!nh_.getParam(param_name.str(), upper_to_lower_workspace_seq_))
    throw runtime_error("missing hand switching configuration from upper to lower workspace");
  
  param_name.str("");
  param_name << "hand_grasping_current";
  if(!nh_.getParam(param_name.str(), hand_grasping_current_defalut_))
    throw runtime_error("missing hand default current while executing a grasp");

}

KCLHandController:: ~KCLHandController()
{
 bool close_result;
 for(unsigned i = 0; i<NUM_JOINTS ; i++)
 {
   close_result = joints_motor_[i].disableMotor();
 }
 cout << "close device: "<< closeDevice() << endl;

 delete [] joints_sensor_;
 delete [] joints_motor_;
 joints_sensor_ = NULL;
 joints_motor_ = NULL;
}

// consider the motor velocity, current, and if the configuration is valid
bool KCLHandController::moveFingerSrvCB(kclhand_control::MoveFinger::Request &req, kclhand_control::MoveFinger::Response &res)
{
  unsigned int node_id = req.joint_idx;
  double target = req.target;
  bool if_at_target =false;
  int timeout = TIMEOUT_COUNT_FINGER;

  ROS_INFO("Move Finger Server. Motor ID: %d, Target: %.1f", node_id, target);
  if(!hand_is_initialized_)
  {
    res.target_reached = false;
    ROS_INFO("The hand is not initialized");
    return false;
  }   
  //if_at_target = joints_motor_[node_id].moveToTarget(joints_sensor_[node_id].getSensorCalibratedValueRad(), deg_to_rad(target));
  ros::Rate r(MOVE_FINGER_LOOP_FREQ);
  if(joints_motor_[node_id].enableMotor())
  {
    while(!if_at_target)
    { 
      bool enable_motor = joints_motor_[node_id].enableMotor();
      ROS_INFO("Move finger. current joint position: %.1f, motor current: %.1f", joints_sensor_[node_id].getSensorCalibratedValueDeg(), joints_motor_[node_id].getCurrent());     
      if_at_target = joints_motor_[node_id].moveToTarget(joints_sensor_[node_id].getSensorCalibratedValueRad(), deg_to_rad(target));
      r.sleep();
      timeout--;

      if (timeout <=0)
      {
        bool disable_motor = joints_motor_[node_id].disableMotor();      
        ROS_INFO("Time out, can not reach the target");
        res.target_reached = false; 
        res.target_result = joints_sensor_[node_id].getSensorCalibratedValueDeg();
        return false;
      }
    }
  }
  else
  {
    ROS_INFO("Motor ID: %d, can not be enabled", node_id);
    res.target_reached = false; 
    return false;
  }
  res.target_reached = if_at_target; 
  res.target_result = joints_sensor_[node_id].getSensorCalibratedValueDeg();
  bool disable_motor = joints_motor_[node_id].disableMotor();
  return if_at_target;
}

bool KCLHandController::moveFinger(const unsigned &joint_idx, double const &target_position)
{
  unsigned int node_id = joint_idx;
  double target = target_position;
  bool if_at_target =false;
  int timeout = TIMEOUT_COUNT_FINGER;

  ROS_INFO("Move Finger Server. Motor ID: %d, Target: %.1f", node_id, target);
  if(!hand_is_initialized_)
  {
    ROS_INFO("The hand is not initialized");
    return false;
  }   
  //if_at_target = joints_motor_[node_id].moveToTarget(joints_sensor_[node_id].getSensorCalibratedValueRad(), deg_to_rad(target));
  ros::Rate r(MOVE_FINGER_LOOP_FREQ);
  if(joints_motor_[node_id].enableMotor())
  {
    while(!if_at_target)
    { 
      bool enable_motor = joints_motor_[node_id].enableMotor();
      ROS_INFO("Move finger. current joint position: %.1f, motor current: %.1f", joints_sensor_[node_id].getSensorCalibratedValueDeg(), joints_motor_[node_id].getCurrent());     
      if_at_target = joints_motor_[node_id].moveToTarget(joints_sensor_[node_id].getSensorCalibratedValueRad(), deg_to_rad(target));
      r.sleep();
      timeout--;

      if (timeout <=0)
      {
        bool disable_motor = joints_motor_[node_id].disableMotor();      
        ROS_INFO("Time out, can not reach the target");
        return false;
      }
    }
  }
  else
  {
    ROS_INFO("Motor ID: %d, can not be enabled", node_id);
    return false;
  }

  bool disable_motor = joints_motor_[node_id].disableMotor();
  return if_at_target;
}


bool KCLHandController::moveHandSrvCB(kclhand_control::MoveHand::Request &req, kclhand_control::MoveHand::Response &res)
{

  std::vector<double> hand_target;
  int motor_num = NUM_JOINTS;

  if((int)req.target.size() != motor_num)
  {
    ROS_INFO("KCLHandController: wrong number of joint target values received. Please check and try again");
    return false;
  }
    
  for(int i = 0; i<motor_num; i++)
  {
    hand_target.push_back(req.target[i]);
    ROS_INFO("Motor ID: %d, target joint position: %.1f",i, hand_target[i]);
    ROS_INFO("Motor ID: %d, current joint position: %.1f",i, joints_sensor_[i].getSensorCalibratedValueDeg());
  }

  res.target_reached = moveHandToTarget(hand_target) ? true : false;
  
  for(int i = 0; i<motor_num; i++)
  {
    res.target_result.resize(motor_num);
    res.target_result[i] = joints_sensor_[i].getSensorCalibratedValueDeg();
    bool enable_motor = joints_motor_[i].enableMotor();
  }

  return true;
  
  /*
  bool if_at_target =false;
  int at_target_num = 0;
  
  if(!hand_is_initialized_)
    res.target_reached = false; 
  //ros::Rate r(30);
  // /if_at_target = joints_motor_[node_id].moveToTarget(joints_sensor_[node_id].getSensorCalibratedValueRad(), deg_to_rad(target));
  // defibe timeout 
  while(at_target_num < motor_num)
  { 
    at_target_num = 0;
    for(int node_id = 0; node_id<motor_num; node_id++) 
    { 
      double target = hand_target[node_id];
      bool enable_motor = joints_motor_[node_id].enableMotor();
      ROS_INFO("Motor ID: %d, current joint position: %.1f",node_id, joints_sensor_[node_id].getSensorCalibratedValueDeg());     
      if_at_target = joints_motor_[node_id].moveToTarget(joints_sensor_[node_id].getSensorCalibratedValueRad(), deg_to_rad(target));
      if(if_at_target)
        at_target_num++;
    }
    //r.sleep();
  }
  */



  //return target position, disable motors when target is reached to save power and erase noise

  


}


bool KCLHandController::moveHandToTarget(const std::vector<double> &target)
{
  int timeout = TIMEOUT_COUNT_MOVE_HAND;
  int motor_num = NUM_JOINTS;
  if((int)target.size() != motor_num)
  {
    ROS_INFO("KCLHandController: wrong number of joint target values received. Please check and try again");
    return false;
  }

  for(int i = 0; i<motor_num; i++)
  {
    ROS_INFO("Motor ID: %d, target joint position: %.1f",i, target[i]);
    ROS_INFO("Motor ID: %d, current joint position: %.1f",i, joints_sensor_[i].getSensorCalibratedValueDeg());
  }

  if(!hand_is_initialized_)
    return false; 

  ros::Rate r(MOVE_HAND_LOOP_FREQ);
  int at_target_num = 0;
  bool if_at_target =false;

  while(at_target_num < motor_num)
  { 
    at_target_num = 0;
    for(int node_id = 0; node_id<motor_num; node_id++) 
    { 
      double target_pos = target[node_id];
      bool enable_motor = joints_motor_[node_id].enableMotor();
      ROS_INFO("Motor ID: %d, current joint position: %.1f. current: %.1f",node_id, joints_sensor_[node_id].getSensorCalibratedValueDeg(),joints_motor_[node_id].getCurrent());     
      if_at_target = joints_motor_[node_id].moveToTarget(joints_sensor_[node_id].getSensorCalibratedValueRad(), deg_to_rad(target_pos));
      if(if_at_target)
        at_target_num++;
    }
    r.sleep();
    timeout--;

    if (timeout <=0)
    {
      for(int node_id = 0; node_id<motor_num; node_id++) 
      {
        bool disable_motor = joints_motor_[node_id].disableMotor();
      }      
      at_target_num = 6;
      ROS_INFO("Time out, can not reach the target");
      return false;
    }
  }
  return true;
}


bool KCLHandController::openHand()
{

  std::vector<double> hand_open_target;
  bool palm_sigularity_flag = false;

  // if the hand is in lower workspace
  if((joints_sensor_[0].getSensorCalibratedValueDeg() < -20.0) && (joints_sensor_[1].getSensorCalibratedValueDeg() < -20.0))
  {
    hand_open_target = lower_workspace_open_conf_;
  }
  // if the hand is in upper workspace
  else if((joints_sensor_[0].getSensorCalibratedValueDeg() > 20.0) && (joints_sensor_[1].getSensorCalibratedValueDeg() > 20.0))
  {
    hand_open_target = upper_workspace_open_conf_;
  }
  
  else 
  {
    for (unsigned node_id = 0; node_id< NUM_JOINTS; node_id++)
    {
      hand_open_target.push_back(joints_sensor_[node_id].getSensorCalibratedValueDeg());
    }
    hand_open_target[2] = upper_workspace_open_conf_[2];
    hand_open_target[3] = upper_workspace_open_conf_[3];
    hand_open_target[4] = upper_workspace_open_conf_[4];
  }


  bool succeeded = moveHandToTarget(hand_open_target) ? true : false;
  ROS_INFO("If the hand is opened: %d", succeeded);
  /*
  for (unsigned int i = 0; i< NUM_JOINTS; i++)
  {
       ROS_INFO("Disable motor ID: %d", i);
       bool disable_motor = joints_motor_[i].disableMotor();     
  }
  */
  return succeeded;

}

bool KCLHandController::foldHand()
{ 

  std::vector<double> fold_hand_target;
  fold_hand_target = lower_workspace_open_conf_;
  bool fold_hand_suc = false;
  // if the hand is in upper workspace
  if((joints_sensor_[0].getSensorCalibratedValueDeg() > 20.0) && (joints_sensor_[1].getSensorCalibratedValueDeg() > 20.0))
  {
    //go to lower workspace first
    ROS_INFO("The hand is in upper workspace, so move it to lower workspace first");
    //switch workspaces
    bool if_suc = upperToLowerWorkspace();
    if(!if_suc)
    {
      ROS_INFO("Hand moves to lower workspace failed!");
      return false;
    } 
  }

  bool succeeded = moveHandToTarget(fold_hand_target) ? true : false;
  if(!succeeded)
  {
    ROS_INFO("The hand is in lower workspace, but cannot fold");
    return false;
  }
  
  fold_hand_suc = moveFinger (3, 80.0);
  if(fold_hand_suc)
  {
    fold_hand_suc = moveFinger(2, 35.0);
  }
  if(fold_hand_suc)
  {
    fold_hand_suc = moveFinger(4, 28.0);
  }

  for (unsigned int i = 0; i< NUM_JOINTS; i++)
  {
    ROS_INFO("Disable motor ID: %d", i);
    bool test1 = joints_motor_[i].disableMotor();     
  }

  if(!fold_hand_suc)
    return false;
  else 
    return fold_hand_suc;
}


bool KCLHandController::upperToLowerWorkspace()
{
  
  if((joints_sensor_[0].getSensorCalibratedValueDeg() < -20.0) && (joints_sensor_[1].getSensorCalibratedValueDeg() < -20.0))
  {
    ROS_INFO("The hand is in lower workspace now");
    return false;
  }


  int steps = upper_to_lower_workspace_seq_.size() / NUM_JOINTS;
  //ROS_INFO("the step is: %d", steps);
  int step_count = 0;
  std::vector<double> target_positions(NUM_JOINTS);
  for (int i = 0; i < steps; i++)
  {
    for(int j = 0 ; j < NUM_JOINTS; j++)
    {
      target_positions[j] = upper_to_lower_workspace_seq_[i*NUM_JOINTS+j];
    }

    bool succeeded = moveHandToTarget(target_positions) ? true : false;
    if (!succeeded)
    {
      ROS_INFO("The hand can not move from upper workspace to lower workspace");
      break;
    }
    step_count++;
  }

  if (steps == step_count)
  {
    ROS_INFO("move from upper workspace to lower workspace done!");
    return true;
  }
  else
  {
    ROS_INFO("moves from upper workspace to lower workspace faild!");
    return false;
  }
}


bool KCLHandController::lowerToUpperWorkspace()
{

  if((joints_sensor_[0].getSensorCalibratedValueDeg() > 20.0) && (joints_sensor_[1].getSensorCalibratedValueDeg() > 20.0))
  {
    ROS_INFO("The hand is in upper workspace now");
    return false;
  }
  
  int steps = lower_to_upper_workspace_seq_.size() / NUM_JOINTS;
  int step_count = 0;
  std::vector<double> target_positions(NUM_JOINTS);
  for (int i = 0; i < steps; i++)
  {
    for(int j = 0 ; j < NUM_JOINTS; j++)
    {
      target_positions[j] = lower_to_upper_workspace_seq_[i*NUM_JOINTS+j];
      //ROS_INFO("ID: %d, target: %3f", j, target_positions[j]);
    }

    bool succeeded = moveHandToTarget(target_positions) ? true : false;
    if (!succeeded)
    {
      ROS_INFO("The hand can not move from upper workspace to lower workspace");
      break;
    }
    step_count++;
  }

  if (steps == step_count)
  {
    ROS_INFO("move from lower workspace to upper workspace done!");
    return true;
  }
  else
  {
    ROS_INFO("moves from lower workspace to upper workspace faild!");
    return false;
  }
}



bool KCLHandController::handModeSrvCB(kclhand_control::HandOperationMode::Request &req, kclhand_control::HandOperationMode::Response &res)
{
   unsigned int mode = req.operation_mode;

   //ROS_INFO("Move Finger Server. Motor ID: %d, Target: %f", node_id, target);
  if(!hand_is_initialized_)
    res.result = 0;

  if(mode == 0) //disable all motors
  {
    ROS_INFO("Mode = 0");
    for (unsigned int i = 0; i< NUM_JOINTS; i++)
    {
       ROS_INFO("Disable motor ID: %d", i);
       bool test1 = joints_motor_[i].disableMotor();     
    }
    res.result = 1;
  }

  if(mode == 1) //enable all motors
  {
    for (unsigned int i = 0; i< NUM_JOINTS; i++)
    {
      bool test2 = joints_motor_[i].enableMotor();
      ROS_INFO("Enable Motor ID: %d", i);
    }  
    res.result = 1;
  }

  if(mode == 2)
  {
    // grasping 
    bool hand_grasping = closingHandForGrasing();
    res.result = hand_grasping;
  }


  if(mode == 3)
  {
    ROS_INFO("Open hand running");
    res.result = openHand();
  }

  if(mode == 4)
  {
    bool if_suc = upperToLowerWorkspace();
    res.result = if_suc;
  }

  if(mode == 5)
  {
    bool if_suc = lowerToUpperWorkspace();
    res.result = if_suc;
  }

  if(mode == 6)
  {
    for (unsigned int i = 0; i< NUM_JOINTS; i++)
    {
      ROS_INFO("Enable Motor ID: %d, is motor enabled: %d", i, joints_motor_[i].getMotorEnableState());
    }  
    res.result = 1;
  }

 if(mode ==9) 
 {
    ROS_INFO("start folding the hand");
    bool foldhand_result = foldHand();
    res.result = foldhand_result;
 }

  

/*
  // set motor following errors 
  if(mode == 6)
  {
    for (unsigned int i = 0; i< NUM_JOINTS; i++)
    {
       joints_motor_[i].setMaxFollowingError();   
    }
  }
  
  // read motor default parameters
  if(mode == 7)
  {
    for (unsigned int i = 0; i< NUM_JOINTS; i++)
    {
       joints_motor_[i].getDCMotorParameter(); 
       ROS_INFO("motor ID: %d, nominal_current: %d, max_output_current_: %d, thermal_time_constant_: %d, max_following_error: %d", i, 
        joints_motor_[i].getNominalCurrent(), joints_motor_[i].getMaxOutputCurrent(), joints_motor_[i].getThermalTimeConstant(), 
        joints_motor_[i].getMaxFollowingError());   
    }
  }
  
  // set motor parameters to apply a larger force 
  if(mode == 8)
  {
    for (unsigned int i = 0; i< NUM_JOINTS; i++)
    {
       joints_motor_[i].setDCMotorParameter(); 
       //ROS_INFO("motor ID: %d, nominal_current: %d, max_output_current_: %d, thermal_time_constant_: %d", i, 
        //oints_motor_[i].nominal_current_, joints_motor_[i].max_output_current_, joints_motor_[i].thermal_time_constant_);   
    }
  }

*/


  

   //bool success = enableMotor();

   return true;
}

// initilize the hand, open the epos2 controller 
bool KCLHandController::init()
{
	hand_is_initialized_ = false;
  has_new_goal_ = false;
	
  // ros setting, 
  joint_raw_value_sub_ = nh_.subscribe("joint_value_arduino", 1, &KCLHandController::jointSensorValueCB, this);
	joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("active_joint_states", 1);
	move_finger_srv_server_ = nh_.advertiseService("move_finger", &KCLHandController::moveFingerSrvCB, this);
  move_hand_srv_server_ = nh_.advertiseService("move_hand", &KCLHandController::moveHandSrvCB, this);
  metahand_mode_srv_server_ = nh_.advertiseService("hand_operation_mode", &KCLHandController::handModeSrvCB, this);
 

  // epos motor setting 
  stringstream param_name;

  param_name.clear();
  param_name << "hand_controller_device_name";
  if(!nh_.getParam(param_name.str(), M_DEVICENAME_NAME_))
    throw runtime_error("missing ROS parameter for hand_controller_device_name");

  param_name.str("");
  param_name << "hand_controller_protocol_stack_name";
  if(!nh_.getParam(param_name.str(), M_PROTOCAL_NAME_))
    throw runtime_error("missing ROS parameter for hand_controller_protocol_stack_name");

  param_name.str("");
  param_name << "hand_controller_interface_name";
  if(!nh_.getParam(param_name.str(), M_INTERFACE_))
    throw runtime_error("missing ROS parameter for hand_controller_interface_name");

  param_name.str("");
  param_name << "hand_controller_port_name";
  if(!nh_.getParam(param_name.str(), M_PORTNAME_))
    throw runtime_error("missing ROS parameter for hand_controller_port_name");

  param_name.str("");
  param_name << "hand_controller_baudrate";
  if(!nh_.getParam(param_name.str(), M_BAUDRATE_))
    throw runtime_error("missing ROS parameter for hand_controller_baudrate");

  if(openDevice())
    ROS_INFO("Deviced opened");
  else
  {
    ROS_INFO("Deviced not opened");
    return false;
  }
    
  joints_sensor_ = new JointSensor[NUM_JOINTS] { JointSensor(nh_,0),
                                                 JointSensor(nh_,1),
                                                 JointSensor(nh_,2),
                                                 JointSensor(nh_,3),
                                                 JointSensor(nh_,4)};
  

  joints_motor_ = new JointMotor[NUM_JOINTS]{ JointMotor(nh_,M_handle_,0), 
                                              JointMotor(nh_,M_handle_,1),
                                              JointMotor(nh_,M_handle_,2),
                                              JointMotor(nh_,M_handle_,3),
                                              JointMotor(nh_,M_handle_,4)};
  

  //set motor default parameters

  for (unsigned int i = 0; i< NUM_JOINTS; i++)
    {
       joints_motor_[i].setMaxFollowingError(); 
       joints_motor_[i].setDCMotorParameter();  
    }

  hand_is_initialized_ = true;
  return true;
}

// build the controller communication and open controller, 
// return 1 means successful
// return 0 means faild 
bool KCLHandController::openDevice()
{
  bool if_controller_open = false;

  char* device_name = new char[255];
  char* protocol_stack_name = new char[255];
  char* interface_name = new char[255];
  char* port_name = new char[255];

  strcpy(device_name, M_DEVICENAME_NAME_.c_str());
  strcpy(protocol_stack_name, M_PROTOCAL_NAME_.c_str());
  strcpy(interface_name, M_INTERFACE_.c_str());
  strcpy(port_name, M_PORTNAME_.c_str());

  ROS_INFO("Open the Metahand Controller ");

  M_handle_ = VCS_OpenDevice(device_name, protocol_stack_name, interface_name, port_name, &M_error_code);

  /* Print errors
  stringstream msg;
  msg << "M_handle" << M_handle_ << "'\n";
  msg << "error code" << M_error_code << "'\n";
  ROS_INFO("%s", msg.str().c_str());
  */

  if(M_handle_!=0 && M_error_code == 0)
  {
    unsigned int baudrate = 0;
    unsigned int timeout = 0;

    if(VCS_GetProtocolStackSettings(M_handle_, &baudrate, &timeout, &M_error_code)!=0)
    {
      if(VCS_SetProtocolStackSettings(M_handle_, M_BAUDRATE_, timeout, &M_error_code)!=0)
      {
        if(VCS_GetProtocolStackSettings(M_handle_, &baudrate, &timeout, &M_error_code)!=0)
        {
          if(M_BAUDRATE_==(int)baudrate)
          {
            if_controller_open = true;
          }
        }
      }
    }
  }
  
  else
  {
    M_handle_ = 0;
  }

  delete []device_name;
  delete []protocol_stack_name;
  delete []interface_name;
  delete []port_name;
  return if_controller_open;
}


bool KCLHandController::closingHandForGrasing()
{
  std::vector<double> hand_grasping_target(NUM_JOINTS);
  
  // if the hand is in lower workspace
  if((joints_sensor_[0].getSensorCalibratedValueDeg() < -30.0) && (joints_sensor_[1].getSensorCalibratedValueDeg() < -30.0))
  {
    hand_grasping_target = lower_workspace_close_conf_;
  }

  // if the hand is in upper workspace
  else if((joints_sensor_[0].getSensorCalibratedValueDeg() > 20.0) && (joints_sensor_[1].getSensorCalibratedValueDeg() > 20.0))
  {
    hand_grasping_target = upper_workspace_close_conf_;
  }
  else 
  {
    ROS_INFO("The palm is near singular configuration, not suitable for grasping");
    return false;
  }


  int motor_num = NUM_JOINTS;  
  unsigned int node_id = 0;
  double grasping_current_set = hand_grasping_current_defalut_; //for vienna hand, the valie is 150. move it to yaml file
  bool if_object_grasped = false;
  int at_target_num = 0;
  int current_exceed_num = 0;
  bool target = 0.;
  bool if_at_target;
  int timeout = TIMEOUT_COUNT_GRASPING;

  ros::Rate r(MOVE_HAND_LOOP_FREQ);
  if(!hand_is_initialized_)
    return false; 

  // Michael Zillich: enable only once
  for(node_id = 0; node_id<motor_num; node_id++) 
    joints_motor_[node_id].enableMotor();
 
  while(at_target_num < NUM_JOINTS)
  { 
    at_target_num = 0;
    for(node_id = 0; node_id<motor_num; node_id++) 
    { 
      double target = hand_grasping_target[node_id];
      // Michael Zillich: why enable in _each_ iteration?
      // bool enable_motor = joints_motor_[node_id].enableMotor();
      ROS_INFO("joint ID: %d, current joint position: %.1f, motor current: %.1f", node_id, joints_sensor_[node_id].getSensorCalibratedValueDeg(), joints_motor_[node_id].getCurrent());     
      if_at_target = joints_motor_[node_id].moveToTarget(joints_sensor_[node_id].getSensorCalibratedValueRad(), deg_to_rad(target));     
      
      if(if_at_target)
      {
        at_target_num++;
        continue;
      }
      
      if(fabs(joints_motor_[node_id].getCurrent())>grasping_current_set)
      {
        joints_motor_[node_id].haltVelocityMovement();
        at_target_num++;
      } 
    }

    r.sleep();
    timeout--;

    if (timeout <=0)
    {
      for(node_id = 0; node_id<motor_num; node_id++) 
      {
        /* Michael Zillich: Why enable the already enabled motors in case of a timeout?
         * And why do it twice?
         * We probably rather want to disable in such a case, as happens in moveHandToTarget()
        bool enable_motor = joints_motor_[node_id].enableMotor();
        if(!enable_motor) 
          enable_motor = joints_motor_[node_id].enableMotor();*/
        joints_motor_[node_id].haltVelocityMovement();
        bool disable_motor = joints_motor_[node_id].disableMotor();
      } 
      
      at_target_num = 6;
      ROS_INFO("Time out");
      return false;
    }

    ROS_INFO("at target value is %d, time out is %d",at_target_num, timeout);
  }

  return true;
}




// Joint motor 
JointMotor::JointMotor(const ros::NodeHandle &nh, void *epos_handle, const unsigned int &joint_id)
:epos_handle_(epos_handle),nh_(nh),motor_node_id_(joint_id)
{
  stringstream param_name;
  
  param_name.clear();
  param_name << "joint_" << joint_id << "/node_id";
  if(!nh_.getParam(param_name.str(), motor_node_id_))
    {}//throw runtime_error("missing motor node ID for motor ID: %d", motor_id);

  param_name.str("");
  param_name << "joint_" << joint_id << "/motor_direction";
  if(!nh_.getParam(param_name.str(), motor_direction_))
    {}//throw runtime_error("missing motor node ID for motor ID: %d", motor_id);

  param_name.str("");
  param_name << "joint_" << joint_id << "/moving_velocity";
  if(!nh_.getParam(param_name.str(), motor_moving_velocity_))
    {}//throw runtime_error("missing motor node ID for motor ID: %d", motor_id);

  motor_velocity_ = 0;
  motor_move_current_ = 0;
  motor_holding_current_ = 0;
  target_position_ = 0;

  max_peak_current_ = 0;
  motor_running_ = false;

  //motor_moving_velocity_ = 300;

}

bool JointMotor::getMotorEnableState()
{
  int is_motor_enabled = false ;
  unsigned int error_code = 0; 
  if(VCS_GetEnableState(epos_handle_, motor_node_id_, &is_motor_enabled, &error_code) == 0)
  {
    ROS_INFO("Faild to get motor ID: %d enable state", motor_node_id_);
    return false;
  }

  return is_motor_enabled;

}




// enable motor
bool JointMotor::enableMotor()
{
  int is_motor_enabled = false ;
  unsigned int error_code = 0; 
  int is_motor_fault = 0; // 1: Device is in fault state  0: Device is not in fault stat

  if(VCS_GetFaultState(epos_handle_, motor_node_id_, &is_motor_fault, &error_code) == 0)
  {
    ROS_INFO("Faild to get motor ID: %d fault state", motor_node_id_);
    return is_motor_enabled;
  }

  if(is_motor_fault) // if motor has fault. then clear motor fault
  {
    stringstream msg;
    msg << "clear fault, node = '" << motor_node_id_ << "'";
    ROS_INFO("%s", msg.str().c_str());

    if(VCS_ClearFault(epos_handle_, motor_node_id_, &error_code) == 0)
    {
      ROS_INFO("Faild to clear motor ID: %d fault", motor_node_id_);
      return is_motor_enabled;
    }
  }
  
  // if no fault, then get the motor enable state
  if(VCS_GetEnableState(epos_handle_, motor_node_id_, &is_motor_enabled, &error_code) == 0)
  {
    ROS_INFO("Faild to get motor ID: %d enable state", motor_node_id_);
    return false;
  }

  // if motor is not enabled, then enable the motor
  if(!is_motor_enabled)
  {
    if(VCS_SetEnableState(epos_handle_, motor_node_id_, &error_code) == 0)
    {
      ROS_INFO("Faild to set motor ID: %d enable state", motor_node_id_);
      return false;
    }
    is_motor_enabled = true;

  }
  // if motor is enabled, then return ture
  return is_motor_enabled;
}


// disable motor
bool JointMotor::disableMotor()
{
  int is_motor_disabled = false ;
  unsigned int error_code = 0; 
  int is_motor_fault = 0; // 1: Device is in fault state  0: Device is not in fault stat

  if(VCS_GetFaultState(epos_handle_, motor_node_id_, &is_motor_fault, &error_code) == 0)
  {
    ROS_INFO("Faild to get motor ID: %d fault state", motor_node_id_);
    return is_motor_disabled;
  }

  if(is_motor_fault) // if motor has fault. then clear motor fault
  {
    stringstream msg;
    msg << "clear fault, node = '" << motor_node_id_ << "'";
    ROS_INFO("%s", msg.str().c_str());

    if(VCS_ClearFault(epos_handle_, motor_node_id_, &error_code) == 0)
    {
      ROS_INFO("Faild to clear motor ID: %d fault", motor_node_id_);
      return is_motor_disabled;
    }
  }
  
  // if no fault, then get the motor disable state
  if(VCS_GetDisableState(epos_handle_, motor_node_id_, &is_motor_disabled, &error_code) == 0)
  {
    ROS_INFO("Faild to get motor ID: %d disable state", motor_node_id_);
    return false;
  }

  // if motor is not disabled, then disable the motor
  if(!is_motor_disabled)
  {
    if(VCS_SetDisableState(epos_handle_, motor_node_id_, &error_code) == 0)
    {
      ROS_INFO("Faild to set motor ID: %d disable state", motor_node_id_);
      return false;
    }
        
  }
  // if motor is disabled, then return ture
  return is_motor_disabled;
}

void JointMotor::reset()
{
  unsigned int error_code = 0;
  if(VCS_ResetDevice(epos_handle_, motor_node_id_, &error_code) == 0)
  {
    ROS_INFO("Hand joint reset node ID: %d faild", motor_node_id_);
    throw runtime_error("reset node faild"); 
  }
}


void JointMotor::activateVelocityMode()
{
  unsigned int error_code = 0;
  if(VCS_ActivateProfileVelocityMode(epos_handle_, motor_node_id_, &error_code) == 0)
  {
    ROS_INFO("Metahand: activate profile velocity mode faild, motor ID: %d ", motor_node_id_);
    throw runtime_error("MetaHand: faild to activate profile velocity mode"); 
  }
}

unsigned int JointMotor::getMaxFollowingError()
{
  unsigned int error_code = 0;
  unsigned int max_following_error = 0;
  
  if(VCS_GetMaxFollowingError(epos_handle_, motor_node_id_, &max_following_error, &error_code) == 0)
  {
    ROS_INFO("Metahand: Get Maximal Following Error faild, motor ID: %d ", motor_node_id_);
    throw runtime_error("MetaHand: faild to Get Maximal Following Error"); 
  }

  return max_following_error;
}

void JointMotor::setMaxFollowingError()
{
  unsigned int error_code = 0;
  unsigned int max_following_error = 20000;
  
  if(VCS_SetMaxFollowingError(epos_handle_, motor_node_id_, max_following_error, &error_code) == 0)
  {
    ROS_INFO("Metahand: Set Maximal Following Error faild, motor ID: %d ", motor_node_id_);
    throw runtime_error("MetaHand: faild to Set Maximal Following Error"); 
  }

}

void JointMotor::getDCMotorParameter()
{
  unsigned int error_code = 0;
  
  if(VCS_GetDcMotorParameter(epos_handle_, motor_node_id_, &nominal_current_, &max_output_current_, &thermal_time_constant_, &error_code) == 0)
  {
    ROS_INFO("Metahand: Get Dc Motor Parameter faild, motor ID: %d ", motor_node_id_);
    throw runtime_error("MetaHand: faild to Get Dc Motor Parameter"); 
  }

}

void JointMotor::setDCMotorParameter()
{
  unsigned int error_code = 0;
  short unsigned int nominal_current = (motor_node_id_ == 1 || motor_node_id_ == 3) ? NOMINAL_CURRENT_PALM : NOMINAL_CURRENT_FINGER;
 
  //short unsigned int nominal_current = NOMINAL_CURRENT_PALM;
  short unsigned int max_output_current = MAX_OUTPUT_CURRENT;
  short unsigned int thermal_time_constant = THERMAL_TIME_CONSTANT;

  if(VCS_SetDcMotorParameter(epos_handle_, motor_node_id_, nominal_current, max_output_current, thermal_time_constant, &error_code) == 0)
  {
    ROS_INFO("Metahand: Set Dc Motor Parameter faild, motor ID: %d ", motor_node_id_);
    throw runtime_error("MetaHand: faild to Set Dc Motor Parameter"); 
  }

}


void JointMotor::moveWithVelocity(double velocity)
{
  unsigned int error_code = 0;
  long velocity_with_direction =  (long)velocity*motor_direction_;
  if(VCS_MoveWithVelocity(epos_handle_, motor_node_id_, velocity_with_direction, &error_code) == 0)
  {
    ROS_INFO("Metahand: move with velocity faild, motor ID: %d, velocity: %ld", motor_node_id_, velocity_with_direction);
    throw runtime_error("Metahand: failed to move with velocity"); 
  }
    
}

void JointMotor::haltVelocityMovement()
{
  unsigned int error_code = 0;
  if(VCS_HaltVelocityMovement(epos_handle_, motor_node_id_, &error_code) == 0)
  {
    ROS_INFO("Metahand: halt velocity movement faild, motor ID: %d", motor_node_id_);
    throw runtime_error("Metahand: failed to halt velocity movement"); 
  }
}

void JointMotor::activateCurrentMode()
{
  unsigned int error_code = 0;
  if(VCS_ActivateCurrentMode(epos_handle_, motor_node_id_, &error_code) == 0)
  {
    ROS_INFO("Metahand: activate current mode faild, motor ID: %d", motor_node_id_);
    throw runtime_error("Metahand: failed to activate current mode"); 
  }  
}

void JointMotor::setCurrent(double current)
{
  unsigned int error_code = 0;
  int actual_current = current*motor_direction_;
  if(VCS_SetCurrentMust(epos_handle_, motor_node_id_, (short)actual_current, &error_code) == 0)
  {
    ROS_INFO("Metahand: set current mode faild, motor ID: %d, current: %d", motor_node_id_, actual_current);
    throw runtime_error("Metahand: failed to set current"); 
  }  
}

double JointMotor::getCurrent()
{
  unsigned int error_code = 0;
  short current = 0.;
  if(VCS_GetCurrentIsAveraged(epos_handle_, motor_node_id_, &current, &error_code) == 0)
  {
    ROS_INFO("Metahand: get current faild, motor ID: %d", motor_node_id_);
    throw runtime_error("Metahand: failed to get motor current"); 
  }  
  return (double)current;
}



bool JointMotor::moveToTarget(double current_position, double target)
{

  target_position_ = target;
  current_joint_position_ = current_position;
  int direction;

  bool motor_state = enableMotor();
  if (!isTargetReached())
  {
    if (current_joint_position_ < target_position_)
      direction = 1;
    else if (current_joint_position_ > target_position_)
      direction = -1;
    activateVelocityMode();
    motor_velocity_ = motor_moving_velocity_ * direction;
    moveWithVelocity(motor_velocity_);   
  }
  else
  {
    motor_velocity_ = 0;
    haltVelocityMovement();
  }

  return is_target_reached;
}



bool KCLHandController::closeDevice()
{
  bool is_device_closed = false;

  M_error_code = 0;

  ROS_INFO("Close device ing");

  if(VCS_CloseDevice(M_handle_, &M_error_code)!=0 && M_error_code == 0)
  {
    is_device_closed = true;
  }

  return is_device_closed;
}


//To do  if one wire borken, arduino shoud retrun a value or flag, maybe add a resistor?
void KCLHandController::jointSensorValueCB(const std_msgs::Int16MultiArray::ConstPtr &msg)
{
  
  static unsigned int seq = 0;

  if((int)msg->data.size() != NUM_JOINTS)
    throw runtime_error("KCLHandController: wrong number of joint position values received");

  // Whenever we get a position update, we also want a current/effort update.
  // joint_mutex_.lock();
  sensor_msgs::JointState joint_state;

  joint_state.header.stamp = ros::Time::now();
  joint_state.position.resize(NUM_JOINTS);
  for(unsigned int i = 0; i < NUM_JOINTS; i++)
  {
    joints_sensor_[i].sensorCalibratedValueUpdate(msg->data[i]);
    joints_sensor_[i].checkJointValueRange();
    if (!joints_sensor_[i].getIfJointValueValid())
    {
      ROS_INFO("The Motor ID: %d, angle value: %3f, exceeds the limit", i, joints_sensor_[i].getSensorCalibratedValueDeg());
      //throw runtime_error("Metahand: the joint anlge exceeds the limit "); 
      //should here go the recover function
    }
    //joint_state.position[i] = joints_sensor_[i].getSensorCalibratedValueDeg();
    joint_state.position[i] = joints_sensor_[i].getSensorCalibratedValueDeg();

  }

  //joint_mutex_.unlock();
  joint_state_pub_.publish(joint_state);
  //ROS_INFO("joint sensor done %f", joints_sensor_[1].joint_sensor_zero_value_);
  //ROS_INFO("joint sensor direction %d", joints_sensor_[1].sensor_direction_);
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "kclhand_controller");
  
  KCLHandController controller("actuate_hand");
  
  if(!controller.init())
  	return 0;
  ros::AsyncSpinner spinner(2); // Use 2 threads
  spinner.start();
  ros::waitForShutdown();
  //ros::Rate loop_rate(50); 
  //ros::spin();	
  return 0;
}
