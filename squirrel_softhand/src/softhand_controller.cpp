#include "ros/ros.h"
#include "squirrel_manipulation_msgs/SoftHandGrasp.h"
#include "math.h"
#include "signal.h"
#include "stdlib.h"
#include "../include/qbmove_communications.h"

struct global_args {
	int device_id;
	comm_settings comm_settings_t;
} global_args;


void my_handler(int s){
	ROS_INFO("Received signal: %d", s);
	short int inputs[2];
	inputs[0] = 0;
	inputs[1] = 0;
	commSetInputs(&global_args.comm_settings_t, global_args.device_id, inputs);
	commActivate(&global_args.comm_settings_t, global_args.device_id, 0);
	closeRS485(&global_args.comm_settings_t);
	exit(EXIT_SUCCESS);
}


bool actuate(squirrel_manipulation_msgs::SoftHandGrasp::Request & req,
		squirrel_manipulation_msgs::SoftHandGrasp::Response & res)
{
	short int inputs[2];
	inputs[0] = ceil(req.position*17000.0);
	ROS_INFO("Grasp position: %d", inputs[0]);
	inputs[1] = 0;    
	commSetInputs(&global_args.comm_settings_t, global_args.device_id, inputs);
	if(global_args.comm_settings_t.file_handle == INVALID_HANDLE_VALUE)
	{
		ROS_ERROR("Couldn't move hand!");
		res.success = false;
		return EXIT_FAILURE;
	}
	res.success = true;
	return true;    
}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "squirrel_softhand");
	ros::NodeHandle n;

	signal(SIGINT, my_handler);

	char ports[10][255];
	RS485listPorts(ports);
	std::string port;
	ros::param::param<std::string>("port", port, ports[0]);
	ROS_INFO("Selected port: %s", port.c_str());

	ros::param::param<int>("device_id", global_args.device_id, 1);
	ROS_INFO("Device ID: %d", global_args.device_id);

	// CONNECT
	openRS485(&global_args.comm_settings_t, port.c_str());
	if(global_args.comm_settings_t.file_handle == INVALID_HANDLE_VALUE)
	{
		ROS_ERROR("Couldn't connect to serial port: %p", port.c_str());
		return EXIT_FAILURE;
	} else {
		ROS_INFO("Connected to %s.", port.c_str());
	}    
	commActivate(&global_args.comm_settings_t, global_args.device_id, 1);    
	ros::ServiceServer service = n.advertiseService("softhand_grasp", actuate);
	ROS_INFO("Ready to grasp.");

	ros::spin();

	// SHUTDOWN
	short int inputs[2];
	inputs[0] = 0;
	inputs[1] = 0;
	commSetInputs(&global_args.comm_settings_t, global_args.device_id, inputs);
	commActivate(&global_args.comm_settings_t, global_args.device_id, 0);
	closeRS485(&global_args.comm_settings_t);

	return EXIT_SUCCESS;
}
