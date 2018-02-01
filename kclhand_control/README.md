# kclhand control 

## Create the udev rules to run the hand controllers without root permission
Go to the /udev folder, Either run the script create_udev_rules.sh or use the following commands:
	
	sudo cp metahand_usb.rules /etc/udev/rules.d/
	
	sudo /etc/init.d/udev restart

## Control the Metahand
The Metahand is controlled through ROS services, so use service calls to run the hand
Here are the several functions

### 1. Hand control via state machine 
	
	rosservice call /hand_operation_mode "operation_mode: mode_id"

mode_id = 0, disable all motors
mode_id = 1, enable all motors
mode_id = 2, close the hand for a grasping action
mode_id = 3, open hand
mode_id = 4, switch hand configuration from the upper workspace to lower workspace
mode_id = 5, switch hand configuration from the lower workspace to upper workspace
mode_id = 6, print hand motor enable status
mode_id = 9, fold hand


So when launching the hand, please run the services with the following sequences 
1. mode_id = 3, open hand. 
2. mode_id = 5, switch hand configuration from the lower workspace to upper workspace 
3. mode_id = 3, open hand.  （for preparing a grasp）
4. mode_id = 2, close hand.  （grasp）

when folding the hand, just use 
mode_id = 9, fold hand



### 2. Finger position control
	
	rosservice call /move_finger 

### 3. Hand position control
	
	rosservice call /move_hand


