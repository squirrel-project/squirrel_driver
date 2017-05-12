//
// Created by Philipp Zech on 20.04.17.
//


#include "squirrel_control/squirrel_hw_control_loop.h"
#include "squirrel_control/squirrel_hw_interface.h"


int main(int argc, char** argv) {   ros::init(argc, argv, "squirrel_hw_interface");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    boost::shared_ptr<squirrel_control::SquirrelHWInterface> squirrel_hw_interface
           (new squirrel_control::SquirrelHWInterface(nh));
    squirrel_hw_interface->init();

    squirrel_control::SquirrelHWControlLoop control_loop(nh, squirrel_hw_interface);

    ros::waitForShutdown();

    return 0;
}
