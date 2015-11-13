// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <squirrel_sensor_tester/sensor_tester_nodeConfig.h>

// ROS message includes
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>

// other includes
#include <sensor_tester_node_common.cpp>


class sensor_tester_node_ros
{
    public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    dynamic_reconfigure::Server<squirrel_sensor_tester::sensor_tester_nodeConfig> server;
    dynamic_reconfigure::Server<squirrel_sensor_tester::sensor_tester_nodeConfig>::CallbackType f;

    ros::Subscriber TestPublish_;
    ros::ServiceServer TestService_;

    sensor_tester_node_data component_data_;
    sensor_tester_node_config component_config_;
    sensor_tester_node_impl component_implementation_;

    sensor_tester_node_ros() : np_("~")
    {
        f = boost::bind(&sensor_tester_node_ros::configure_callback, this, _1, _2);
        server.setCallback(f);

        std::string TestService_remap;
        n_.param("TestService_remap", TestService_remap, (std::string)"TestService");
        TestService_ = n_.advertiseService<std_srvs::Empty::Request , std_srvs::Empty::Response>(TestService_remap, boost::bind(&sensor_tester_node_impl::callback_TestService, &component_implementation_,_1,_2,component_config_));

        TestPublish_ = n_.subscribe("TestPublish", 1, &sensor_tester_node_ros::topicCallback_TestPublish, this);


    }
    void topicCallback_TestPublish(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
        component_data_.in_TestPublish = *msg;
    }

    void configure_callback(squirrel_sensor_tester::sensor_tester_nodeConfig &config, uint32_t level)
    {
    }

    void configure()
    {
        component_implementation_.configure(component_config_);
    }

    void activate_all_output()
    {
    }

    void update()
    {
        activate_all_output();
        component_implementation_.update(component_data_, component_config_);
    }
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "sensor_tester_node");

    sensor_tester_node_ros node;
    node.configure();

 // if cycle time == 0 do a spin() here without calling node.update()
    ros::spin();

    return 0;
}
