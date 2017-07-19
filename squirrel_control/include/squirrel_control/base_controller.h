#ifndef BASECONTROLLER
#define BASECONTROLLER

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread.hpp>
#include <control_toolbox/pid.h>
#include <tf/tf.h>
#include <vector>
#define ROBOTINO_MOVE_TOPIC "/cmd_rotatory"
#define ROBOTINO_ODOM_TOPIC "/odom"
#define DONTCARE nan("1")

namespace squirrel_control {

    class BaseController {

    private:

        ros::NodeHandle  private_nh;

        ros::Subscriber subOdometry;
        ros::Publisher pubMove;

        nav_msgs::Odometry odometry;

        boost::thread* move_base_thread_;
        boost::thread* ptp_base_thread_;
        bool start_move_base_;
        bool start_ptp_base_;
        bool gotoCommand;
        double controller_frequency_, time_step_;
        double vel_ang_max_, vel_x_max_, vel_y_max_;
        double p_theta_;//, d_theta_, i_theta_, i_theta_min_, i_theta_max_;
        double p_x_;//, d_x_, i_x_, i_x_min_, i_x_max_;
        double p_y_;//, d_y_, i_y_, i_y_min_, i_y_max_;

        double desired_theta_, desired_x_, desired_y_;
        double desired_theta_ptp, desired_x_ptp, desired_y_ptp;
        control_toolbox::Pid pid_theta_;
        control_toolbox::Pid pid_x_;
        control_toolbox::Pid pid_y_;
        geometry_msgs::Twist current_base_vel_;

        boost::mutex robot_pose_mutex_;
        boost::mutex move_pose_mutex_;
        boost::mutex ptp_pose_mutex_;
        void ptpBaseThread();
        void callbackOdometry(nav_msgs::Odometry msg);
        void moveBaseThread();

        geometry_msgs::Twist getNullTwist();
        double rotationDifference(double angle, double theta_robot);
        void move(double desired_x, double desired_y,double desired_theta);
        void initialize(ros::NodeHandle& node);
        bool targetReached(float currentVal, float targetVal, float startingVal);

    public:

        BaseController(ros::NodeHandle& node, double controller_freq);
        ~BaseController();

        void ptp(double desired_x, double desired_y,double desired_theta);
        void moveBase(double desired_x, double desired_y,double desired_theta);
        void gotoBase(double desired_x, double desired_y,double desired_theta);
        std::vector<double> getCurrentState();//

    };
}

#endif
