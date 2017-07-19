#include "squirrel_control/base_controller.h"

//#include <chrono>
//#include <thread>

using namespace ros;
using namespace std;

namespace squirrel_control {

    void BaseController::initialize(ros::NodeHandle& node){

        private_nh.param("baseControl/proportional_theta", p_theta_, 0.8);
        //    private_nh.param("baseControl/derivative_theta", d_theta_, 0.25);
        //    private_nh.param("baseControl/integral_theta", i_theta_, 0.004);
        //    private_nh.param("baseControl/integral_theta_max", i_theta_max_, 0.8);
        //    private_nh.param("baseControl/integral_theta_min", i_theta_min_, -0.8);

        private_nh.param("baseControl/proportional_x", p_x_, 1.23);
        //    private_nh.param("baseControl/derivative_x", d_x_, 0.3);
        //    private_nh.param("baseControl/integral_x", i_x_, 0.004);
        //    private_nh.param("baseControl/integral_x_max", i_x_max_, 0.8);
        //    private_nh.param("baseControl/integral_x_min", i_x_min_, -0.8);

        private_nh.param("baseControl/proportional_y", p_y_, 1.23);
        //    private_nh.param("baseControl/derivative_y", d_y_, 0.3);
        //    private_nh.param("baseControl/integral_y", i_y_, 0.004);
        //    private_nh.param("baseControl/integral_y_max", i_y_max_, 0.8);
        //    private_nh.param("baseControl/integral_y_min", i_y_min_, -0.8);

        private_nh.param("baseControl/vel_ang_max", vel_ang_max_, 0.5);
        private_nh.param("baseControl/vel_x_max", vel_x_max_, 0.5);
        private_nh.param("baseControl/vel_y_max", vel_y_max_, 0.5);

        pid_theta_.initPid(p_theta_, 0.0,0.0,0.0,0.0);
        pid_x_.initPid(p_x_, 0.0,0.0,0.0,0.0);
        pid_y_.initPid(p_y_, 0.0,0.0,0.0,0.0);

        this->time_step_ = 1 / controller_frequency_;

        subOdometry = node.subscribe(ROBOTINO_ODOM_TOPIC, 1, &BaseController::callbackOdometry, this);
        pubMove = node.advertise<geometry_msgs::Twist>(ROBOTINO_MOVE_TOPIC, 1);

        sleep(1);
        //std::this_thread::sleep_for(std::chrono::milliseconds(100));
        gotoCommand=false;
        move_base_thread_ = new boost::thread(boost::bind(&BaseController::moveBaseThread, this));
        //ptp_base_thread_ = new boost::thread(boost::bind(&BaseController::ptpBaseThread, this));
    }

    BaseController::BaseController(ros::NodeHandle& node,double controller_freq):
        start_move_base_(false),
        start_ptp_base_(false),
        private_nh("~"),
        controller_frequency_(controller_freq)
        {
            this->initialize(node);
        }

    BaseController::~BaseController() {

        move_base_thread_->interrupt();
        move_base_thread_->join();
        ptp_base_thread_->interrupt();
        ptp_base_thread_->join();

        delete ptp_base_thread_;
        delete move_base_thread_;
    }


    void BaseController::callbackOdometry(nav_msgs::Odometry msg) {
        robot_pose_mutex_.lock();
        odometry = msg;
        robot_pose_mutex_.unlock();
    }

    void BaseController::ptp(double desired_x, double desired_y, double desired_theta){
        ptp_pose_mutex_.lock();
        desired_theta_ptp = desired_theta;
        desired_x_ptp= desired_x;
        desired_y_ptp = desired_y;
        start_ptp_base_ = true;
        ptp_pose_mutex_.unlock();

    }

    bool BaseController::targetReached(float currentVal, float targetVal , float startingVal){
        float acceptencePercentage =0.03;
        float distance = fabs(targetVal-startingVal);
        if (distance < 0.01)
            return true;
        if (currentVal < targetVal + acceptencePercentage *distance && currentVal >  targetVal - acceptencePercentage *distance){
            return true;
        }

        return false;
    }


    void BaseController::moveBase(double desired_x, double desired_y,double desired_theta) {
        start_ptp_base_=false;
        move(desired_x, desired_y,desired_theta);
        gotoCommand=false;
    }

    void BaseController::gotoBase(double desired_x, double desired_y,double desired_theta) {
        start_ptp_base_=false;
        gotoCommand=true;
        move(desired_x, desired_y,desired_theta);

    }


    void BaseController::move(double desired_x, double desired_y,double desired_theta) {

        move_pose_mutex_.lock();
        desired_theta_ = desired_theta;
        desired_x_ = desired_x;
        desired_y_ = desired_y;
        start_move_base_ = true;
        move_pose_mutex_.unlock();

    }
    void BaseController::ptpBaseThread(){
        while(1){
            if (start_ptp_base_){
                auto startingPose = getCurrentState();
                auto current_pose = startingPose;

                while (start_ptp_base_ && !(targetReached(current_pose.at(2),desired_theta_ptp,startingPose.at(2)) && targetReached(current_pose.at(0),desired_x_ptp,startingPose.at(0)) && targetReached(current_pose.at(1),desired_y_ptp,startingPose.at(1)))){
                    move( desired_x_ptp,desired_y_ptp, desired_theta_ptp);
                    current_pose = getCurrentState();
                }
                start_ptp_base_=false;
            }
        }
    }

    void BaseController::moveBaseThread(){

        ros::Rate moveBaseRate(controller_frequency_);

        while (ros::ok){

            if(start_move_base_) {

                bool velExceeded=false;
                current_base_vel_ = getNullTwist();
                auto currentPose = getCurrentState();
                double current_theta = currentPose.at(2);
                if(std::isnan(desired_theta_) == 0){
                    double orient_error = rotationDifference(desired_theta_, current_theta);
                    current_base_vel_.angular.z = pid_theta_.computeCommand(orient_error, ros::Duration(time_step_));
                    if(fabs(current_base_vel_.angular.z) > vel_ang_max_) {
                        if(gotoCommand)
                            current_base_vel_.angular.z = (current_base_vel_.angular.z > 0 ? vel_ang_max_ : - vel_ang_max_);
                        else
                            velExceeded=true;
                    }


                }


                double err_x_odom = std::isnan(desired_x_) == 0  ? desired_x_ - currentPose.at(0) : 0;
                double err_y_odom = std::isnan(desired_y_) == 0  ? desired_y_ - currentPose.at(1) : 0;


                double err_x_r = cos(current_theta) * err_x_odom + sin(current_theta) * err_y_odom;
                double err_y_r = -sin(current_theta) * err_x_odom + cos(current_theta) * err_y_odom;
                current_base_vel_.linear.x = pid_x_.computeCommand(err_x_r, ros::Duration(time_step_));
                if(fabs(current_base_vel_.linear.x) > vel_x_max_) {
                    if (gotoCommand)
                        current_base_vel_.linear.x= (current_base_vel_.linear.x > 0 ? vel_x_max_ : - vel_x_max_);
                    else
                        velExceeded=true;
                }


                current_base_vel_.linear.y = pid_y_.computeCommand(err_y_r, ros::Duration(time_step_));

                if(fabs(current_base_vel_.linear.y) > vel_y_max_) {
                    if(gotoCommand)
                        current_base_vel_.linear.y = (current_base_vel_.linear.y > 0 ? vel_y_max_ : - vel_y_max_);
                    else
                        velExceeded=true;

                }

                if (!velExceeded)
                    pubMove.publish(current_base_vel_);
                else
                    cout << " Base velocity exceeded the limit" << endl;

                start_move_base_ = false;
            }

            moveBaseRate.sleep();

        }

    }

    geometry_msgs::Twist BaseController::getNullTwist() {

        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = 0.0;

        return cmd;
    }

    double BaseController::rotationDifference(double angle, double theta_robot) {

        double err_th = angle - theta_robot;

        if(err_th > M_PI) err_th = - (2 * M_PI - err_th);
        if(err_th < -M_PI) err_th = 2 * M_PI + err_th;

        return err_th;
    }

    std::vector<double> BaseController::getCurrentState() {
        std::vector<double> states;
        robot_pose_mutex_.lock();
        geometry_msgs::Quaternion odomBkp = odometry.pose.pose.orientation;
        tf::Quaternion quat(odomBkp.x,odomBkp.y,odomBkp.z,odomBkp.w);
        quat.normalize();

        states.push_back(odometry.pose.pose.position.x);
        states.push_back(odometry.pose.pose.position.y);
        states.push_back(tf::getYaw(quat));
        robot_pose_mutex_.unlock();

        return states;

    }
}
