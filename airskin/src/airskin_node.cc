/**
 * Test the airskin safety algorithm, using a sense board and microblower pump.
 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <airskin/kbhit.h>
#include <airskin/Except.h>
#include <airskin/I2C_Master_Devantech_ISS.h>
#include <airskin/AirSkin_Sense.h>

static void timespec_diff(struct timespec *start, struct timespec *stop,
                          struct timespec *result)
{
  if ((stop->tv_nsec - start->tv_nsec) < 0)
  {
    result->tv_sec = stop->tv_sec - start->tv_sec - 1;
    result->tv_nsec = stop->tv_nsec - start->tv_nsec + 1000000000;
  }
  else
  {
    result->tv_sec = stop->tv_sec - start->tv_sec;
    result->tv_nsec = stop->tv_nsec - start->tv_nsec;
  }
}

class RunningMean
{
  std::vector<int> history;  // ring buffer
  int pos;
  float mean;
  int numFilled;

public:
  RunningMean(int size)
  {
    history.resize(size, 0);
    pos = 0;
    numFilled = 0;
    mean = 0.;
  }
  bool fill(int val)
  {
    if(numFilled < history.size())
    {
      history[pos] = val;
      mean += (float)history[pos]/(float)history.size();
      pos++;
      if(pos >= history.size())
        pos -= history.size();
      numFilled++;
      return false;
    }
    else
    {
      return true;
    }
  }
  void updateMean(int val)
  {
    // remove oldest
    mean -= (float)history[pos]/(float)history.size();
    // and add new
    history[pos] = val;
    mean += (float)history[pos]/(float)history.size();
    pos++;
    if(pos >= history.size())
      pos -= history.size();
  }
  float getMean()
  {
    return mean;
  }
};

class AirSkinNode
{
  static const int VALID_PRESSURE_MIN =  90000;
  static const int VALID_PRESSURE_MAX = 120000;
  static const int HISTORY_SIZE = 50;
  static const int ACTIVATION_THR = 50;

  ros::NodeHandle nh;
  std::string device_file_name;
  std::vector<unsigned char> addrs; // 8 Bit addresses
  I2C_Master *i2c_master;
  std::vector<AirSkin_Sense*> sensors;
  std::vector<RunningMean> means;
  ros::Publisher bump_pub;

public:  
  AirSkinNode()
  : nh("~")
  {
    if(!nh.getParam("device", device_file_name))
    {
      ROS_INFO("no parameter 'device' given, using default");
      device_file_name = "/dev/ttyACM0";
    }
    ROS_INFO("opening I2C device: '%s'", device_file_name.c_str());
    i2c_master = new I2C_Master_Devantech_ISS(device_file_name);
    ROS_INFO("Devantech USB-ISS adapter, rev. %d, serial number: %s\n",
        ((I2C_Master_Devantech_ISS*)i2c_master)->GetFirmwareVersion(),
        ((I2C_Master_Devantech_ISS*)i2c_master)->GetSerialNumber().c_str());
    // HACK: hardcoded addresses
    addrs.push_back(2*0x4);
    addrs.push_back(2*0x5);
    addrs.push_back(2*0x6);
    addrs.push_back(2*0x7);
    addrs.push_back(2*0x8);
    addrs.push_back(2*0x9);
    for(size_t i = 0; i < addrs.size(); i++)
    {
      sensors.push_back(new AirSkin_Sense(i2c_master, addrs[i]));
      means.push_back(RunningMean(HISTORY_SIZE));
      ROS_INFO("using AirSkin sensor with I2C address (8 Bit) %02X", addrs[i]);
    }
    for(size_t i = 0; i < addrs.size(); i++)
    {
      bool filled = false;
      int cnt = 0;
      int maxCnt = 1000;
      while(!filled && cnt < maxCnt)
      {
        int p = sensors[i]->ReadRawPressure();
        ROS_DEBUG("filling: %d", p);
        if(isValidPressure(p))
          filled = means[i].fill(p);
        // reading too fast apparently causes communication error
        usleep(5000);
        cnt++;
      }
      if(cnt == maxCnt)
        ROS_ERROR("failed to get mean for sensor %2X", addrs[i]);
    }
  }

  ~AirSkinNode()
  {
    for(size_t i = 0; i < sensors.size(); i++)
      delete sensors[i];
    delete i2c_master;
  }
  
  void init()
  {
    bump_pub = nh.advertise<std_msgs::Bool>("arm_bumper", 1);
    ROS_INFO("arm skin is ready");
  }

  void run()
  {
    int cycle_cnt = 0;
    std::vector<int> pressures(sensors.size());
    ros::Rate r(10); // 10 hz

    while(ros::ok())
    {
      bool anyActivated = false;
      std::stringstream info;
      for(size_t i = 0; i < sensors.size(); i++)
      {
        bool activated = false;
        int p = sensors[i]->ReadRawPressure();
        if(isValidPressure(p))
        {
          pressures[i] = p;
          if(p > means[i].getMean() + ACTIVATION_THR)
            activated = true;
          else
            means[i].updateMean(p);
          if(activated)
            anyActivated = true;
        }
        // else, remain unchanged
        if(cycle_cnt % 10 == 0)
          if(!activated)
            info << "  " << pressures[i] << "  ";
          else
            info << " >" << pressures[i] << "< ";
      }
      if(anyActivated)
        ROS_INFO("arm skin pressed");
      std_msgs::Bool msg;
      msg.data = anyActivated;
      bump_pub.publish(msg);
      info << "  means: ";
      for(size_t i = 0; i < sensors.size(); i++)
        info << means[i].getMean() << "  ";
      if(cycle_cnt % 10 == 0)
        ROS_DEBUG("pressures: %s", info.str().c_str());
      ros::spinOnce();
      cycle_cnt++;
      r.sleep();
    }
  }

  // NOTE: apparently we have I2C wiring issues, so somtimes -1 or some
  // crazy value is returned -> do sanity check
  bool isValidPressure(int p)
  {
    return VALID_PRESSURE_MIN < p && p < VALID_PRESSURE_MAX;
  }
};

int main (int argc, char ** argv)
{
  ros::init(argc, argv, "airskin");
  AirSkinNode an;
  an.init();
  an.run();
  return 0;
}
