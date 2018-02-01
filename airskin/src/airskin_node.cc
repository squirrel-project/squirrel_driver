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
#include <visualization_msgs/Marker.h>
#include <airskin/kbhit.h>
#include <airskin/Except.h>
#include <airskin/I2C_Master_Devantech_ISS.h>
#include <airskin/AirSkin_Sense.h>

using namespace std;

class RunningMean
{
  vector<int> history;  // ring buffer
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
  /**
   * Initial filling of the ringbuffer.
   * @return true if buffer is now completely filled, false otherwise
   */
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

class Sensor
{
private:
  static const int VALID_PRESSURE_MIN =  90000;
  static const int VALID_PRESSURE_MAX = 300000;
  static const int HISTORY_SIZE = 50;
  static const int ACTIVATION_THR = 50;
  static const int ACTIVATION_HYST = 25;
  static const float UNFREEZE_DLEAY = 5.;

  AirSkin_Sense sensor;
  RunningMean mean;
  unsigned char addr;  // 8 Bit I2C address
  string name;         // readable name of respective pad
  int p;               // current pressure
  bool is_activated;   // activation status
  bool ref_is_frozen;  // true if reference value update is frozen
  ros::Time ref_unfreeze_timer;  // we unfreeze the reference value a little bit after a release

  void updateReference()
  {
    mean.updateMean(p);
  }

public:
  Sensor(I2C_Master *_master, unsigned char _addr, const string &_name);
  // NOTE: apparently we have I2C wiring issues, so somtimes -1 or some
  // crazy value is returned -> do sanity check
  bool isValidPressure(int p)
  {
    return VALID_PRESSURE_MIN < p && p < VALID_PRESSURE_MAX;
  }
  string getName()
  {
    return name;
  }
  unsigned char getAddr()
  {
    return addr;
  }
  bool isActivated()
  {
    return is_activated;
  }
  int getPressure()
  {
    return p;
  }
  int getReference()
  {
    return mean.getMean();
  }
  bool init();
  void update();
};

Sensor::Sensor(I2C_Master *_master, unsigned char _addr, const string &_name)
: sensor(_master, _addr),
  mean(HISTORY_SIZE)
{
  addr = _addr;
  name = _name;
  p = 0;
  // set to true initially, to be on the safe side
  is_activated = true;
  ref_is_frozen = false;
  ref_unfreeze_timer = ros::Time(0.);
  ROS_INFO("using AirSkin sensor %s with I2C address (8 Bit) %02X", name.c_str(), addr);
}

bool Sensor::init()
{
  bool filled = false;
  int cnt = 0;
  int maxCnt = 200;

  // initially fill the sliding mean
  while(!filled && cnt < maxCnt)
  {
    int p = sensor.ReadRawPressure();
    if(isValidPressure(p))
      filled = mean.fill(p);
    // reading too fast apparently causes communication error
    usleep(5000);
    cnt++;
  }
  if(filled)
  {
    is_activated = false;
    ROS_INFO("pad %s (addr %02X) ready", name.c_str(), addr);
  }
  else
  {
    is_activated = true;
    ROS_ERROR("failed to get mean for pad %s (addr %02X)", name.c_str(), addr);
  }

  return filled;
}

void Sensor::update()
{
  p = sensor.ReadRawPressure();
  if(isValidPressure(p))
  {
    if(!is_activated)
    {
      if(p >= getReference() + ACTIVATION_THR)
      {
        ROS_DEBUG("pad %s (addr %02X) pressed", name.c_str(), addr);
        is_activated = true;
        // freeze reference value calculation
        ref_is_frozen = true;
      }
    }
    else
    {
      if(p <= getReference() + ACTIVATION_THR - ACTIVATION_HYST)
      {
        ROS_DEBUG("pad %s (addr %02X) released", name.c_str(), addr);
        is_activated = false;
        // trigger unfreezing reference value calculation
        ref_unfreeze_timer = ros::Time::now();
      }
    }
    if(!ref_is_frozen)
    {
      updateReference();
    }
    else
    {
      ros::Duration d = ros::Time::now() - ref_unfreeze_timer;
      if(d.toSec() > UNFREEZE_DLEAY)
      {
        // NOTE: This check is necessary in case the pad is leaking. In that case there
        // will be under-pressurs after a longer/harder press, which will slowly grow to
        // 0 again. If we update the reference during that time, the pad will activate
        // itself. Therefore wait until the under-pressure is almost0 again.
        if(p > getReference() - ACTIVATION_HYST && p < getReference())
        {
          ref_is_frozen = false;
          ROS_DEBUG("pad %s (addr %02X) resume reference", name.c_str(), addr);
        }
      }
    }
  }
}

class AirSkinNode
{
  ros::NodeHandle nh;
  ros::Publisher bump_pub;
  ros::Publisher marker_pub;
  string device_file_name;
  I2C_Master *i2c_master;
  vector<Sensor*> sensors;
  bool is_activated;
  bool airskin_ok;

public:  
  AirSkinNode();
  ~AirSkinNode();
  void init();
  void run();
  /**
   * Update rviz visualisation marker.
   * To be called when activation status changes. If activated, then it will display
   * some text above the robotthe robot.
   */
  void updateDisplayActivated();
};

AirSkinNode::AirSkinNode()
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

  // fill in fixed I2C addresses of pads
  // Note: It might be nice to get these from ROS parameters, but actually they never
  // change. So, hardcoding is fine. Really. Trust me.
  sensors.push_back(new Sensor(i2c_master, 2*0x4, "5-forearm-end"));      // 0x08 in 8 bit
  sensors.push_back(new Sensor(i2c_master, 2*0x5, "6-hand"));             // 0x0A
  sensors.push_back(new Sensor(i2c_master, 2*0x6, "4-forearm-top"));      // 0x0C
  sensors.push_back(new Sensor(i2c_master, 2*0x7, "2-upperarm-top"));     // 0x0E
  sensors.push_back(new Sensor(i2c_master, 2*0x8, "3-forearm-bottom"));   // 0x10
  sensors.push_back(new Sensor(i2c_master, 2*0x9, "1-upperarm-bottom"));  // 0x12
 
  size_t ok_cnt = 0;
  for(size_t i = 0; i < sensors.size(); i++)
  {
    if(sensors[i]->init())
      ok_cnt++;
  }
  airskin_ok = (ok_cnt == sensors.size());
  is_activated = false;
}

AirSkinNode::~AirSkinNode()
{
  for(size_t i = 0; i < sensors.size(); i++)
    delete sensors[i];
  delete i2c_master;
}

void AirSkinNode::init()
{
  bump_pub = nh.advertise<std_msgs::Bool>("arm_bumper", 1);
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualisation", 1);
  ROS_INFO("arm skin is ready");
}

void AirSkinNode::run()
{
  ros::Rate r(10);

  while(ros::ok() && airskin_ok)
  {
    bool anyActivated = false;
    for(size_t i = 0; i < sensors.size(); i++)
    {
      sensors[i]->update();
      if(sensors[i]->isActivated())
        anyActivated = true;
    }

    if(anyActivated)
    {
      if(!is_activated)
      {
        is_activated = true;
        ROS_INFO("arm skin pressed");
        updateDisplayActivated();
      }
    }
    else
    {
      if(is_activated)
      {
        is_activated = false;
        ROS_INFO("arm skin released");
        updateDisplayActivated();
      }
    }

    std_msgs::Bool msg;
    msg.data = anyActivated;
    bump_pub.publish(msg);

    ros::spinOnce();
    r.sleep();
  }

  if(!airskin_ok)
    ROS_ERROR("Airskin is damaged");
}

void AirSkinNode::updateDisplayActivated()
{
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "/base_link";
  marker.ns = "airskin";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;    
  marker.lifetime = ros::Duration();

  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 1.5;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
 
  marker.text = "ARM COLLISION";
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;


  if(is_activated)
    marker.action = visualization_msgs::Marker::ADD;
  else
    marker.action = visualization_msgs::Marker::DELETE;

  marker_pub.publish(marker);
}

int main (int argc, char ** argv)
{
  ros::init(argc, argv, "airskin");
  AirSkinNode an;
  an.init();
  an.run();
  return 0;
}
