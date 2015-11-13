#ifndef SENSING_DRIVERS
#define SENSING_DRIVERS

#include <string>
#include <vector>
#include <queue>
#include <FT17/FT17Interface.h>

class Driver{   //abstract class: everything in common across sensors is here
//nothing here (no private data)
protected:
    std::string m_portname; //name of the arduino port
    int m_fileDesc;         //file descriptor for termios

    //config matrix (filename?)

    virtual bool setup();   //assuming setup is equal for 2 sensors on 3


public:
    virtual std::vector<double>* readData()=0;  //this function reads the data from the sensors and returns a vector (double[][])
    virtual void flush();

};

//------------------------------


class Tactile : public Driver{

    static const double A_TACT;
    static const double B_TACT;
    static const double C_TACT;
    static const double D_TACT;

    static const double A_PROX;
    static const double B_PROX;
    static const double C_PROX;
    static const double D_PROX;

    static const int NUM_TACT; //num of tactile sensors
    static const int NUM_PROX; //num of proximity sensors
    static const int NUM_VALS; //num of readings

    static const int NUM_HISTORY_VALS; //number of readings to use for stationary check
    static const int NUM_BIAS_VALS; //number of values to accumulate for calculating the bias
    static const double STATIONARY_TACTILE_THREASHOLD; //voltage threashold, if passed data is stationary
    static const double STATIONARY_PROXIMITY_THREASHOLD; //voltage threashold, if passed data is stationary

    std::vector<double> divider; //vecotr containing voltage divider numbers read from file
    std::vector<double> history_val_tact; //sum previous voltage values
    std::vector<std::queue<double>* > history_tact; //list of previous voltage values
    std::vector<double> history_val_prox; //sum previous voltage values
    std::vector<std::queue<double>* > history_prox; //list of previous voltage values
    std::vector<std::vector<double> > mean; //first 10 values used for calculating the bias

    double bias(const int idx,const double val);
    double convertTact(const double num);
    double convertProx(const double num);
    bool isStationaryTact(const double val,const int idx);
    bool isStationaryProx(const double val,const int idx);

public:
    Tactile(const std::string& portname);
    ~Tactile();
    virtual std::vector<double>* readData();

};

//not used for now
/*class Proximity : public Driver{

public:
    virtual std::vector<double>* readData();

};*/

class Wrist : public Driver{

    ft_data ft_bc_data;
    FT17Interface* ft17;
public:
    enum WristData
    {
        ForceX,
        ForceY,
        ForceZ,
        TorqueX,
        TorqueY,
        TorqueZ,
        Timestamp,
        WristDataNum
    };

    Wrist();
    ~Wrist();

    virtual std::vector<double>* readData();

};

#endif
