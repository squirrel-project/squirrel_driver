#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <cmath>
#include <ros/package.h>
#include <ros/ros.h>


#include "../include/squirrel_sensing_node/sensing_drivers.h"

using namespace std;

//makes the connection with m_portname
bool Driver::setup(){    //assuming setup is equal for 2 sensros on 3

    m_fileDesc=open(m_portname.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    fcntl(m_fileDesc, F_SETFL,0);//reset file status flags

    /* Set up the control structure */
     struct termios toptions;
     /* Get currently set options for the tty */
     tcgetattr(m_fileDesc, &toptions);
     /* 9600 baud */
      cfsetispeed(&toptions, B9600);
      cfsetospeed(&toptions, B9600);
      /* 8 bits, no parity, no stop bits */
       toptions.c_cflag &= ~PARENB;
       toptions.c_cflag &= ~CSTOPB;
       toptions.c_cflag &= ~CSIZE;
       toptions.c_cflag |= CS8; //should be one stop bit
       /* Canonical mode */
        toptions.c_lflag |= ICANON;

        //---------------------------------------------------------------
        /*non canonical mode */
       // /* no hardware flow control */
       //toptions.c_cflag &= ~CRTSCTS;
       // /* enable receiver, ignore status lines */
       //toptions.c_cflag |= CREAD | CLOCAL;
       // /* disable input/output flow control, disable restart chars */
       //toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
       // /* disable canonical input, disable echo,
       //disable visually erase chars,
       //disable terminal-generated signals */
       //toptions.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
       // /* disable output processing */
       //toptions.c_oflag &= ~OPOST;
        //---------------------------------------------------------------

    /* commit the options */
     tcsetattr(m_fileDesc, TCSANOW, &toptions);
     /* Flush anything already in the serial buffer */
     tcflush(m_fileDesc, TCIFLUSH);

     return true;
}

void Driver::flush(){
    /* Flush anything already in the serial buffer */
    tcflush(m_fileDesc, TCIFLUSH);
}

//------------------------------TACTILE and PROXIMITY

const int Tactile::NUM_VALS=15;
const int Tactile::NUM_TACT=12; //number of tactile values
const int Tactile::NUM_PROX=3;

const int Tactile::NUM_HISTORY_VALS=10;
const int Tactile::NUM_BIAS_VALS=100;
const double Tactile::STATIONARY_TACTILE_THREASHOLD=0.02;    //volts
const double Tactile::STATIONARY_PROXIMITY_THREASHOLD=0.2;    //volts

//calibration coefficients tactile sensor
const double Tactile::A_TACT=0.617;
const double Tactile::B_TACT=-3.386;
const double Tactile::C_TACT=6.185;
const double Tactile::D_TACT=0.7531;

//calibration coefficients proximity sensor
const double Tactile::A_PROX=20.74;
const double Tactile::B_PROX=-0.1808; //exponent
const double Tactile::C_PROX=-17.28;



Tactile::Tactile(const std::string& portname){

    m_portname=portname;

    //initialising history
    for(int i=0;i<NUM_TACT;i++)
    {
        history_val_tact.push_back(0);
        history_tact.push_back(new queue<double>());
        mean.push_back(vector<double>() );
    }
    for(int i=0;i<NUM_PROX;i++)
    {
        history_val_prox.push_back(0);
        history_prox.push_back(new queue<double>());
        mean.push_back(vector<double>() );
    }


    //read from conf.ini the divider values
    string filepath=ros::package::getPath("squirrel_sensing_node");
    filepath+="/tactile_calibration.ini";
    ifstream config(filepath.c_str());
    if(config.good()){
        string line;
        int sensId=0;
        while(getline(config,line) && sensId<=NUM_TACT){
            if(line.at(0)!='#'){ //if line is not a comment
                istringstream iss(line);
                double val;
                iss >> val;
                divider.push_back(val);
                sensId++;
            }
        }
    }else{
        ROS_INFO("ERROR: Could not locate file config.ini, assumed no dividers (==1)");
        for(int i=0;i<NUM_TACT;++i){
            divider.push_back(1);
        }
    }

    setup();
}

Tactile::~Tactile(){

    close(m_fileDesc);

    for(int i=0;i<NUM_TACT;i++)
    {
        delete history_tact[i];
    }

    for(int i=0;i<NUM_PROX;i++)
    {

        delete history_prox[i];
    }
}

//returns true if data is stationary, input is the value and the number of sensor used
bool Tactile::isStationaryTact(const double val,const int idx){

    history_tact.at(idx)->push(val);
    history_val_tact.at(idx)+=val;
    if(history_tact.at(idx)->size()<NUM_HISTORY_VALS){ //if we have not enough values yet
        return false;
    }

    history_val_tact.at(idx)-=history_tact.at(idx)->front(); //subtract oldest value
    history_tact.at(idx)->pop();
    return ( (history_val_tact.at(idx)/NUM_HISTORY_VALS) < STATIONARY_TACTILE_THREASHOLD);

}

//returns true if data is stationary, input is the value and the sensor id
bool Tactile::isStationaryProx(const double val,const int idx){

    history_prox.at(idx)->push(val);
    history_val_prox.at(idx)+=val;
    if(history_prox.at(idx)->size()<NUM_HISTORY_VALS){ //if we have not enough values yet
        return false;
    }

    history_val_prox.at(idx)-=history_prox.at(idx)->front(); //subtract oldest value
    history_prox.at(idx)->pop();
    return ( (history_val_prox.at(idx)/NUM_HISTORY_VALS) < STATIONARY_PROXIMITY_THREASHOLD);

}

//calculates mean of first 10 values and return the bias value
double Tactile::bias(const int idx,const double val)
{
    if(mean[idx].size()==0){
        mean[idx].push_back(val);
        return 1.0;
    }
    //else calculate mean of past values
    if(mean[idx].size()<NUM_BIAS_VALS){
        mean[idx].push_back(val);
    }

    double accumulator=0.0;

    for(int i=0;i<mean[idx].size();++i){
        accumulator+=mean[idx].at(i);
    }

    return (accumulator/mean[idx].size());
}


std::vector<double>* Tactile::readData(){

    std::vector<double>* res=new vector<double>(NUM_VALS,-1.0); //if a -1 is given it means there was a problem with reading that value

    char buff[255];
    int rd=read(m_fileDesc,buff,255);

    bool done=false;
    int readingNum=0;
    string st_buf;

    for(int i=0;i<255 && !done;i++){

        if(buff[i]==' '){

            double val=atof(st_buf.c_str());

            if(readingNum==NUM_VALS){
                done = true;                
            }else{

                res->at(readingNum)=val-bias(readingNum,val);  //biasing to zero
            }//if read enough values

            st_buf="";
            readingNum++;


        }//if separator found
        else if(buff[i]=='\n'){
            done = true;            
        }//else if end of line found

        st_buf+=buff[i];

    }//for stuff in buff


    for(int i=0;i<NUM_TACT;i++){//biasing to calibration
        res->at(i)= (res->at(i)*(divider[i]-1))/3.764; //calibartion curve maximum is (4.764-1)
    }

    for(int i=0;i<NUM_TACT;i++){//first 12 values are tactile

        if(!isStationaryTact(res->at(i),i)){      //if values changed
            res->at(i)=convertTact(res->at(i));
        }else{
            res->at(i)=0;
        }//if not stationary
    }

    for(int i=NUM_TACT;i<NUM_VALS;i++){//biasing to calibration distance
        res->at(i)= (res->at(i)); //calibartion curve maximum is not accounted
    }

    for(int i=NUM_TACT;i<NUM_VALS;i++){//last 3 values are proximity

        if(!isStationaryProx(res->at(i),i-NUM_TACT)){      //if values changed
            res->at(i)=convertProx(res->at(i));
        }else{
            res->at(i)=-1;
        }


    }

    return res;
}

//convert volts into newtons
double Tactile::convertTact(const double num){

    if(num==-1) {   //if value not valid
        return -1;
    }

    double val=num; //-1;   //subtract 1 volt

    return (A_TACT*pow(val,3))+(B_TACT*pow(val,2))+(C_TACT*val)+D_TACT;
}

//convert volts into distance
double Tactile::convertProx(const double num){

    if(num==-1) {   //if value not valid
        return -1;
    }

    double val=num;//-1;   //subtract 1 volt

    return (A_PROX*pow(val,B_PROX))+C_PROX;


}

//----------------------WRIST-----------------------

Wrist::Wrist()
{
    ft17=new FT17Interface ( "eth0" );
    ft17->init();
    ft17->configure ( 1, 127 );
    ft17->start_broadcast();
}

Wrist::~Wrist()
{
    ft17->stop_broadcast();
    delete ft17;
}

std::vector<double>* Wrist::readData(){
    // get the FT17 data
    ft17->get_broadcast_data ( ft_bc_data );

    vector<double>* res=new vector<double>(WristDataNum,0);

    // fill the FT_filt msg
    //frame_id = std::to_string ( ft_bc_data.board_id ) ; //frame ID, if needed

    res->at(Wrist::ForceX) = ft_bc_data.FT_filt[Wrist::ForceX];
    res->at(Wrist::ForceY) = ft_bc_data.FT_filt[Wrist::ForceY];
    res->at(Wrist::ForceZ) = ft_bc_data.FT_filt[Wrist::ForceZ];
    res->at(Wrist::TorqueX) = ft_bc_data.FT_filt[Wrist::TorqueX];
    res->at(Wrist::TorqueY) = ft_bc_data.FT_filt[Wrist::TorqueY];
    res->at(Wrist::TorqueZ) = ft_bc_data.FT_filt[Wrist::TorqueZ];
    res->at(Wrist::Timestamp) =  ft_bc_data.tStamp ;

    return res;
}

