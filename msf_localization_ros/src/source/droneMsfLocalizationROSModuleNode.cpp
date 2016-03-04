//////////////////////////////////////////////////////
//  droneMsfLocalizationROSModuleNode.cpp
//
//  Created on: Feb, 2015
//      Author: joselusl
//
//  Last modification on:
//      Author: joselusl
//
//////////////////////////////////////////////////////



//I/O stream
//std::cout
#include <iostream>


//MSF Odometre ROS Module
#include "msf_localization_ros/droneMsfLocalizationROSModule.h"



int main(int argc,char **argv)
{
    MsfLocalizationROS MyMsfLocalizationROS(argc, argv);
    std::cout<<"[ROSNODE] Starting "<<ros::this_node::getName()<<std::endl;
  	
    MyMsfLocalizationROS.open();

    MyMsfLocalizationROS.run();

    return 0;
}
