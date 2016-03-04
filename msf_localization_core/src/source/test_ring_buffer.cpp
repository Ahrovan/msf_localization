

#include "msf_localization_core/stamped_ring_buffer.h"


#include "ros/ros.h"


int main(int argc,char **argv)
{

    //Ros Init
    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh;


    StampedRingBuffer<int> TimeStampRingBuffer;

    std::cout<<" "<<std::endl;


    //////////  addElementTop
    for(int i=0; i<10; i++)
    {
        ros::Time rosTime=ros::Time::now();
        StampedBufferObjectType<int> elementI(TimeStamp(rosTime.sec,rosTime.nsec),i);

        TimeStampRingBuffer.addElementTop(elementI);

        ros::Duration(0.01).sleep();
    }





}
