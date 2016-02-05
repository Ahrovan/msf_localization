

#include "stamped_ring_buffer.h"


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

    std::cout<<"Ring Buffer:"<<std::endl;
    for(unsigned int i=0; i<TimeStampRingBuffer.getSize(); i++)
    {
        StampedBufferObjectType<int> stampI;

        TimeStampRingBuffer.getElementI(stampI, i);

        std::cout<<" - i="<<i<<"; stamp="<<stampI.timeStamp.sec<<" s; "<<stampI.timeStamp.nsec<<" ns; "<<"; object="<<stampI.object<<std::endl;

    }



    ///////  addElementByStamp
    // Get two elements
    StampedBufferObjectType<int> elementI;
    if(TimeStampRingBuffer.getElementI(elementI, 3))
        std::cout<<"Error aqui 3"<<std::endl;

    StampedBufferObjectType<int> elementI1;
    if(TimeStampRingBuffer.getElementI(elementI1, 4))
        std::cout<<"Error aqui 4"<<std::endl;


    TimeStamp newStamp;
    newStamp.sec=(elementI.timeStamp.sec+elementI1.timeStamp.sec)/2;
    newStamp.nsec=(elementI.timeStamp.nsec+elementI1.timeStamp.nsec)/2;

    std::cout<<"TimeStamp to add:"<<newStamp.sec<<" s; "<<newStamp.nsec<<" ns"<<std::endl;

    StampedBufferObjectType<int> TheNewObjectInTheBuffer(newStamp, 100);

    TimeStampRingBuffer.addElementByStamp(TheNewObjectInTheBuffer);



    std::cout<<"Ring Buffer:"<<std::endl;
    for(unsigned int i=0; i<TimeStampRingBuffer.getSize(); i++)
    {
        StampedBufferObjectType<int> stampI;

        TimeStampRingBuffer.getElementI(stampI, i);

        std::cout<<" - i="<<i<<"; stamp="<<stampI.timeStamp.sec<<" s; "<<stampI.timeStamp.nsec<<" ns; "<<"; object="<<stampI.object<<std::endl;

    }




    ////////// getElementByStamp







    /////////// searchElementByStamp






    ///////// updateElementByStamp






    //////////// purgeOlderThanStamp
    if(TimeStampRingBuffer.getSize()>10)
    {
        StampedBufferObjectType<int> stampI;
        TimeStampRingBuffer.getElementI(stampI, 3);

        std::cout<<" Purge older than stamp="<<stampI.timeStamp.sec<<" s; "<<stampI.timeStamp.nsec<<" ns"<<std::endl;

        TimeStampRingBuffer.purgeOlderThanStamp(stampI.timeStamp);
    }



    std::cout<<"Ring Buffer:"<<std::endl;
    for(unsigned int i=0; i<TimeStampRingBuffer.getSize(); i++)
    {
        StampedBufferObjectType<int> stampI;

        TimeStampRingBuffer.getElementI(stampI, i);

        std::cout<<" - i="<<i<<"; stamp="<<stampI.timeStamp.sec<<" s; "<<stampI.timeStamp.nsec<<" ns; "<<"; object="<<stampI.object<<std::endl;

    }

    std::cout<<" Size of the ring buffer="<<TimeStampRingBuffer.getSize()<<std::endl;
    std::cout<<" Size of the empty elements on ring buffer="<<TimeStampRingBuffer.getEmptySize()<<std::endl;



}
