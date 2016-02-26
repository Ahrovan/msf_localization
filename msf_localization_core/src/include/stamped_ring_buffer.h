
#ifndef _STAMPED_RING_BUFFER_H
#define _STAMPED_RING_BUFFER_H


#include "ring_buffer.h"

#include "time_stamp.h"





template <class BufferObjectType>
class StampedBufferObjectType
{
public:
    TimeStamp timeStamp;

public:
    BufferObjectType object;

public:
    StampedBufferObjectType()
    {
        return;
    }
    StampedBufferObjectType(TimeStamp timeStamp, BufferObjectType object) :
        timeStamp(timeStamp),
        object(object)
    {
        return;
    }


};




template <class BufferObjectType>
class StampedRingBuffer : public RingBuffer< StampedBufferObjectType<BufferObjectType> >
{
public:
    int searchElementByStamp(StampedBufferObjectType<BufferObjectType>& TheElement, TimeStamp timeStamp)
    {
        typename std::list< StampedBufferObjectType<BufferObjectType> >::iterator ListIterator=this->TheElementsList.begin();

        for(ListIterator; ListIterator!=this->TheElementsList.end(); ++ListIterator)
        {
            if(*ListIterator->timeStamp==timeStamp)
            {
                TheElement=*ListIterator;
                return 0;
            }
        }
        // Not found
        return 1;
    }

    int searchIElementByStamp(typename std::list< StampedBufferObjectType<BufferObjectType> >::iterator& itElement, TimeStamp timeStamp)
    {
        itElement=this->TheElementsList.begin();

        for(itElement; itElement!=this->TheElementsList.end(); ++itElement)
        {
            if((itElement)->timeStamp==timeStamp)
            {
                return 0;
            }
        }
        // Not found
        return 1;
    }


    int searchPreIElementByStamp(typename std::list< StampedBufferObjectType<BufferObjectType> >::iterator& itElement, TimeStamp timeStamp)
    {

        // The List Doesn't have any elements
        if(this->TheElementsList.size()==0)
        {
            //std::cout<<"The List Has no elements"<<std::endl;
            itElement=this->TheElementsList.begin();
            return -1;
        }


        // The First Element is older than the searched time stamp. The element goes at the beginig
        if(timeStamp > this->TheElementsList.begin()->timeStamp)
        {
            //std::cout<<"The new Element goes at the begining"<<std::endl;
            itElement=this->TheElementsList.begin();
            return 10;
        }

        // The Last Element is newer than the searched time stamp. The element goes at the end
        if(timeStamp < this->TheElementsList.back().timeStamp)
        {
            //std::cout<<"The new Element goes at the end"<<std::endl;
            itElement=this->TheElementsList.end();
            return 20;
        }


        //typename std::list< StampedBufferObjectType<BufferObjectType> >::iterator ListIterator=this->TheElementsList.begin();

        typename std::list< StampedBufferObjectType<BufferObjectType> >::iterator AuxListIterator;

        typename std::list< StampedBufferObjectType<BufferObjectType> >::iterator ListIteratorEndSearch=this->TheElementsList.end();
        --ListIteratorEndSearch;


        for(itElement=this->TheElementsList.begin(); itElement!=ListIteratorEndSearch; ++itElement)
        {
            AuxListIterator=itElement;
            ++AuxListIterator;

            // The Current Element has the timestamp
            if(itElement->timeStamp==timeStamp)
            {
                //std::cout<<"The Current Element has the timestamp"<<std::endl;
                return 1;
            }

            // The element iElement is the previous element
            if(itElement->timeStamp>timeStamp && AuxListIterator->timeStamp<timeStamp)
            {
                //std::cout<<"The element iElement is the previous element"<<std::endl;
                return 0;
            }

        }

        // Not found -> Should never happend
        //itElement=nullptr;
        //std::cout<<"Error Finding Element by Stamp"<<std::endl;

        return -2;
    }

    // TODO FIX!!!
    int searchPostIElementByStamp(typename std::list< StampedBufferObjectType<BufferObjectType> >::iterator& itElement, TimeStamp timeStamp)
    {
        int result=searchPreIElementByStamp(itElement, timeStamp);
        if(result==0)
        {
            std::advance(itElement, 1);
            return result;
        }
        else if(result==1)
        {
            std::advance(itElement, 1);
            return result;
        }
        else if(result==10)
        {
            //iElement=0;
            // TODO Check!
            return result;
        }
        else if(result==20)
        {
            //iElement=this->getSize();
            return result;
        }
        else
        {
            return result;
        }
    }

public:

    int addElementByStamp(const StampedBufferObjectType<BufferObjectType> TheElement)
    {
        typename std::list< StampedBufferObjectType<BufferObjectType> >::iterator itElement;
        int searchResult=this->searchPreIElementByStamp(itElement, TheElement.timeStamp);
        if(searchResult==0)
        {
            // No problem
            //std::cout<<"Element is going to be added after i="<<iElement<<std::endl;
            //std::cout<<"Element is going to be added in the middle"<<std::endl;
            if(this->addElementAfterI(TheElement,itElement))
                return 1;
            return 0;

        }
        else if(searchResult==1)
        {
            // Already added element. Need to be updated!
            //std::cout<<"Element is going to be added after i="<<iElement<<std::endl;
            //std::cout<<"Element is going to be added in the middle. Overwritting!"<<std::endl;

            if(this->addElementInI(TheElement,itElement))
                return 1;

            //std::cout<<"Element is going to be added in the middle. Overwritting done!"<<std::endl;

            return 0;
        }
        else if(searchResult==10 || searchResult==-1)
        {
            // we put in the begining
            //std::cout<<"Element is going to be added in the top of the buffer"<<std::endl;
            if(this->addElementTop(TheElement))
                return 2;
            return 0;

        }
        else if(searchResult==20)
        {
            // We put in the end
            //std::cout<<"Element is going to be added in the end of the buffer"<<std::endl;
            if(this->addElementEnd(TheElement))
                return 2;
            return 0;
        }



        return -1;
    }





public:
    int getElementByStamp(TimeStamp timeStamp, BufferObjectType& TheElement)
    {
        typename std::list< StampedBufferObjectType<BufferObjectType> >::iterator itElement;
        if(this->searchIElementByStamp(itElement, timeStamp))
            return 1;

        StampedBufferObjectType<BufferObjectType> TheBufferElement;
        if(this->getElementI(TheBufferElement, itElement))
            return 2;

        TheElement=TheBufferElement.object;

        return 0;
    }



public:
    int purgeOlderThanStamp(TimeStamp timeStamp)
    {
        typename std::list< StampedBufferObjectType<BufferObjectType> >::iterator itElement;
        int result=this->searchPostIElementByStamp(itElement,timeStamp);

        if(result==0)
        {
            this->purgeLastElementsFromI(itElement);
            return 0;
        }
        else if(result==1)
        {
            this->purgeLastElementsFromI(itElement);
            return 0;
        }
        else
        {
            // Do nothing
            return 1;
        }

        return -1;
    }


};






#endif
