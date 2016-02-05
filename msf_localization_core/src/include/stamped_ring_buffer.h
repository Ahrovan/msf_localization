
#ifndef _STAMPED_RING_BUFFER_H
#define _STAMPED_RING_BUFFER_H


#include "ring_buffer.h"

#include <ctime>
#include <cstdint>


class TimeStamp
{
public:
    uint32_t sec;
    uint32_t nsec;

public:
    TimeStamp()
    {
        return;
    }

    TimeStamp(uint32_t sec, uint32_t nsec) :
        sec(sec),
        nsec(nsec)
    {
        return;
    }

public:
    bool operator==(const TimeStamp t2)
    {
        if(this->sec == t2.sec && this->nsec == t2.nsec)
            return true;
        else
            return false;
    }

    bool operator>(const TimeStamp t2)
    {
        if(this->sec > t2.sec)
            return true;
        else if(this->sec == t2.sec && this->nsec > t2.nsec)
            return true;
        else
            return false;
    }

    bool operator<(const TimeStamp t2)
    {
        if(this->sec < t2.sec)
            return true;
        else if(this->sec == t2.sec && this->nsec < t2.nsec)
            return true;
        else
            return false;
    }

};


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

    int searchIElementByStamp(unsigned int& iElement, TimeStamp timeStamp)
    {
        typename std::list< StampedBufferObjectType<BufferObjectType> >::iterator ListIterator=this->TheElementsList.begin();
        unsigned int iElementAux=0;

        for(ListIterator; ListIterator!=this->TheElementsList.end(); ++ListIterator)
        {
            if(*ListIterator->timeStamp==timeStamp)
            {
                iElement=iElementAux;
                return 0;
            }
            else
                iElementAux++;
        }
        // Not found
        return 1;
    }

    int searchPreIElementByStamp(unsigned int& iElement, TimeStamp timeStamp)
    {
        typename std::list< StampedBufferObjectType<BufferObjectType> >::iterator ListIterator=this->TheElementsList.begin();
        typename std::list< StampedBufferObjectType<BufferObjectType> >::iterator AuxListIterator;

        typename std::list< StampedBufferObjectType<BufferObjectType> >::iterator ListIteratorEndSearch=this->TheElementsList.end();
        --ListIteratorEndSearch;
        iElement=0;

        for(ListIterator; ListIterator!=ListIteratorEndSearch; ++ListIterator)
        {
            AuxListIterator=ListIterator;
            ++AuxListIterator;
            if(ListIterator->timeStamp==timeStamp)
            {
                return 1;
            }
            if(ListIterator->timeStamp>timeStamp && AuxListIterator->timeStamp<timeStamp)
            {
                return 0;
            }
            else
                iElement++;
        }
        // Not found

        return 2;
    }

    int searchPostIElementByStamp(unsigned int& iElement, TimeStamp timeStamp)
    {
        int result=searchPreIElementByStamp(iElement, timeStamp);
        if(result==0)
        {
            iElement++;
            return result;
        }
        else if(result==1)
        {
            iElement++;
            return result;
        }
        else
            return result;
    }

public:
    int addElementAfterStamp(const StampedBufferObjectType<BufferObjectType> TheElement, TimeStamp timeStamp)
    {
        unsigned int iElement=0;
        if(this->searchIElementByStamp(iElement, timeStamp))
        {
            // Element not found, we put in the begining
            iElement=0;
        }

        if(this->addElementAfterI(TheElement,iElement))
            return 1;

        return 0;
    }

    int addElementByStamp(const StampedBufferObjectType<BufferObjectType> TheElement)
    {
        unsigned int iElement=0;
        int searchResult=this->searchPreIElementByStamp(iElement, TheElement.timeStamp);
        if(searchResult==0)
        {
            // No problem

        }
        else if(searchResult==1)
        {
            // Already added element. Need to be updated!
            // TODO
        }
        else
        {
            // Element not found, we put in the begining
            iElement=0;
        }

        std::cout<<"Element is going to be added after i="<<iElement<<std::endl;

        if(this->addElementAfterI(TheElement,iElement))
            return 1;


        return 0;
    }


public:
    int getElementByStamp(TimeStamp timeStamp)
    {


        return 0;
    }



public:
    int purgeOlderThanStamp(TimeStamp timeStamp)
    {
        unsigned int iElement=0;
        int result=this->searchPostIElementByStamp(iElement,timeStamp);

        if(result==0)
        {
            this->purgeLastElementsFromI(iElement);
            return 0;
        }
        else if(result==1)
        {
            this->purgeLastElementsFromI(iElement);
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
