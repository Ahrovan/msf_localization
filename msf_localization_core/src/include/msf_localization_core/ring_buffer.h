
#ifndef _RING_BUFFER_H
#define _RING_BUFFER_H


#include <iostream>

#include <list>

#include <iterator>     // std::advance

#include <mutex>

//#include <memory>



// TODO FIX RACE PROBLEMS
// Multithreading

template <class BufferObjectType>
class RingBuffer
{
protected:
    std::list<BufferObjectType> TheElementsList;


public:
    RingBuffer()
    {
        return;
    }

    ~RingBuffer()
    {
        // Be tidy

        // TheElementsList
        this->TheElementsList.clear();

        return;
    }

public:
    // Add element in the top of the buffer
    int addElementTop(const BufferObjectType TheElement)
    {
        // There are no available elements in the empty elements list. Create elements in the lists
        TheElementsList.push_front(TheElement);

        //
        return 0;

    }

    // Add element in the end of the buffer
    int addElementEnd(const BufferObjectType TheElement)
    {
        TheElementsList.push_back(TheElement);

        //
        return 0;
    }


    // Add element in the middle of the buffer, before element I
    int addElementBeforeI(const BufferObjectType TheElement, typename std::list<BufferObjectType>::iterator itElement)
    {
        TheElementsList.insert(itElement, TheElement);
        return 0;
    }

    // Add element in the middle of the buffer, after element I
    int addElementAfterI(const BufferObjectType TheElement, typename std::list<BufferObjectType>::iterator itElement)
    {
        if(itElement==TheElementsList.end())
        {
            // Do nothing
        }
        else
        {
            std::advance(itElement, 1);
        }

        addElementBeforeI(TheElement, itElement);

        return 0;
    }

    int addElementInI(const BufferObjectType TheElement, typename std::list<BufferObjectType>::iterator itElement)
    {
        // Previous
        //typename std::list<BufferObjectType>::iterator previousIt=itElement;

        // Change
        (*itElement)=TheElement;

        // Delete old
        //TheElementsList.erase(previousIt);

        // end
        return 0;
    }


public:


    // Get the first element
    int getFirstElement(BufferObjectType& TheElement) const
    {

        TheElement=*TheElementsList.begin();

        return 0;
    }

    int getElementI(BufferObjectType& TheElement, typename std::list<BufferObjectType>::const_iterator itElement) const
    {
        if(itElement==TheElementsList.end())
            return 1;

        TheElement=*itElement;

        return 0;
    }

    // Get begining
    typename std::list<BufferObjectType>::const_iterator getBegin() const
    {
        return TheElementsList.begin();
    }

    typename std::list<BufferObjectType>::iterator getBegin()
    {
        return TheElementsList.begin();
    }

    // Get end
    typename std::list<BufferObjectType>::iterator getEnd()
    {
        return TheElementsList.end();
    }

    typename std::list<BufferObjectType>::const_iterator getEnd() const
    {
        return TheElementsList.end();
    }



public:
    typename std::list<BufferObjectType>::size_type getSize() const
    {
        return TheElementsList.size();
    }



public:
    // Purge Last Elements in the buffer
    int purgeLast()
    {

        typename std::list<BufferObjectType>::iterator ListIterator=TheElementsList.end();
        std::advance(ListIterator, -1);

        TheElementsList.erase(ListIterator);


        //
        return 0;
    }

    // Purge Last Elements Starting from I
    int purgeLastElementsFromI(int iElement)
    {
        //std::cout<<"Cleaning buffer. Last "<<iElement<<" elements out of "<<this->getSize()<<std::endl;

        if(this->getSize()>=iElement)
        {
            typename std::list<BufferObjectType>::iterator ListIterator=TheElementsList.begin();

            std::advance(ListIterator, iElement);

            TheElementsList.erase(ListIterator, TheElementsList.end());
        }


        return 0;
    }


    int purgeLastElementsFromI(typename std::list<BufferObjectType>::iterator itElement)
    {
        //std::cout<<"Cleaning buffer. It has "<<this->getSize()<<" elements"<<std::endl;


        TheElementsList.erase(itElement, TheElementsList.end());



        return 0;
    }

    // Purge complete buffer
    int purgeFull()
    {
        TheElementsList.clear();

        return 0;
    }



};





#endif
