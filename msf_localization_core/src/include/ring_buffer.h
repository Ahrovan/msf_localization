
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

protected:
    std::list<BufferObjectType> TheEmptyElementsList;
    //std::list<BufferObjectType>::iterator TheEmptyElementsListIterator;

protected:
    //BufferObjectType* TheInitElement;
    //BufferObjectType* TheFinElement;


public:
    RingBuffer()
    {
        //TheInitElement=ptrnull;
        //TheFinElement=ptrnull;

        return;
    }

    ~RingBuffer()
    {
        // Be tidy

        // TheElementsList
//        for(typename std::list<BufferObjectType>::iterator it=this->TheElementsList.begin(); it!=this->TheElementsList.end(); ++it)
//            delete *it;
        this->TheElementsList.clear();

        // TheEmptyElementsList
//        for(typename std::list<BufferObjectType>::iterator it=this->TheEmptyElementsList.begin(); it!=this->TheEmptyElementsList.end(); ++it)
//            delete *it;
        this->TheEmptyElementsList.clear();


        return;
    }

public:
    // Add element in the top of the buffer
    int addElementTop(const BufferObjectType TheElement)
    {
        if(TheEmptyElementsList.size() == 0)
        {
            // There are no available elements in the empty elements list. Create elements in the lists
            TheElementsList.push_front(TheElement);

            //
            return 0;
        }
        else
        {
            // Copy first element of TheEmptyElementsList to the begining of TheElementsList
            TheElementsList.splice(TheElementsList.begin(), TheEmptyElementsList, TheEmptyElementsList.begin());

            // Write the value
            typename std::list<BufferObjectType>::iterator ListIterator=TheElementsList.begin();

            *ListIterator=TheElement;


            //
            return 0;
        }

        return 1;
    }

    // Add element in the middle of the buffer, after element I
    int addElementAfterI(const BufferObjectType TheElement, int iElement)
    {
        if(TheEmptyElementsList.size() == 0)
        {
            // There are no available elements in the empty elements list. Create elements in the lists
            TheEmptyElementsList.push_front(TheElement);
        }
        else
        {
            typename std::list<BufferObjectType>::iterator ListIterator=TheEmptyElementsList.begin();
            *ListIterator=TheElement;
        }

        typename std::list<BufferObjectType>::iterator ListIterator=TheElementsList.begin();
        std::advance(ListIterator, iElement+1);


        // Copy first element of TheEmptyElementsList to the begining of TheElementsList
        TheElementsList.splice(ListIterator, TheEmptyElementsList, TheEmptyElementsList.begin());



        return 0;
    }


public:

    // Get element in the middle of the buffer by position
    int getElementI(BufferObjectType& TheElement, int iElement)
    {
        typename std::list<BufferObjectType>::iterator ListIterator=TheElementsList.begin();

        std::advance(ListIterator, iElement);

        if(ListIterator==TheElementsList.end())
            return 1;

        TheElement=*ListIterator;



        return 0;
    }


public:
    typename std::list<BufferObjectType>::size_type getSize() const
    {
        return TheElementsList.size();
    }

    typename std::list<BufferObjectType>::size_type getEmptySize() const
    {
        return TheEmptyElementsList.size();
    }


public:
    // Purge Last Elements in the buffer
    int purgeLast()
    {
        // TODO fix
        //TheElementsList.pop_back();
        typename std::list<BufferObjectType>::iterator ListIterator=TheElementsList.end();
        --ListIterator;
        TheEmptyElementsList.splice(TheEmptyElementsList.end(), TheElementsList, ListIterator);


        //
        return 0;
    }

    // Purge Last Elements Starting from I
    int purgeLastElementsFromI(int iElement)
    {
        std::cout<<"Cleaning buffer. Last "<<iElement<<" elements out of "<<this->getSize()<<std::endl;

        if(this->getSize()>=iElement)
        {
        typename std::list<BufferObjectType>::iterator ListIterator=TheElementsList.begin();

        std::advance(ListIterator, iElement);


        TheEmptyElementsList.splice(TheEmptyElementsList.end(), TheElementsList, ListIterator, TheElementsList.end());
        }


        return 0;
    }


};





#endif
