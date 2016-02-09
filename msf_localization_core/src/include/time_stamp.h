
#ifndef _TIME_STAMP_H
#define _TIME_STAMP_H



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




#endif
