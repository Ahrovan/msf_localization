
#ifndef _TIME_STAMP_H
#define _TIME_STAMP_H



#include <ctime>
#include <cmath>
#include <cstdint>



class TimeStamp
{
public:
    uint32_t sec;
    uint32_t nsec;

public:
    TimeStamp() :
        TimeStamp(0, 0)
    {
        return;
    }

    TimeStamp(uint32_t sec, uint32_t nsec) :
        sec(sec),
        nsec(nsec)
    {
        return;
    }

    TimeStamp(const TimeStamp* t) :
        TimeStamp(t->sec, t->nsec)
    {
        return;
    }

    TimeStamp(double time_in_secs)
    {
        double fractpart, intpart;
        fractpart = modf (time_in_secs , &intpart);

        this->sec=static_cast<uint32_t>(intpart);
        this->nsec=static_cast<uint32_t>(fractpart*1e9);

        return;
    }

public:
    double get_double() const
    {
        return static_cast<double>(sec)+1e-9*static_cast<double>(nsec);
    }

public:
    int toNSec() const
    {
        int nsec;

        nsec=this->sec*1e9 + this->nsec;

        return nsec;
    }

public:
    bool operator==(const TimeStamp& t2) const
    {
        if(this->sec == t2.sec && this->nsec == t2.nsec)
            return true;
        else
            return false;
    }

    bool operator>(const TimeStamp& t2) const
    {
        if(this->sec > t2.sec)
            return true;
        else if(this->sec == t2.sec && this->nsec > t2.nsec)
            return true;
        else
            return false;
    }

    bool operator>=(const TimeStamp& t2) const
    {
        if(this->operator>(t2) || this->operator==(t2))
            return true;
        else
            return false;
    }

    bool operator<(const TimeStamp& t2) const
    {
        if(this->sec < t2.sec)
            return true;
        else if(this->sec == t2.sec && this->nsec < t2.nsec)
            return true;
        else
            return false;
    }

    bool operator<=(const TimeStamp& t2) const
    {
        if(this->operator<(t2) || this->operator==(t2))
            return true;
        else
            return false;
    }

    TimeStamp operator-(const TimeStamp& t2) const
    {
        TimeStamp timeDiff;

        if(this->operator<(t2))
            return timeDiff;
        else
        {
            TimeStamp auxTimeStamp(this);
            if(this->nsec<t2.nsec)
            {
                // nsec
                auxTimeStamp.nsec+=1e9;
                // sec
                auxTimeStamp.sec-=1;
            }
            // nsec
            timeDiff.nsec=auxTimeStamp.nsec-t2.nsec;
            //sec
            timeDiff.sec=auxTimeStamp.sec-t2.sec;


            return timeDiff;
        }

    }

};




#endif
