#include <Os/IntervalTimer.hpp>
#include <Fw/Types/Assert.hpp>
#include <R5/TiHal/include/HL_rti.h>
#include <string.h>

namespace Os {

    IntervalTimer::IntervalTimer() {
        memset(&this->m_startTime,0,sizeof(this->m_startTime));
        memset(&this->m_stopTime,0,sizeof(this->m_stopTime));
    }

    IntervalTimer::~IntervalTimer() {
    }

    void IntervalTimer::getRawTime(RawTime& time) {
        time.upper = rtiGetMedResTimestamp();
        time.lower = 0;
    }

    U32 IntervalTimer::getDiffUsec(void) {
        return getDiffUsec(this->m_stopTime,this->m_startTime);
    }

    U32 IntervalTimer::getDiffUsec(const RawTime& t1In, const RawTime& t2In) {
        return t1In.upper - t2In.upper;
    }

    void IntervalTimer::start() {
        getRawTime(this->m_startTime);
    }

    void IntervalTimer::stop() {
        getRawTime(this->m_stopTime);
    }

}


