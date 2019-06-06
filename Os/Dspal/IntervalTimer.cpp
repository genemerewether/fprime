#include <Os/IntervalTimer.hpp>
#include <Fw/Types/Assert.hpp>
#include <dspal_time.h>
#include <errno.h>
#include <string.h>

namespace Os {

    void IntervalTimer::getRawTime(RawTime& time) {

        timespec t;

        FW_ASSERT(clock_gettime(CLOCK_REALTIME,&t) == 0,errno);

        time.upper = t.tv_sec;
        time.lower = t.tv_nsec;
    }
        
}


