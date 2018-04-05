// ======================================================================
// \title  RateGroupDecouplerImpl.cpp
// \author mereweth
// \brief  cpp file for RateGroupDecoupler component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged. Any commercial use must be negotiated with the Office
// of Technology Transfer at the California Institute of Technology.
//
// This software may be subject to U.S. export control laws and
// regulations.  By accepting this document, the user agrees to comply
// with all U.S. export laws and regulations.  User has the
// responsibility to obtain export licenses, or other export authority
// as may be required before exporting such information to foreign
// countries or providing access to foreign persons.
// ======================================================================


#include <Svc/RateGroupDecoupler/RateGroupDecouplerComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
//#define DEBUG_PRINT(x,...)

namespace Svc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

    RateGroupDecouplerComponentImpl ::
      RateGroupDecouplerComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName
#endif
      ) :
        RateGroupDecouplerComponentBase(
#if FW_OBJECT_NAMES == 1
                                        compName
#endif
                                        ),
        m_cycles(0),
        m_maxTime(0),
        m_cycleStarted(false),
        m_overrunThrottle(0),
        m_cycleSlips(0)
    {

    }

    void RateGroupDecouplerComponentImpl ::
      init(
          const NATIVE_INT_TYPE queueDepth,
          const NATIVE_INT_TYPE instance
      )
    {
      RateGroupDecouplerComponentBase::init(queueDepth, instance);
    }

    RateGroupDecouplerComponentImpl ::
      ~RateGroupDecouplerComponentImpl(void)
    {

    }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

    void RateGroupDecouplerComponentImpl ::
      CycleIn_handler(
          const NATIVE_INT_TYPE portNum,
          Svc::TimerVal &cycleStart
      )
    {
        TimerVal end;

        this->m_cycleStarted = false;

        this->CycleOut_out(0, cycleStart);

        // grab timer for end of cycle
        end.take();

        // get rate group execution time
        U32 cycle_time = end.diffUSec(cycleStart);

        // check to see if the time has exceeded the previous maximum
        if (cycle_time > this->m_maxTime) {
            this->m_maxTime = cycle_time;
        }

        // update cycle telemetry
        //this->tlmWrite_RgMaxTime(this->m_maxTime);

        // check for cycle slip. That will happen if new cycle message has been received
        // which will cause flag will be set again.
        if (this->m_cycleStarted) {
            this->m_cycleSlips++;
            if (this->m_overrunThrottle < 5/*RATE_GROUP_DECOUPLER_OVERRUN_THROTTLE*/) {
                //this->log_WARNING_HI_RateGroupCycleSlip(this->m_cycles);
                this->m_overrunThrottle++;
            }
            // update cycle cycle slips
            DEBUG_PRINT("RG Decoupled cycle slips %d on cycle %d\n",
                        this->m_cycleSlips, this->m_cycles);
            //this->tlmWrite_RgCycleSlips(this->m_cycleSlips);
        } else { // if cycle is okay start decrementing throttle value
            if (this->m_overrunThrottle > 0) {
                this->m_overrunThrottle--;
            }
        }

        // increment cycle
        this->m_cycles++;
        //this->tlmWrite_RgNumCycles(this->m_cycles);
    }

    void RateGroupDecouplerComponentImpl ::
      CycleIn_preMsgHook(NATIVE_INT_TYPE portNum, Svc::TimerVal& cycleStart) {
        // set flag to indicate cycle has started. Check in thread for overflow.
        this->m_cycleStarted = true;
    }

} // end namespace Svc
