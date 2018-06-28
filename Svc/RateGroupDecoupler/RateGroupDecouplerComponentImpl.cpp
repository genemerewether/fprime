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

//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#define DEBUG_PRINT(x,...)

namespace Svc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

    RateGroupDecouplerComponentImpl ::
      RateGroupDecouplerComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName,
#endif
          U32 droppedCyclesError
      ) :
        RateGroupDecouplerComponentBase(
#if FW_OBJECT_NAMES == 1
                                        compName
#endif
                                        ),
        m_cycles(0u),
        m_maxTime(0u),
        m_cycleStarted(false),
        m_overrunThrottle(0),
        m_cycleSlips(0u),
        m_backupCycles(0u),
        m_droppedCyclesError(droppedCyclesError)
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
      BackupCycleIn_handler(
          const NATIVE_INT_TYPE portNum,
          Svc::TimerVal &cycleStart
      )
    {
        this->m_backupCycles++;
        /* If we hit specified number of backup cycles, set output error port
         * and begin cycling ourselves
         */

        if (this->m_backupCycles > m_droppedCyclesError) {
            this->CycleIn_handler(portNum, cycleStart);
        }
    }

    void RateGroupDecouplerComponentImpl ::
      BackupCycleIn_preMsgHook(NATIVE_INT_TYPE portNum, Svc::TimerVal& cycleStart) {
        // set flag to indicate cycle has started. Check in thread for overflow.

        // we are using backup cycler, so allow for checking for cycle slip
        // TODO(mereweth) - might see a fake slip when backup cycler starts/stops
        if (this->m_backupCycles > m_droppedCyclesError) {
            DEBUG_PRINT("Backup cycle %u gt than error threshold %u; cycle %u\n",
                        this->m_backupCycles, m_droppedCyclesError, this->m_cycles);
            this->m_cycleStarted = true;
        }
    }

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

        if (this->m_backupCycles > m_droppedCyclesError) {
            DEBUG_PRINT("Got a real cycle after %u backup; switching\n",
                        this->m_backupCycles);
        }

        // reset the number of backup cycles because we got a real cycle
        this->m_backupCycles = 0;
    }

} // end namespace Svc
