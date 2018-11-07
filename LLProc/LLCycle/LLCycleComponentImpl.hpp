// ====================================================================== 
// \title  LLCycleImpl.hpp
// \author tcanham
// \brief  hpp file for LLCycle component implementation class
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

#ifndef LLCycle_HPP
#define LLCycle_HPP

#include "LLProc/LLCycle/LLCycleComponentAc.hpp"
#include "Fw/Time/Time.hpp"
#include <Os/IntervalTimer.hpp>

namespace LLProc {

    class LLCycleComponentImpl: public LLCycleComponentBase {

        public:

            // ----------------------------------------------------------------------
            // Construction, initialization, and destruction
            // ----------------------------------------------------------------------

            //! Construct object LLCycle
            //!
            LLCycleComponentImpl(
#if FW_OBJECT_NAMES == 1
                    const char * const compName /*!< The component name*/
#else
                    void
#endif
                    );

            //! Initialize object LLCycle
            //!
            void init(const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
            );

            //! Destroy object LLCycle
            //!
            ~LLCycleComponentImpl(void);

            //! Run cycles
            //!
            void runCycles(NATIVE_UINT_TYPE cycles);

        PRIVATE:

            Svc::TimerVal m_rgTimer; //!< timer for measuring software cycle time
            Svc::TimerVal m_rtiStartTimer; //!< timer for measuring start of RTI
            Svc::TimerVal m_rtiEndTimer; //!< timer for measuring end of RTI
            U32 m_maxRti; //!< Store the max RTI duration
            U32 m_minRti; //!< Store the max RTI duration

            Fw::Time m_rtiTime; //!< current LLProc clock

            U32 m_overRuns; //!< count of cycle overruns

            NATIVE_UINT_TYPE m_rtiNum;
            bool m_firstRti;

            //Os::IntervalTimer::RawTime m_rtiLog[10];

            // time sync variables
            enum TimeSyncState {
                TIME_SYNC_START,
                //TODO - implement platform-specific timesync here
                TIME_SYNC_WAIT_FOR_SYNCH_RTI,
                TIME_SYNC_DONE
            } m_timeSyncStage;

    };

} // end namespace LLProc

#endif
