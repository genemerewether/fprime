// ======================================================================
// \title  RateGroupDecouplerImpl.hpp
// \author mereweth
// \brief  hpp file for RateGroupDecoupler component implementation class
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

#ifndef RateGroupDecoupler_HPP
#define RateGroupDecoupler_HPP

#include "Svc/RateGroupDecoupler/RateGroupDecouplerComponentAc.hpp"

namespace Svc {

  class RateGroupDecouplerComponentImpl :
    public RateGroupDecouplerComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object RateGroupDecoupler
      //!
      RateGroupDecouplerComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
#endif
          U32 droppedCyclesError /*!< Number of backup cycles before error */
      );

      //! Initialize object RateGroupDecoupler
      //!
      void init(
          const NATIVE_INT_TYPE queueDepth, /*!< The queue depth*/
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object RateGroupDecoupler
      //!
      ~RateGroupDecouplerComponentImpl(void);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for CycleIn
      //!
      void BackupCycleIn_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Svc::TimerVal &cycleStart /*!< Cycle start timer value*/
      );

      //! Handler implementation for CycleIn
      //!
      void CycleIn_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Svc::TimerVal &cycleStart /*!< Cycle start timer value*/
      );

      //!  \brief Input cycle port pre message hook
      //!
      //!  The input cycle port pre message hook is called on the thread of the calling
      //!  cycle port. It sets flag to indicate that the cycle has started.
      //!
      //!  \param portNum incoming port call. For this class, should always be zero
      //!  \param cycleStart value stored by the cycle driver, used to compute execution time.

      void CycleIn_preMsgHook(NATIVE_INT_TYPE portNum, Svc::TimerVal& cycleStart); //!< CycleIn pre-message hook
      void BackupCycleIn_preMsgHook(NATIVE_INT_TYPE portNum, Svc::TimerVal& cycleStart); //!< BackupCycleIn pre-message hook

      U32 m_cycles; //!< cycles executed
      U32 m_maxTime; //!< maximum execution time in microseconds
      volatile bool m_cycleStarted; //!< indicate that cycle has started. Used to detect overruns.
      NATIVE_INT_TYPE m_overrunThrottle; //!< throttle value for overrun events
      U32 m_cycleSlips; //!< tracks number of cycle slips
      U32 m_backupCycles; //!< tracks number of missing regular cycles
      U32 m_droppedCyclesError; //!< Number of missing regular cycles before error
  };

} // end namespace Svc

#endif
