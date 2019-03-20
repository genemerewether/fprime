// ======================================================================
// \title  HiresCamComponentImplStub.cpp
// \author mereweth
// \brief  cpp file for HiresCam component stub class
//
// \copyright
// Copyright 2009-2017, by the California Institute of Technology.
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

#include <SnapdragonFlight/HiresCam/HiresCamComponentImpl.hpp>
#include <SnapdragonFlight/HiresCam/HiresCamComponentImplCfg.hpp>

#include <stdio.h> // TODO(mereweth@jpl.nasa.gov) - remove the debug prints
#include <sys/time.h>

#include <stdlib.h>

//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#define DEBUG_PRINT(x,...)

namespace SnapdragonFlight {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  HiresCamComponentImpl ::
    ~HiresCamComponentImpl(void)
  {
      m_initialized = false;
  }

  // ----------------------------------------------------------------------
  // Private method implementations
  // ----------------------------------------------------------------------

  void HiresCamComponentImpl ::
    preamble(void)
  {
      m_initialized = true;
#ifdef ALWAYS_CYCLE
      //(void) activate();
#endif //ALWAYS_CYCLE
  }

  void HiresCamComponentImpl ::
    finalizer(void)
  {
      //(void) deactivate();
  }

  NATIVE_INT_TYPE HiresCamComponentImpl ::
    activate(void)
  {
      //m_captureParamsLock.lock();
      m_activated = true;
      //m_captureParamsLock.unLock();
      this->log_ACTIVITY_HI_HIRESCAM_Activated();
      return 0;
  }

  NATIVE_INT_TYPE HiresCamComponentImpl ::
    deactivate(void)
  {
      //m_captureParamsLock.lock();
      m_activated = false;
      //m_captureParamsLock.unLock();
      this->log_ACTIVITY_HI_HIRESCAM_Deactivated();
      return 0;
  }

} // end namespace SnapdragonFlight
