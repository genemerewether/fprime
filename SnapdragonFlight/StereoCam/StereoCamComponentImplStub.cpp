// ======================================================================
// \title  StereoCamComponentImplStub.cpp
// \author mereweth
// \brief  cpp file for StereoCam component stub class
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

#include <SnapdragonFlight/StereoCam/StereoCamComponentImpl.hpp>
#include <SnapdragonFlight/StereoCam/StereoCamComponentImplCfg.hpp>

#include <stdio.h> // TODO(mereweth@jpl.nasa.gov) - remove the debug prints
#include <sys/time.h>

#include <stdlib.h>

#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
//#define DEBUG_PRINT(x,...)

namespace SnapdragonFlight {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  StereoCamComponentImpl ::
    ~StereoCamComponentImpl(void)
  {
      m_initialized = false;
  }

  // ----------------------------------------------------------------------
  // Private method implementations
  // ----------------------------------------------------------------------

    void StereoCamComponentImpl ::
      preamble(void)
    {
        m_initialized = true;
#ifdef ALWAYS_CYCLE
        //(void) activate();
#endif //ALWAYS_CYCLE
    }

    void StereoCamComponentImpl ::
      finalizer(void)
    {
        //(void) deactivate();
    }

    NATIVE_INT_TYPE StereoCamComponentImpl ::
      activate(void)
    {
        //m_captureParamsLock.lock();
        m_activated = true;
        //m_captureParamsLock.unLock();
        this->log_ACTIVITY_HI_SCAM_Activated();

        // TODO(mereweth) - start timer at 30 Hz to send fake images?

        /*BYTE* ptr = (BYTE*)buff.getdata();
        ptr[SCAM_IMAGE_WIDTH*10 + 320] = 255;
        ptr[SCAM_IMAGE_WIDTH*20 + 320] = 255;
        ptr[SCAM_IMAGE_WIDTH*50 + 320] = 255;
        */

        return 0;
    }

    NATIVE_INT_TYPE StereoCamComponentImpl ::
      deactivate(void)
    {
        //m_captureParamsLock.lock();
        m_activated = false;
        //m_captureParamsLock.unLock();
        this->log_ACTIVITY_HI_SCAM_Deactivated();
        return 0;
    }

    NATIVE_INT_TYPE StereoCamComponentImpl ::
      setCallbackType(CallbackType type)
    {
        return 0;
    }

} // end namespace SnapdragonFlight
