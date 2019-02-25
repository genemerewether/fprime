// ======================================================================
// \title  MVCamComponentImplStub.cpp
// \author mereweth
// \brief  cpp file for MVCam component stub class
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

#include <SnapdragonFlight/MVCam/MVCamComponentImpl.hpp>
#include <SnapdragonFlight/MVCam/MVCamComponentImplCfg.hpp>

#include <stdio.h> // TODO(mereweth@jpl.nasa.gov) - remove the debug prints
#include <sys/time.h>

#include <stdlib.h>

#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
//#define DEBUG_PRINT(x,...)

namespace SnapdragonFlight {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  MVCamComponentImpl ::
    ~MVCamComponentImpl(void)
  {
      m_initialized = false;
  }

  // ----------------------------------------------------------------------
  // Private method implementations
  // ----------------------------------------------------------------------

    void MVCamComponentImpl ::
      preamble(void)
    {
        m_initialized = true;
#ifdef ALWAYS_CYCLE
        //(void) activate();
#endif //ALWAYS_CYCLE
    }

    void MVCamComponentImpl ::
      finalizer(void)
    {
        //(void) deactivate();
    }

    NATIVE_INT_TYPE MVCamComponentImpl ::
      activate(void)
    {
        //m_captureParamsLock.lock();
        m_activated = true;
        //m_captureParamsLock.unLock();
        this->log_ACTIVITY_HI_MVCAM_Activated();

        // TODO(mereweth) - start timer at 30 Hz to send fake images?

        /*BYTE* ptr = (BYTE*)buff.getdata();
        ptr[MVCAM_IMAGE_WIDTH*10 + 320] = 255;
        ptr[MVCAM_IMAGE_WIDTH*20 + 320] = 255;
        ptr[MVCAM_IMAGE_WIDTH*50 + 320] = 255;
        */

        return 0;
    }

    NATIVE_INT_TYPE MVCamComponentImpl ::
      deactivate(void)
    {
        //m_captureParamsLock.lock();
        m_activated = false;
        //m_captureParamsLock.unLock();
        this->log_ACTIVITY_HI_MVCAM_Deactivated();
        return 0;
    }

    NATIVE_INT_TYPE MVCamComponentImpl ::
      setCallbackType(CallbackType type)
    {
        return 0;
    }

} // end namespace SnapdragonFlight
