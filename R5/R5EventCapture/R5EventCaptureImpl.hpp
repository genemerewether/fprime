// ====================================================================== 
// \title  R5EventCaptureImpl.hpp
// \author kubiak
// \brief  hpp file for R5EventCapture component implementation class
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

#ifndef R5EventCapture_HPP
#define R5EventCapture_HPP

#include "R5/R5EventCapture/R5EventCaptureComponentAc.hpp"
#include <Fw/Types/MemAllocator.hpp>

namespace R5 {

  class R5EventCaptureComponentImpl :
    public R5EventCaptureComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object R5EventCapture
      //!
      R5EventCaptureComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object R5EventCapture
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      void initDriver(const NATIVE_UINT_TYPE memSize,
                      Fw::MemAllocator& allocator);

      //! Destroy object R5EventCapture
      //!
      ~R5EventCaptureComponentImpl(void);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for time
      //!
      void time_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Time &time /*!< The U32 cmd argument*/
      );


    };

} // end namespace 

#endif
