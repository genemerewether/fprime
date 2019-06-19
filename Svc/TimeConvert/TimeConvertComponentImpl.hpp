// ======================================================================
// \title  TimeConvertComponentImpl.hpp
// \author mereweth
// \brief  hpp file for TimeConvert component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================

#ifndef TimeConvert_HPP
#define TimeConvert_HPP

#include "Svc/TimeConvert/TimeConvertComponentAc.hpp"

namespace Svc {

  class TimeConvertComponentImpl :
    public TimeConvertComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object TimeConvert
      //!
      TimeConvertComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object TimeConvert
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object TimeConvert
      //!
      ~TimeConvertComponentImpl(void);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for SchedIn
      //!
      void SchedIn_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

      //! Handler implementation for ClockTimes
      //!
      void ClockTimes_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Time time1, /*!< first time*/
          Fw::Time time2 /*!< second time*/
      );

      //! Handler implementation for ConvertTime
      //!
      Fw::Time ConvertTime_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Time time, /*!< The time to convert*/
          U32 timeBase, /*!< TimeBase to convert to*/
          U32 timeContext /*!< TimeContext to convert to*/
      );


    };

} // end namespace Svc

#endif
