// ====================================================================== 
// \title  ActuatorControlsImpl.hpp
// \author mereweth
// \brief  hpp file for ActuatorControls component implementation class
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

#ifndef ActuatorControls_HPP
#define ActuatorControls_HPP

#include "Drv/Mavlink/ActuatorControls/ActuatorControlsComponentAc.hpp"

namespace Drv {

  class ActuatorControlsComponentImpl :
    public ActuatorControlsComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object ActuatorControls
      //!
      ActuatorControlsComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object ActuatorControls
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object ActuatorControls
      //!
      ~ActuatorControlsComponentImpl(void);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for SerReadPort
      //!
      void SerReadPort_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &serBuffer, /*!< Buffer containing data*/
          SerialReadStatus &status /*!< Status of read*/
      );
    
      //! Handler implementation for pwmSetDuty
      //!
      void pwmSetDuty_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          PwmSetDutyCycle pwmSetDutyCycle 
      );

      //! Handler implementation for sched
      //!
      void sched_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );


    };

} // end namespace Drv

#endif
