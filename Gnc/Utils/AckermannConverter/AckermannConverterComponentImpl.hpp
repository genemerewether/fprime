// ====================================================================== 
// \title  AckermannConverterImpl.hpp
// \author mereweth
// \brief  hpp file for AckermannConverter component implementation class
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

#ifndef AckermannConverter_HPP
#define AckermannConverter_HPP

#include "Gnc/Utils/AckermannConverter/AckermannConverterComponentAc.hpp"

namespace Gnc {

  class AckermannConverterComponentImpl :
    public AckermannConverterComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object AckermannConverter
      //!
      AckermannConverterComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object AckermannConverter
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object AckermannConverter
      //!
      ~AckermannConverterComponentImpl(void);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for ackermann
      //!
      void ackermann_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::ackermann_msgs::AckermannDriveStamped &AckermannDriveStamped 
      );


    };

} // end namespace Gnc

#endif
