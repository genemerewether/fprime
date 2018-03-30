// ======================================================================
// \title  MPU9250Impl.hpp
// \author mereweth
// \brief  hpp file for MPU9250 component implementation class
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

#ifndef MPU9250_HPP
#define MPU9250_HPP

#include "Drv/IMU/MPU9250/MPU9250ComponentAc.hpp"
#include "Drv/IMU/MPU9250/MPU9250ComponentImplCfg.hpp"

namespace Drv {

  class MPU9250ComponentImpl :
    public MPU9250ComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object MPU9250
      //!
      MPU9250ComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object MPU9250
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object MPU9250
      //!
      ~MPU9250ComponentImpl(void);

    PRIVATE:

    // ----------------------------------------------------------------------
    // Constants/Types
    // ----------------------------------------------------------------------

    /*! \brief MPU9250 Initialization State
     *
     */
    enum InitState {
        INIT_WAITING,
        INIT_COMPLETE,
    } initState;

    // ----------------------------------------------------------------------
    // Handler implementations for user-defined typed input ports
    // ----------------------------------------------------------------------

    //! Handler implementation for sched
    //!
    void sched_handler(
        const NATIVE_INT_TYPE portNum, /*!< The port number*/
        NATIVE_UINT_TYPE context /*!< The call order*/
    );

    //! Handler implementation for pingIn
    //!
    void pingIn_handler(
        const NATIVE_INT_TYPE portNum, /*!< The port number*/
        U32 key /*!< Value to return to pinger*/
    );


  };

} // end namespace Drv

#endif
