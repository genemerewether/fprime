// ======================================================================
// \title  MPU9250Impl.cpp
// \author mereweth
// \brief  cpp file for MPU9250 component implementation class
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


#include <Drv/IMU/MPU9250/MPU9250ComponentImpl.hpp>
#include <Drv/IMU/MPU9250/MPU9250Reg.hpp>
#include "Fw/Types/BasicTypes.hpp"

#ifdef BUILD_DSPAL
#include <HAP_farf.h>
#define DEBUG_PRINT(x,...) FARF(ALWAYS,x,##__VA_ARGS__);
#else
#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#endif

//#undef DEBUG_PRINT
//#define DEBUG_PRINT(x,...)

namespace Drv {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

    MPU9250ComponentImpl ::
  #if FW_OBJECT_NAMES == 1
      MPU9250ComponentImpl(
          const char *const compName,
          bool useMagnetometer
      ) :
        MPU9250ComponentBase(compName),
  #else
        MPU9250ComponentBase(void),
  #endif
        m_useMagnetometer(useMagnetometer)
    {

    }

    void MPU9250ComponentImpl ::
      init(
          const NATIVE_INT_TYPE instance
      )
    {
      MPU9250ComponentBase::init(instance);
    }

    MPU9250ComponentImpl ::
      ~MPU9250ComponentImpl(void)
    {

    }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

    void MPU9250ComponentImpl ::
      sched_handler(
          const NATIVE_INT_TYPE portNum,
          NATIVE_UINT_TYPE context
      )
    {
        if (context == MPU9250_SCHED_CONTEXT_OPERATE) {
            switch(m_initState) {
                case INIT_RESET:
                    // this->set SPI clock to 1 MHZ
                    //this->SpiReadWrite_out()
                    break;
                case INIT_COMPLETE:
                    break;
                case INIT_ERROR:
                    break;
                default:
                    FW_ASSERT(0, m_initState);
            }
        }
        else if (context == MPU9250_SCHED_CONTEXT_TLM) {

        }
    }

    void MPU9250ComponentImpl ::
      pingIn_handler(
          const NATIVE_INT_TYPE portNum,
          U32 key
      )
    {
        // TODO
    }

} // end namespace Drv
