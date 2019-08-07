// ====================================================================== 
// \title  TeeImpl.cpp
// \author parallels
// \brief  cpp file for Tee component implementation class
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


#include <Svc/Tee/TeeComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

#include <stdio.h>

//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#define DEBUG_PRINT(x,...)

namespace Svc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  TeeComponentImpl ::
#if FW_OBJECT_NAMES == 1
    TeeComponentImpl(
        const char *const compName
    ) :
      TeeComponentBase(compName)
#else
    TeeImpl(void)
#endif
  {

  }

  void TeeComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    ) 
  {
    TeeComponentBase::init(instance);
  }

  TeeComponentImpl ::
    ~TeeComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined serial input ports
  // ----------------------------------------------------------------------

  void TeeComponentImpl ::
    DataIn_handler(
        NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
    )
  {
      DEBUG_PRINT("tee handling %d\n", portNum);
      NATIVE_INT_TYPE idx;
      for (idx = 0; idx < this->getNum_DataOut_OutputPorts(); idx++) {
          if (isConnected_DataOut_OutputPort(idx)) {
              this->DataOut_out(idx, Buffer);
              DEBUG_PRINT("tee sending %d on %d\n", portNum, idx);
              Buffer.resetDeser();
          }
      }
  }

} // end namespace Svc
