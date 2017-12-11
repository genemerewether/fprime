// ====================================================================== 
// \title  KraitRouterImpl.cpp
// \author vagrant
// \brief  cpp file for KraitRouter component implementation class
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


#include <SnapdragonFlight/KraitRouter/KraitRouterComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace SnapdragonFlight {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  KraitRouterComponentImpl ::
#if FW_OBJECT_NAMES == 1
    KraitRouterComponentImpl(
        const char *const compName
    ) :
      KraitRouterComponentBase(compName)
#else
    KraitRouterImpl(void)
#endif
  {

  }

  void KraitRouterComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    ) 
  {
    KraitRouterComponentBase::init(instance);
  }

  KraitRouterComponentImpl ::
    ~KraitRouterComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void KraitRouterComponentImpl ::
    Sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    // TODO
  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined serial input ports
  // ----------------------------------------------------------------------

  void KraitRouterComponentImpl ::
    HexPortsIn_handler(
        NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
    )
  {
    // TODO
  }

} // end namespace SnapdragonFlight
