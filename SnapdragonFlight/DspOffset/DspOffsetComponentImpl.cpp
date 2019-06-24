// ======================================================================
// \title  DspOffsetComponentImpl.cpp
// \author mereweth
// \brief  cpp file for DspOffset component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================


#include <SnapdragonFlight/DspOffset/DspOffsetComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace SnapdragonFlight {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  DspOffsetComponentImpl ::
#if FW_OBJECT_NAMES == 1
    DspOffsetComponentImpl(
        const char *const compName
    ) :
      DspOffsetComponentBase(compName)
#else
    DspOffsetComponentImpl(void)
#endif
  {

  }

  void DspOffsetComponentImpl ::
    init(
        const NATIVE_INT_TYPE queueDepth,
        const NATIVE_INT_TYPE instance
    )
  {
    DspOffsetComponentBase::init(queueDepth, instance);
  }

  DspOffsetComponentImpl ::
    ~DspOffsetComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void DspOffsetComponentImpl ::
    SchedIn_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    // TODO
  }

} // end namespace SnapdragonFlight
