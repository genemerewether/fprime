// ====================================================================== 
// \title  RosSeqImpl.cpp
// \author genemerewether
// \brief  cpp file for RosSeq component implementation class
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


#include <ROS/RosSeq/RosSeqComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace ROS {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  RosSeqComponentImpl ::
#if FW_OBJECT_NAMES == 1
    RosSeqComponentImpl(
        const char *const compName
    ) :
      RosSeqComponentBase(compName)
#else
    RosSeqImpl(void)
#endif
  {

  }

  void RosSeqComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    ) 
  {
    RosSeqComponentBase::init(instance);
  }

  RosSeqComponentImpl ::
    ~RosSeqComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void RosSeqComponentImpl ::
    sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    // TODO
  }

  void RosSeqComponentImpl ::
    pingIn_handler(
        const NATIVE_INT_TYPE portNum,
        U32 key
    )
  {
    // TODO
  }

  void RosSeqComponentImpl ::
    seqDoneIn_handler(
        const NATIVE_INT_TYPE portNum,
        FwOpcodeType opCode,
        U32 cmdSeq,
        Fw::CommandResponse response
    )
  {
    // TODO
  }

} // end namespace ROS
