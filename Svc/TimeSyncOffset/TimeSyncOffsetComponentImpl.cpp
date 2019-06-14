// ======================================================================
// \title  TimeSyncOffsetComponentImpl.cpp
// \author kedelson
// \brief  cpp file for TimeSyncOffset component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================


#include <Svc/TimeSyncOffset/TimeSyncOffsetComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace Svc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  TimeSyncOffsetComponentImpl ::
#if FW_OBJECT_NAMES == 1
    TimeSyncOffsetComponentImpl(
        const char *const compName
    ) :
      TimeSyncOffsetComponentBase(compName)
#else
    TimeSyncOffsetComponentImpl(void)
#endif
  {

  }

  void TimeSyncOffsetComponentImpl ::
    init(
        const NATIVE_INT_TYPE queueDepth,
        const NATIVE_INT_TYPE instance
    )
  {
    TimeSyncOffsetComponentBase::init(queueDepth, instance);
  }

  TimeSyncOffsetComponentImpl ::
    ~TimeSyncOffsetComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void TimeSyncOffsetComponentImpl ::
    SchedIn_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    // TODO

    // output low pulse on port 0, does port num matter?
    // how do i make the sendpulsetime a global within the class?
    // do I init gpio out high in the constructor?
    const int PORT_NUM = 0;
    Fw::Time SendPulseTime = this->getTime();
    this->GPIOPulse_out(PORT_NUM, false);
  }

  void TimeSyncOffsetComponentImpl ::
    LLTime_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Time &time
    )
  {
    // TODO

    // calculate time offset
    // make sendpulsetime global???
    Fw::Time offset = this->SendPulseTime - time;
    this->Offset_out(offset);

    // send tlm message, takes F64
    // (float) offset
    this->tlmWrite_LLOffset((float) offset);

    // reset gpio pulse high
    this->GPIOPulse_out(0, true);


    // MathOpTlm opTlm;
    // MathOperation opPort;
    // MathOpEv opEv;
    // switch (operation) {
    //   case ADD:
    //       opTlm = ADD_TLM;
    //       opPort = MATH_ADD;
    //       opEv = ADD_EV;
    //       break;
    //   case SUBTRACT:
    //       opTlm = SUB_TLM;
    //       opPort = MATH_SUB;
    //       opEv = SUB_EV;
    //       break;
    //   case MULTIPLY:
    //       opTlm = MULT_TLM;
    //       opPort = MATH_MULTIPLY;
    //       opEv = MULT_EV;
    //       break;
    //   case DIVIDE:
    //       opTlm = DIV_TLM;
    //       opPort = MATH_DIVIDE;
    //       opEv = DIV_EV;
    //       break;
    //   default:
    //       FW_ASSERT(0,operation);
    //       break;
    // }
    // this->tlmWrite_MS_OP(opTlm);
    // this->tlmWrite_MS_VAL1(val1);
    // this->tlmWrite_MS_VAL2(val2);
    // this->log_ACTIVITY_LO_MS_COMMAND_RECV(val1,val2,opEv);
    // this->mathOut_out(0,val1,val2,opPort);
    // // reply with completion status
    // this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_OK);

  }

} // end namespace Svc
