// ====================================================================== 
// \title  SerializePortLoggerImpl.cpp
// \author tcanham
// \brief  cpp file for SerializePortLogger component implementation class
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


#include <Svc/SerLogger/SerLoggerComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace Svc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  SerLoggerComponentImpl ::
#if FW_OBJECT_NAMES == 1
    SerLoggerComponentImpl(
        const char *const compName
    ) :
    SerLoggerComponentBase(compName)
#else
    SerializePortLoggerImpl(void)
#endif
    ,m_streamId(AFL_SERIAL_DATA)
  {

  }

  void SerLoggerComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    ) 
  {
      SerLoggerComponentBase::init(instance);
  }

  SerLoggerComponentImpl ::
    ~SerLoggerComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined serial input ports
  // ----------------------------------------------------------------------

  void SerLoggerComponentImpl ::
    SerPortIn_handler(
        NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
    )
  {
      // copy incoming serialized buffer to file logger
      this->m_packet.resetSer();
      // serialize stream ID
      Fw::SerializeStatus stat = this->m_packet.serialize(static_cast<U8>(this->m_streamId));
      FW_ASSERT(Fw::FW_SERIALIZE_OK == stat,stat);
      // serialize buffer
      stat = this->m_packet.serialize(Buffer.getBuffAddr(),Buffer.getBuffLength(),true);
      FW_ASSERT(Fw::FW_SERIALIZE_OK == stat,stat);
      // send buffer
      this->LogOut_out(0,this->m_packet);
  }

  void SerLoggerComponentImpl::setStreamId(active_file_logger_stream_t stream) {
      this->m_streamId = stream;
  }


} // end namespace Svc
