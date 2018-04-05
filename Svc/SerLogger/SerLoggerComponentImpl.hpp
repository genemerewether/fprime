// ====================================================================== 
// \title  SerializePortLoggerImpl.hpp
// \author tcanham
// \brief  hpp file for SerializePortLogger component implementation class
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

#ifndef SerializePortLogger_HPP
#define SerializePortLogger_HPP

#include "Svc/SerLogger/SerLoggerComponentAc.hpp"
#include <Svc/ActiveFileLogger/ActiveFileLoggerStreams.hpp>

namespace Svc {

  class SerLoggerComponentImpl :
    public SerLoggerComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object SerializePortLogger
      //!
          SerLoggerComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object SerializePortLogger
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! set the stream ID for the component
      void setStreamId(active_file_logger_stream_t stream);

      //! Destroy object SerializePortLogger
      //!
      ~SerLoggerComponentImpl(void);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined serial input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for SerPortIn
      //!
      void SerPortIn_handler(
        NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
      );

      active_file_logger_stream_t m_streamId; //!< ID of stream for data

      Svc::ActiveFileLoggerPacket m_packet; //!< working buffer

    };

} // end namespace Svc

#endif
