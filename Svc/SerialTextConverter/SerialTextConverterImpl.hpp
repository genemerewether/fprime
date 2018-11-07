// ======================================================================
// \title  SerialTextConverterImpl.hpp
// \author Gorang Gandhi
// \brief  hpp file for SerialTextConverter component implementation class
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

#ifndef SERIALTEXTCONVERTERIMPL_HPP
#define SERIALTEXTCONVERTERIMPL_HPP

#include "Svc/SerialTextConverter/SerialTextConverterComponentAc.hpp"
#include <Drv/LinuxSerialDriver/LinuxSerialDriverComponentImplCfg.hpp>
#include <Svc/SerialTextConverter/SerialTextConverterImplCfg.hpp>
#include <Svc/ActiveTextLogger/LogFile.hpp>


namespace Svc {

  class SerialTextConverterComponentImpl :
    public SerialTextConverterComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object SerialTextConverter
      //!
      SerialTextConverterComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object SerialTextConverter
      //!
      void init(
              NATIVE_INT_TYPE queueDepth, /*!< The queue depth*/
              NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object SerialTextConverter
      //!
      ~SerialTextConverterComponentImpl(void);

      //!< A function that will be called before the event loop is entered
      virtual void preamble(void);

      //!  \brief Set log file and max size
      //!
      //!  This is to create a optional log file to write all the messages to.
      //!  The file will not be written to once the max size is hit.
      //!  The file will only be written to if in FILE mode, which is not enabled by default
      //!  FILE mode is enabled by ground command.
      //!
      //!  \param fileName The name of the file to create.  Must be less than 80 characters.
      //!  \param maxSize The max size of the file
      //!
      //!  \return true if creating the file was successful, false otherwise
      bool set_log_file(const char* fileName, const U32 maxSize, const U32 maxBackups = 10);

    PRIVATE:

        // ----------------------------------------------------------------------
        // Prohibit Copying
        // ----------------------------------------------------------------------

        /*! \brief Copy constructor
         *
         */
        SerialTextConverterComponentImpl(const SerialTextConverterComponentImpl&);

        /*! \brief Copy assignment operator
         *
         */
        SerialTextConverterComponentImpl& operator=(const SerialTextConverterComponentImpl&);

        // ----------------------------------------------------------------------
        // Constants/Types
        // ----------------------------------------------------------------------

        // ----------------------------------------------------------------------
        // Helper Methods
        // ----------------------------------------------------------------------

        /*! \brief Processes serial read status
         *
         */
        bool process_serial_read(Drv::SerialReadStatus status);

        // ----------------------------------------------------------------------
        // Handler implementations for user-defined typed input ports
        // ----------------------------------------------------------------------

        //! Handler for input port SerReadPort
        //
        virtual void SerReadPort_handler(
            NATIVE_INT_TYPE portNum, /*!< The port number*/
            Fw::Buffer &serBuffer, /*!< Buffer containing data*/
            Drv::SerialReadStatus &status /*!< Status of read*/
        );

        // ----------------------------------------------------------------------
        // Command handlers to implement
        // ----------------------------------------------------------------------

        //! Handler for command STC_SET_MODE
        /* Command to put in FILE (write texts to a file) or EVR (write texts to Evrs) mode. */
        virtual void STC_SET_MODE_cmdHandler(
            FwOpcodeType opCode, /*!< The opcode*/
            U32 cmdSeq, /*!< The command sequence number*/
            StcMode mode
        );

        // ----------------------------------------------------------------------
        // Member Variables
        // ----------------------------------------------------------------------

        // Wrappers for internal buffers used for reading data from serial driver component:
        // Note, these multiple buffers are rotated through each serial read:
        Fw::Buffer serial_buffs[DR_MAX_NUM_BUFFERS];

        // Data storage for serial read rotating buffers:
        // These buffers are rotated through for each serial read call.
        BYTE serial_raw_buffs[DR_MAX_NUM_BUFFERS][STC_READ_BUFF_SIZE];

        // The optional file to text logs to:
        Svc::LogFile m_log_file;

        // The current mode (write to EVRs or log):
        StcMode m_mode;

    };

} // end namespace Svc

#endif
