// ======================================================================
// \title  SerialTextConverterImpl.cpp
// \author Gorang Gandhi
// \brief  cpp file for SerialTextConverter component implementation class
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


#include <Svc/SerialTextConverter/SerialTextConverterImpl.hpp>
#include <algorithm>


namespace Svc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  SerialTextConverterComponentImpl ::
#if FW_OBJECT_NAMES == 1
    SerialTextConverterComponentImpl(
        const char *const compName
    ) :
      SerialTextConverterComponentBase(compName),
#else
    SerialTextConverterImpl(void) :
#endif
    m_log_file(), m_mode(EVR_MODE)
  {

      // Clear out buffer memory:
      memset(this->serial_raw_buffs, 0, sizeof(this->serial_raw_buffs));

      // Set up buffer wrappers for serial reads:
      for (U32 i=0; i < DR_MAX_NUM_BUFFERS; ++i) {

          this->serial_buffs[i].set(0, i+1, reinterpret_cast<U64>(this->serial_raw_buffs[i]), STC_READ_BUFF_SIZE);
      }
  }

  void SerialTextConverterComponentImpl ::
    init(
            NATIVE_INT_TYPE queueDepth, /*!< The queue depth*/
            NATIVE_INT_TYPE instance /*!< The instance number*/
    )
  {
    SerialTextConverterComponentBase::init(queueDepth,instance);
  }

  SerialTextConverterComponentImpl ::
    ~SerialTextConverterComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Helper Methods
  // ----------------------------------------------------------------------

  void SerialTextConverterComponentImpl::preamble(void)
  {
      // Give the serial driver the available buffers for serial reads:
      for (U32 i=0; i < DR_MAX_NUM_BUFFERS; ++i) {

          this->SerialBufferSend_out(0, this->serial_buffs[i]);
      }
  }

  bool SerialTextConverterComponentImpl::process_serial_read(Drv::SerialReadStatus status)
  {
      bool bad_read = false;

      FW_ASSERT(status < Drv::SerialReadStatus_MAX);

      if (status != Drv::SER_OK) {

          this->log_WARNING_LO_STC_SerialReadBadStatus(status);
          bad_read = true;
      }

      return bad_read;
  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void SerialTextConverterComponentImpl::SerReadPort_handler(NATIVE_INT_TYPE portNum,
                                                             Fw::Buffer &serBuffer,
                                                             Drv::SerialReadStatus &status)
  {

      // If the serial read had a good status:
      if (!this->process_serial_read(status)) {

          const U32 buff_size = serBuffer.getsize();
          const char *const buf = reinterpret_cast<char*>(serBuffer.getdata());

          // If in FILE mode, then write to file:
          if (this->m_mode == FILE_MODE) {

              (void) this->m_log_file.write_to_log(buf, buff_size);  // TODO Ignoring return status
          }
          // Otherwise, write to EVRs:
          else {

              /*
               * Assumptions:
               * - The reads can chunk up the string in smaller pieces, even
               * though sent as one string by the FC
               * - Events can only have up to 80 char string args, so must break up
               * the string if needed
               *
               * Loop through bytes in passed buffer
               *    Scan for null
               *        If null found
               *            Put that chunk in event up to 80 char
               *        else
               *            Put up to 80 char in event
               *    Adjust pointer into buffer
               *
               * For simplicity, can put any remainder into Event instead of storing,
               * as I do not think it will be likely to have a starting of new
               * string in the same serial passed buffer.
               *
               */
              U32 buff_idx = 0;

              // Loop while there is still data left to process in the serial buffer:
              while(buff_idx < buff_size) {

                  // Subtracting 1 from FW_LOG_STRING_MAX_SIZE, so that there is room for null terminator:
                  U32 max_buff_idx = std::min(buff_idx+static_cast<U32>(FW_LOG_STRING_MAX_SIZE-1),
                                              buff_size);

                  // Scan for null in the largest chunk size we can have:
                  U32 i;
                  bool null_fnd = false;
                  for (i = buff_idx; i < max_buff_idx; ++i) {
                      if (buf[i] == 0) {
                          null_fnd = true;
                          break;
                      }
                  }

                  // Sanity check:
                  FW_ASSERT(buff_idx < buff_size,buff_idx,buff_size);

                  // Copy data over if needed:
                  char data_buff[FW_LOG_STRING_MAX_SIZE];
                  U32 evr_buff_size = i-buff_idx;

                  if (evr_buff_size > 0) {

                      // Always add null at the end to simply the code, and make sure strncpy() does overrun
                      // the buffer in the LogStringArg ctor call
                      FW_ASSERT(evr_buff_size < FW_LOG_STRING_MAX_SIZE,evr_buff_size,FW_LOG_STRING_MAX_SIZE);
                      memcpy(data_buff, buf+buff_idx, evr_buff_size);
                      data_buff[evr_buff_size] = '\0';

                      // Create Event string arg:
                      //
                      // Simply use the data directly.  Assuming no format and all valid ASCII
                      Fw::LogStringArg data(data_buff);

                      // Event for the serial data converted to string:
                      this->log_ACTIVITY_LO_STC_SerialText(data);
                  }

                  // Adjust buff_idx to past previous chunk:
                  buff_idx = null_fnd ? i+1 : i;

              } // ends while

          } // ends else (in EVR mode)

      } // ends if (!this->process_serial_read(status))

      // Give the buffer back to the serial driver b/c we are done with it:
      // Note, Must reset the size back to max, as the serial driver adjusted
      // it to the amount of data it read
      serBuffer.setsize(STC_READ_BUFF_SIZE);
      this->SerialBufferSend_out(0, serBuffer);

  }

  // ----------------------------------------------------------------------
  // Command handlers to implement
  // ----------------------------------------------------------------------

  void SerialTextConverterComponentImpl::STC_SET_MODE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, StcMode mode)
  {

      bool status = true;

      // Can't go into file mode if there is no open file:
      if ((mode == FILE_MODE) && !this->m_log_file.m_openFile) {

          status = false;
          this->log_WARNING_LO_STC_SetMode_Cmd_Invalid();
      }
      else {

          this->m_mode = mode;
          this->log_ACTIVITY_LO_STC_SetMode_Cmd_Sent(static_cast<StcModeEv>(mode));

          if (mode == FILE_MODE) {
              Fw::LogStringArg log_file(this->m_log_file.m_fileName.toChar());
              this->log_ACTIVITY_LO_STC_File(log_file);
          }
      }

      this->cmdResponse_out(opCode, cmdSeq,
                            status ? Fw::COMMAND_OK : Fw::COMMAND_EXECUTION_ERROR);
  }

  // ----------------------------------------------------------------------
  // Helper Methods
  // ----------------------------------------------------------------------

  bool SerialTextConverterComponentImpl::set_log_file(const char* fileName, const U32 maxSize, const U32 maxBackups)
  {
      FW_ASSERT(fileName != NULL);

      return this->m_log_file.set_log_file(fileName, maxSize, maxBackups);
  }


} // end namespace Svc
