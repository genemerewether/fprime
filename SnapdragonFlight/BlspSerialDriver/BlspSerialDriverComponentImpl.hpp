// ====================================================================== 
// \title  BlspSerialDriverImpl.hpp
// \author tcanham
// \brief  hpp file for BlspSerialDriver component implementation class
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

#ifndef BlspSerialDriver_HPP
#define BlspSerialDriver_HPP

#include <SnapdragonFlight/BlspSerialDriver/BlspSerialDriverComponentAc.hpp>
#include <SnapdragonFlight/BlspSerialDriver/BlspSerialDriverComponentImplCfg.hpp>
#include <Os/Mutex.hpp>
#include <Os/Task.hpp>

namespace SnapdragonFlight {

  class BlspSerialDriverComponentImpl :
    public BlspSerialDriverComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object BlspSerialDriver
      //!
      BlspSerialDriverComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object BlspSerialDriver
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Configure UART parameters
      typedef enum BAUD_RATE {
          BAUD_115K,
          BAUD_230K,
          BAUD_921K
      } UartBaudRate ;

      typedef enum FLOW_CONTROL {
          NO_FLOW,
          HW_FLOW
      } UartFlowControl;

      // Open device tty-<device> on DSP with specified baud and flow control.
      void open(NATIVE_UINT_TYPE device, UartBaudRate baud, UartFlowControl fc, bool block);

      //! start the serial poll thread.
      //! buffSize is the max receive buffer size
      //!
      void startReadThread(NATIVE_INT_TYPE priority, NATIVE_INT_TYPE stackSize, NATIVE_INT_TYPE cpuAffinity = -1);

      //! Quit thread
      void quitReadThread(void);

      //! Destroy object BlspSerialDriver
      //!
      ~BlspSerialDriverComponentImpl(void);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for serialSend
      //!
      void serialSend_handler(
              NATIVE_INT_TYPE portNum, /*!< The port number*/
              Fw::Buffer &serBuffer
          );

      //! Handler implementation for readBufferSend
      //!
      void readBufferSend_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &fwBuffer
      );

      NATIVE_INT_TYPE m_fd; //!< file descriptor returned for DSP I/O device
      NATIVE_INT_TYPE m_device; //!< original device ID for EVRs

      //! This method will be called by the new thread to wait for input on the serial port.
      static void serialReadTaskEntry(void * ptr);

      Os::Task m_readTask; //!< task instance for thread to read serial port

      struct BufferSet {
          Fw::Buffer readBuffer; //!< buffers for port reads
          bool available; //!< is buffer available?
      } m_buffSet[MAX_NUM_BUFFERS];

      Os::Mutex m_readBuffMutex;

      bool m_quitReadThread; //!< flag to quit thread

    };

} // end namespace Snapdragon

#endif
