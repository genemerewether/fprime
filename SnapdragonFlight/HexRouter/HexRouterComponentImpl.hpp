// ======================================================================
// \title  HexRouterImpl.hpp
// \author mereweth
// \brief  hpp file for HexRouter component implementation class
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

#ifndef HexRouter_HPP
#define HexRouter_HPP

#include "SnapdragonFlight/HexRouter/HexRouterComponentAc.hpp"
#include <SnapdragonFlight/HexRouter/HexRouterComponentImplCfg.hpp>
#include <Os/Mutex.hpp>

namespace SnapdragonFlight {

  class HexRouterComponentImpl :
    public HexRouterComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object HexRouter
      //!
      HexRouterComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object HexRouter
      //!
      void init(
          const NATIVE_INT_TYPE queueDepth, /*!< The queue depth*/
          const NATIVE_INT_TYPE msgSize, /*!< The message size*/
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object HexRouter
      //!
      ~HexRouterComponentImpl(void);

    void startBuffReadThread(NATIVE_INT_TYPE priority,
                             NATIVE_INT_TYPE stackSize,
                             NATIVE_INT_TYPE cpuAffinity);

    void startPortReadThread(NATIVE_INT_TYPE priority,
                             NATIVE_INT_TYPE stackSize,
                             NATIVE_INT_TYPE cpuAffinity);

    void quitReadThreads(void);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      // TODO(mereweth) - add writeBufferRecv and writeBufferSend (return port)?

      //! Handler implementation for readBufferRecv
      //!
      void readBufferRecv_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer Buffer
      );

      //! Handler implementation for Sched
      //!
      void Sched_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined serial input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for KraitPortsIn
      //!
      void KraitPortsIn_handler(
          NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
      );

    // ----------------------------------------------------------------------
    // Implementations of class methods
    // ----------------------------------------------------------------------

      static void hexBuffReadTaskEntry(void * ptr);

      static void hexPortReadTaskEntry(void * ptr);

      Os::Task m_portReadTask; //!< task instance for thread to read ports
      Os::Task m_buffReadTask; //!< task instance for thread to read generic buffers

      struct BufferSet {
          Fw::Buffer readBuffer; //!< buffers for port reads
          bool available; //!< is buffer available?
      } m_buffSet[RECEIVE_BUFFER_POOL_SIZE];

      // input buffers
      BYTE m_inputBuff[RECEIVE_BUFFER_POOL_SIZE][RECEIVE_BUFFER_SIZE]; //!< locally allocated buffer
      Fw::Buffer m_inputBuffObj[RECEIVE_BUFFER_POOL_SIZE]; //!< input buffer objects

      Os::Mutex m_readBuffMutex;

      bool m_quitReadThreads; //!< flag to quit threads

      U32 m_numDecodeErrors; //!< number of buffer decoder errors
      U32 m_numBadSerialPortCalls;  //<! number of bad Serial port calls, ie bad serialize status returned

      U32 m_numPackets; //!< number of received packets
      U32 m_numInvalidPorts; //!< number of invalid ports

  };

} // end namespace SnapdragonFlight

#endif
