// ====================================================================== 
// \title  STIM300Impl.hpp
// \author kubiak
// \brief  hpp file for STIM300 component implementation class
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

#ifndef STIM300_HPP
#define STIM300_HPP

#include <Fw/Types/BasicTypes.hpp>
#include <Fw/Time/Time.hpp>
#include <Fw/Types/Serializable.hpp>

#include "Drv/IMU/STIM300/STIM300ComponentAc.hpp"
#include <Drv/IMU/STIM300/STIM300ImplCfg.hpp>
#include <Drv/IMU/STIM300/STIM300Pkt.hpp>

#include <Gnc/quest_gnc/include/quest_gnc/utils/ringbuffer.h>

namespace Drv {

  class STIM300ComponentImpl :
    public STIM300ComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object STIM300
      //!
      STIM300ComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object STIM300
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object STIM300
      //!
      ~STIM300ComponentImpl(void);

      PRIVATE:

      static const U32 stimCrc32Lookup[256];

      struct STIM300DummyMap {
          U8 ident;
          U8 dummyBytes;
      };

      static const STIM300DummyMap dummyBytesMap[2];

      enum STIM300TSState {
          S300_TS_CLEAR,
          S300_TS_WAIT,
          S300_TS_CHECK,
          S300_TS_SYNCED,
          S300_TS_NOSYNC
      };

      enum STIM300FindPktStatus {
          S300_PKT_FOUND,
          S300_PKT_NONE,
          S300_PKT_LOST_SYNC
      };

      enum STIM300GatherStatus {
          S300_GATHER_OK,
          S300_GATHER_DROPPED
      };

      enum STIM300ReadUartStatus {
          S300_UART_OK,
          S300_UART_FULL,
          S300_UART_ERR
      };

      enum STIM300ConsistencyStatus {
          S300_CONSISTENCY_OK,
          S300_CONSISTENCY_INVALID
      };

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------


      //! Handler implementation for sched
      //!
      void sched_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

      void runTsClear();
      void runTsWait();
      void runTsCheck();
      void runTsSynced();
      void runTsNosync();

      STIM300GatherStatus gatherEvents(void);

      STIM300ConsistencyStatus verifyConsistency(void);

      STIM300FindPktStatus findSTIMPkt(U8* buffer, const U32 bufferLength, U32& bytesRead_out, ROS::sensor_msgs::ImuNoCov& pkt_out);

      STIM300ReadUartStatus readUartBytes(void);

      U8 m_pktBuffer[STIM_PKT_BUFFER_SIZE];
      U32 m_pktBufferIdx;

      Fw::Buffer m_uartFwBuffer;
      // Backing datastore for m_uartFwBuffer
      U8 m_uartBuffer[STIM_UART_BUFFER_SIZE];

      U32 m_pktCounter;

      STIM300TSState m_timeSyncState;

      U32 m_numReceivedPkts;

      U32 m_tsCheckCount;

      quest_gnc::ringbuffer<Fw::Time, STIM_MAX_EVENTS> m_eventsRing;


    };

} // end namespace 

#endif
