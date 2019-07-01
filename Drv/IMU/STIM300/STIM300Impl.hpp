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
          S300_TS_SYNCED
      };

      enum STIM300FindPktStatus {
          S300_PKT_FOUND,
          S300_PKT_NONE,
          S300_PKT_LOST_SYNC
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

      //! Handler implementation for serialRead
      //!
      void serialRead_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &serBuffer, /*!< Buffer containing data*/
          Drv::SerialReadStatus &status /*!< Status of read*/
      );

      void runTsClear();
      void runTsWait();
      void runTsSynced();

      STIM300FindPktStatus findSTIMPkt(U8* buffer, const U32 bufferLength, U32& bytesRead_out, ROS::sensor_msgs::ImuNoCov& pkt_out);

      U8 m_uartBuffer[STIM_UART_BUFFER_SIZE];
      U32 m_uartBufferIdx;

      U32 m_pktCounter;

      STIM300TSState m_timeSyncState;

      U32 m_numReceivedPkts;

    };

} // end namespace 

#endif
