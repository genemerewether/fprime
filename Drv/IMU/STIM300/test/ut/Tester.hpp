// ====================================================================== 
// \title  STIM300/test/ut/Tester.hpp
// \author kubiak
// \brief  hpp file for STIM300 test harness implementation class
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

#ifndef TESTER_HPP
#define TESTER_HPP

#include "GTestBase.hpp"
#include "Drv/IMU/STIM300/STIM300Impl.hpp"

#include <Utils/RingBuffer/RingBuffer.h>

namespace Drv {

  class Tester :
    public STIM300GTestBase
  {

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

    public:

      //! Construct object Tester
      //!
      Tester(void);

      //! Destroy object Tester
      //!
      ~Tester(void);

    public:

      // ---------------------------------------------------------------------- 
      // Tests
      // ---------------------------------------------------------------------- 

      //! To do
      //!
      void nominalTest(void);

      void manyPackets(void);

    private:

      // ----------------------------------------------------------------------
      // Handlers for typed from ports
      // ----------------------------------------------------------------------

      //! Handler for from_IMU
      //!
      void from_IMU_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::sensor_msgs::ImuNoCov &ImuNoCov 
      );

      //! Handler for from_packetTime
      //!
      void from_packetTime_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Time &time /*!< The U32 cmd argument*/
      );

    private:

      // ----------------------------------------------------------------------
      // Helper methods
      // ----------------------------------------------------------------------

      //! Connect ports
      //!
      void connectPorts(void);

      //! Initialize components
      //!
      void initComponents(void);

      void eventFunc(Fw::Time time);
      void bytesFunc(U8* data, U32 len);
      void expectFunc(U8* data, U32 len, ROS::sensor_msgs::ImuNoCov imuPkt);

      void compareImuPkts(ROS::sensor_msgs::ImuNoCov& act,
                          ROS::sensor_msgs::ImuNoCov& exp);

    private:

      // ----------------------------------------------------------------------
      // Variables
      // ----------------------------------------------------------------------

      //! The component under test
      //!
      STIM300ComponentImpl component;

      std::mutex m_modelMutex;

      Utils::RingBuffer<Fw::Time, 100> m_eventRB;
      Utils::RingBuffer<ROS::sensor_msgs::ImuNoCov, 100> m_expPkts;


      U8 m_uartBufferData[256];
      Fw::ExternalSerializeBuffer m_uartExtBuffer;
      Fw::Buffer m_uartFwBuffer;

      U32 m_receivedPkts;
      U32 m_partialPkts;
  };

} // end namespace Drv

#endif
