// ====================================================================== 
// \title  STIM300/test/ut/STIM300Model.hpp
// \author kubiak
// \brief  hpp file for STIM300 model class
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

#ifndef STIM300_MODEL_HPP
#define STIM300_MODEL_HPP

#include "Drv/IMU/STIM300/STIM300Impl.hpp"

namespace Drv {

 class STIM300Model {

    public:

      STIM300Model(const int rateHz,
                   std::function<void(Fw::Time)> push_event_fun,
                   std::function<void(U8*,U32)> push_bytes_fun,
                   std::function<void(U8*,U32,ROS::sensor_msgs::ImuNoCov)> push_exp);


      void run();

      void generatePkts(ROS::sensor_msgs::ImuNoCov& imuPkt,
                        Fw::Time& eventTime,
                        U8* stimData, U32& stimDataLen);

      void stop();
    private:

      ROS::sensor_msgs::ImuNoCov getImuPkt();

      int m_rateHz;

      std::function<void(Fw::Time)> m_pushEventFun;
      std::function<void(U8*,U32)> m_pushBytesFun;
      std::function<void(U8*,U32,ROS::sensor_msgs::ImuNoCov)> m_pushExpFun;

      bool m_should_stop;

      int m_seq_num;
 };
}

#endif
