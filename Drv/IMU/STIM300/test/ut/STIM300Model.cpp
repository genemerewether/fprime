// ====================================================================== 
// \title  STIM300/test/ut/STIM300Model.cpp
// \author kubiak
// \brief  cpp file for STIM300 model class
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

#include <thread>
#include <chrono>
#include "STIM300Model.hpp"

#include <math.h>

//#include <ROS/Gen/sensor_msgs/Types/ImuNoCovSerializableAc.hpp>

namespace Drv {

  STIM300Model::STIM300Model(const int rateHz,
                             std::function<void(Fw::Time)> push_event_fun,
                             std::function<void(U8*,U32)> push_bytes_fun,
                             std::function<void(U8*,U32,ROS::sensor_msgs::ImuNoCov)> push_exp_fun) :
      m_rateHz(rateHz),
      m_pushEventFun(push_event_fun),
      m_pushBytesFun(push_bytes_fun),
      m_pushExpFun(push_exp_fun),
      m_should_stop(false),
      m_seq_num(0)
  {
  }

  static U32 calcCrc(U8* data, U32 len, U32 dummyBytes) {

    // Check CRC
    U32 crc = 0xFFFFFFFF;

    for (int i = 0; i < len; i++) {
        crc = STIM300ComponentImpl::stimCrc32Lookup[((crc >> 24) ^ data[i]) & 0xFF] ^ (crc << 8);
    }

    // Add dummy 0 bytes for CRC
    for (int i = 0; i < dummyBytes; i++) {
        crc = STIM300ComponentImpl::stimCrc32Lookup[((crc >> 24) ^ 0) & 0xFF] ^ (crc << 8);
    }

    return crc;
  }

  static void doubleToTriplet(F64 val, U8* triplet, U32 scale)
  {
    F64 abs_val;
    U32 temp;

    abs_val = fabs(val);

    temp = static_cast<U32>(abs_val * scale);

    if (val < 0) {
        // Two's complement
        temp = (~temp) + 1;
    }

    triplet[0] = (temp >> 16) & 0xFF;
    triplet[1] = (temp >> 8) & 0xFF;
    triplet[2] = temp & 0xFF;
  }

  static void createStimPkt(ROS::sensor_msgs::ImuNoCov imuPkt, Fw::Time& eventTime, U8* pktData, U32& pktLen) {

    F64 gyro_x,gyro_y,gyro_z;
    F64 accel_x,accel_y,accel_z;
    F64 incl_x,incl_y,incl_z;

    int count;
    int idx;
    U32 crc;

    gyro_x = imuPkt.getangular_velocity().getx();
    gyro_y = imuPkt.getangular_velocity().gety();
    gyro_z = imuPkt.getangular_velocity().getz();

    accel_x = imuPkt.getlinear_acceleration().getx();
    accel_y = imuPkt.getlinear_acceleration().gety();
    accel_z = imuPkt.getlinear_acceleration().getz();

    incl_x = imuPkt.getlinear_acceleration().getx();
    incl_y = imuPkt.getlinear_acceleration().gety();
    incl_z = imuPkt.getlinear_acceleration().getz();

    count = imuPkt.getheader().getseq();

    idx = 0;
    pktData[idx] = 0x93;
    idx++;

    doubleToTriplet(gyro_x, &pktData[idx], 1 << 14);
    idx += 3;
    doubleToTriplet(gyro_y, &pktData[idx], 1 << 14);
    idx += 3;
    doubleToTriplet(gyro_z, &pktData[idx], 1 << 14);
    idx += 3;
    pktData[idx] = 0;
    idx++;

    doubleToTriplet(accel_x * 9.81, &pktData[idx], 1 << 19);
    idx += 3;
    doubleToTriplet(accel_y * 9.81, &pktData[idx], 1 << 19);
    idx += 3;
    doubleToTriplet(accel_z * 9.81, &pktData[idx], 1 << 19);
    idx += 3;
    pktData[idx] = 0;
    idx++;

    doubleToTriplet(incl_x * 9.81, &pktData[idx], 1 << 22);
    idx += 3;
    doubleToTriplet(incl_y * 9.81, &pktData[idx], 1 << 22);
    idx += 3;
    doubleToTriplet(incl_z * 9.81, &pktData[idx], 1 << 22);
    idx += 3;
    pktData[idx] = 0;
    idx++;

    pktData[idx] = count;
    idx++;

    // latency
    pktData[idx] = (516 >> 8) & 0xFF;
    idx++;
    pktData[idx] = 516 & 0xFF;
    idx++;

    pktData[idx] = 0;
    idx++;
    pktData[idx] = 0;
    idx++;
    pktData[idx] = 0;
    idx++;
    pktData[idx] = 0;
    idx++;

    pktLen = idx;

    crc = calcCrc(pktData, pktLen - 4, 2);


    pktData[pktLen - 4] = (crc >> 24) & 0xFF;
    pktData[pktLen - 3] = (crc >> 16) & 0xFF;
    pktData[pktLen - 2] = (crc >> 8) & 0xFF;
    pktData[pktLen - 1] = (crc) & 0xFF;

    eventTime = imuPkt.getheader().getstamp();
    eventTime.add(0, 516);
  }

  ROS::sensor_msgs::ImuNoCov STIM300Model::getImuPkt() {

      int seq_num = this->m_seq_num;

      Fw::Time time;
      ROS::sensor_msgs::ImuNoCov imuPkt;
      ROS::std_msgs::Header header;
      ROS::geometry_msgs::Quaternion dummy_quat(0,0,0,1);
      ROS::geometry_msgs::Vector3 gyro;
      ROS::geometry_msgs::Vector3 accel;

      time.set(TB_PROC_TIME, seq_num + 1, 0);

      header.set(seq_num % 256, time, 0);

      imuPkt.setheader(header);
      imuPkt.setorientation(dummy_quat);

      gyro.set(sin(seq_num),sin(seq_num*2),sin(seq_num*3));
      imuPkt.setangular_velocity(gyro);

      accel.set(cos(seq_num*1./3.),cos(seq_num*2./3.),cos(seq_num));
      imuPkt.setlinear_acceleration(accel);

      this->m_seq_num++;

      return imuPkt;
  }

  void STIM300Model::run()
  {
    U8 stimPktBuffer[256];
    U32 pktBufferLen;
    ROS::sensor_msgs::ImuNoCov imuPkt;
    ROS::std_msgs::Header header;
    Fw::Time eventTime;
    int idx;

    int period_us = (1000 * 1000 / m_rateHz);

    while (!m_should_stop) {

        imuPkt = getImuPkt();

        createStimPkt(imuPkt, eventTime, stimPktBuffer, pktBufferLen);

        this->m_pushEventFun(eventTime);

        for (idx = 0; idx < (pktBufferLen-1); idx++) {
            this->m_pushBytesFun(&stimPktBuffer[idx], 1);
        }

        // Need to push the last byte along with the expected
        // packet to workaround a race condition
        this->m_pushExpFun(&stimPktBuffer[idx], 1, imuPkt);

        std::this_thread::sleep_for(std::chrono::microseconds(period_us));
    }
  }

  void STIM300Model::
      generatePkts(ROS::sensor_msgs::ImuNoCov& imuPkt,
                   Fw::Time& eventTime,
                   U8* stimData, U32& stimDataLen) {

    imuPkt = getImuPkt();

    createStimPkt(imuPkt, eventTime, stimData, stimDataLen);

  }

  void STIM300Model::stop() {

    this->m_should_stop = true;

  }

}
