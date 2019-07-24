// ====================================================================== 
// \title  STIM300Pkt.hpp
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

#ifndef STIM300_PKT_HPP
#define STIM300_PKT_HPP

#include <Fw/Types/BasicTypes.hpp>

namespace Drv {

    inline static F64 imuU24ToF64(U8* dataPtr, F64 divDnEu) {

        U32 temp = (dataPtr[0] << 16) | (dataPtr[1] << 8) | (dataPtr[2]);

        if (temp & 0x800000) {
            // Two's Complement
            temp = ((~temp) + 1) & 0xFFFFFF;

            return -1 * (temp / divDnEu);
        } else {
            return temp / divDnEu;
        }
    }

    // This assumes accel and gyro packets are available.
    // Will not compile for packet types without accel data
    template <typename T>
    void toImuPktGeneric(T& stimPkt,
                         Fw::Time& eventTime,
                         ROS::sensor_msgs::ImuNoCov& imuPkt) {

        F64 gyro_x, gyro_y, gyro_z;
        F64 accel_x, accel_y, accel_z;
        I32 latency;

        latency = stimPkt.latency[0] << 8 | stimPkt.latency[1];

        gyro_x = imuU24ToF64(&stimPkt.gyro_x[0], STIM_GYRO_DN_TO_EU_RAD_S_DIV);
        gyro_y = imuU24ToF64(&stimPkt.gyro_y[0], STIM_GYRO_DN_TO_EU_RAD_S_DIV);
        gyro_z = imuU24ToF64(&stimPkt.gyro_z[0], STIM_GYRO_DN_TO_EU_RAD_S_DIV);

        accel_x = imuU24ToF64(&stimPkt.accel_x[0], STIM_ACCEL_DN_TO_EU_M_S_DIV);
        accel_y = imuU24ToF64(&stimPkt.accel_y[0], STIM_ACCEL_DN_TO_EU_M_S_DIV);
        accel_z = imuU24ToF64(&stimPkt.accel_z[0], STIM_ACCEL_DN_TO_EU_M_S_DIV);

        Fw::Time latencyTime(eventTime.getTimeBase(), 0, latency);
        Fw::Time compTime = Fw::Time::sub(eventTime, latencyTime);

        ROS::std_msgs::Header header(stimPkt.counter, compTime, 0);

        ROS::geometry_msgs::Quaternion orientation(0, 0, 0, 1);
        ROS::geometry_msgs::Vector3 gyro(gyro_x, gyro_y, gyro_z);

        ROS::geometry_msgs::Vector3 accel(accel_x, accel_y, accel_z);

        imuPkt.set(header, orientation, gyro, accel);
    }

    struct STIM300Packet_90 {
        U8 ident;
        U8 gyro_x[3];
        U8 gyro_y[3];
        U8 gyro_z[3];
        U8 gyro_status;
        U8 counter;
        U8 latency[2];
        U32 crc;

    } __attribute__((packed));

    struct STIM300Packet_93 {
        U8 ident;
        U8 gyro_x[3];
        U8 gyro_y[3];
        U8 gyro_z[3];
        U8 gyro_status;
        U8 accel_x[3];
        U8 accel_y[3];
        U8 accel_z[3];
        U8 accel_status;
        U8 incl_x[3];
        U8 incl_y[3];
        U8 incl_z[3];
        U8 incl_status;
        U8 counter;
        U8 latency[2];
        U8 crc[4];

    }__attribute__((packed));
}

#endif
