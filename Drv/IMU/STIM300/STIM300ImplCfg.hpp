// ====================================================================== 
// \title  STIM300ImplCfg.hpp
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

#ifndef STIM300_IMPL_CFG_HPP
#define STIM300_IMPL_CFG_HPP

#include <Fw/Types/BasicTypes.hpp>

#include <math.h>

namespace Drv {

    const U32 STIM_MAX_EVENTS = 40;

#ifdef BUILD_UT
    const U32 STIM_TS_CHECK_CYCLES = 20;
#else
    const U32 STIM_TS_CHECK_CYCLES = 200; // 1 s at 500Hz cycles
#endif

    const U32 STIM_PKT_BUFFER_SIZE = 1024;
    const U32 STIM_UART_BUFFER_SIZE = 1024;

    const U32 STIM_LATENCY_DN_TO_EU_US = 1; // TODO:
    const F64 STIM_GYRO_DN_TO_EU_RAD_S_DIV = 16384 / 180.0 * M_PI; // 2^14 * pi / 180
    const F64 STIM_ACCEL_DN_TO_EU_M_S_DIV = (524288 / 9.81); //(2^19)/g
}

#endif
