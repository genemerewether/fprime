// ======================================================================
// \title  KraitRouterImpl.hpp
// \author vagrant
// \brief  hpp file for KraitRouter component implementation class
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

#ifndef KraitRouterCfg_HPP
#define KraitRouterCfg_HPP

namespace SnapdragonFlight {

// TODO(mereweth) - add error status codes

enum {
    KR_PREINIT_SLEEP_US = 1000,
    KR_SCHED_MAX_ITER = 10 // max of 10 port calls out per sched call
};

enum KraitRouterReturnCode {
    KR_RTN_QUIT_PREINIT = -10,
    KR_RTN_SEND_TOO_BIG = -3,
    KR_RTN_QUIT = -2,

    //NOTE(mereweth) - don't return this value - just a placeholder for when FastRPC returns it
    KR_RTN_FASTRPC_FAIL = -1,

    KR_RTN_OK = 0
};

}; // namespace SnapdragonFlight

#endif //ifndef KraitRouterCfg_HPP
