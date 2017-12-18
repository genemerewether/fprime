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
  enum {
    // TODO(mereweth) - must be manually synced with max port size
    KR_RECV_PORT_BUFF_SIZE = 256,
    KR_NUM_RECV_PORT_BUFFS = 5,
    KR_PREINIT_SLEEP_US = 1000,
    KR_NOPORT_SLEEP_US = 1000
  };
};

#endif //ifndef KraitRouterCfg_HPP
