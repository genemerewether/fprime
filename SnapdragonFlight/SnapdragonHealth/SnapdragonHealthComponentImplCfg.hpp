/**
 * \file   SnapdragonHealthComponentImplCfg.hpp
 * \author Gene Merewether
 * \brief  Component responsible for monitoring Snapdragon health
 *
 * \copyright
 * Copyright 2009-2016, by the California Institute of Technology.
 * ALL RIGHTS RESERVED.  United States Government Sponsorship
 * acknowledged. Any commercial use must be negotiated with the Office
 * of Technology Transfer at the California Institute of Technology.
 * <br /><br />
 * This software may be subject to U.S. export control laws and
 * regulations.  By accepting this document, the user agrees to comply
 * with all U.S. export laws and regulations.  User has the
 * responsibility to obtain export licenses, or other export authority
 * as may be required before exporting such information to foreign
 * countries or providing access to foreign persons.
 */


#ifndef SNAPDRAGONHEALTHCOMPONENTIMPLCFG_HPP_
#define SNAPDRAGONHEALTHCOMPONENTIMPLCFG_HPP_

namespace SnapdragonFlight {

  enum {
    SH_SCHED_COUNTDOWN = 1
  };

  enum SH_PowerSaverMode {
    SH_SAVER_ON,
    SH_SAVER_DYNAMIC,
    SH_SAVER_OFF,
    SH_SAVER_RAPID,
  };  
}

#endif // SNAPDRAGONHEALTHCOMPONENTIMPLCFG_HPP_
