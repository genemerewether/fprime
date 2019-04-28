// ======================================================================
// \title  ActuatorAdapterComponentImplCfg.hpp
// \author mereweth
// \brief  hpp defines file for ActuatorAdapter component implementation class
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

#ifndef ActuatorAdapterCfg_HPP
#define ActuatorAdapterCfg_HPP

namespace Gnc {

enum {
    ACTADAP_MAX_ACTUATORS = 8,
    ACTADAP_ARM_COUNT = 35
};

enum {
    ACTADAP_SCHED_CONTEXT_TLM,
    ACTADAP_SCHED_CONTEXT_ARM
};
  
}

#endif //ActuatorAdapterCfg_HPP
