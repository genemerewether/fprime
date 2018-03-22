#ifndef RotorSDrvCfg_HPP
#define RotorSDrvCfg_HPP

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32.h"

#include "Os/Mutex.hpp"

#include "SIMREF/RotorSDrv/RotorSDrvComponentAc.hpp"
#include "SIMREF/RotorSDrv/RotorSDrvComponentImplCfg.hpp"

namespace SIMREF {

enum {
    RSDRV_SCHED_CONTEXT_TLM,
    RSDRV_SCHED_CONTEXT_POS,
    RSDRV_SCHED_CONTEXT_ATT,
};

} // namespace SIMREF

#endif // RotorSDrvCfg_HPP
