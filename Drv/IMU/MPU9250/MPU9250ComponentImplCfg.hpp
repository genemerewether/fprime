#ifndef Drv_IMU_MPU9250Cfg_HPP
#define Drv_IMU_MPU9250Cfg_HPP

namespace Drv {

enum {
    MPU9250_SCHED_CONTEXT_OPERATE,
    MPU9250_SCHED_CONTEXT_TLM
};

enum {
    MPU9250_RESET_WAIT_CYCLES = 1000,
};

} // namespace Drv

#endif // Drv_IMU_MPU9250Cfg_HPP
