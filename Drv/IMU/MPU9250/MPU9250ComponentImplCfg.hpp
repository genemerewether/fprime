#ifndef Drv_IMU_MPU9250Cfg_HPP
#define Drv_IMU_MPU9250Cfg_HPP

namespace Drv {

enum {
    MPU9250_SCHED_CONTEXT_OPERATE,
    MPU9250_SCHED_CONTEXT_TLM
};

enum {
    MPU9250_RESET_WAIT_CYCLES = 1000, // TODO(mereweth) - reduce
    MPU9250_SPI_CONFIG_HZ = 1000 * 1000, // 1 MHz for config registers
    MPU9250_SPI_FIFO_HZ = 20 * 1000 * 1000, // 20 MHz for config registers
};

} // namespace Drv

#endif // Drv_IMU_MPU9250Cfg_HPP
