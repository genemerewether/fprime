// ======================================================================
// \title  MPU9250Impl.cpp
// \author mereweth
// \brief  cpp file for MPU9250 component implementation class
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


#include <Drv/IMU/MPU9250/MPU9250ComponentImpl.hpp>
#include <Drv/IMU/MPU9250/MPU9250Reg.hpp>
#include "Fw/Types/BasicTypes.hpp"

#include "Drv/LinuxSpiDriver/LinuxSpiDriverComponentImplCfg.hpp"

#ifdef BUILD_DSPAL
#include <HAP_farf.h>
#define DEBUG_PRINT(x,...) FARF(ALWAYS,x,##__VA_ARGS__);
#else
#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#endif

//#undef DEBUG_PRINT
//#define DEBUG_PRINT(x,...)

namespace Drv {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

    MPU9250ComponentImpl ::
      MPU9250ComponentImpl(
    #if FW_OBJECT_NAMES == 1
          const char *const compName,
    #endif
          bool useMagnetometer
      ) :
        MPU9250ComponentBase(
    #if FW_OBJECT_NAMES == 1
                             compName
    #endif
                             ),
        m_initState(INIT_RESET),
        m_useMagnetometer(useMagnetometer),
        m_cycleCount(0u)
    {

    }

    void MPU9250ComponentImpl ::
      init(
          const NATIVE_INT_TYPE instance
      )
    {
      MPU9250ComponentBase::init(instance);
    }

    MPU9250ComponentImpl ::
      ~MPU9250ComponentImpl(void)
    {

    }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

    void MPU9250ComponentImpl ::
      sched_handler(
          const NATIVE_INT_TYPE portNum,
          NATIVE_UINT_TYPE context
      )
    {
        if (context == MPU9250_SCHED_CONTEXT_OPERATE) {
            BYTE writeBuf[2] = {0};
            Fw::Buffer writeBufObj(0, 0, (U64) writeBuf, 2);
            Fw::Buffer dummyReadBufObj(0, 0, 0, 0);
            BYTE readBuf[MPU9250_FIFO_LEN + 1] = {0}; // biggest read
            Fw::Buffer readBufObj(0, 0, (U64) readBuf, 2); // most reads are a single byte

            // outside of switch statement for clarity
            if (m_initState == INIT_COMPLETE) {
                // TODO(mereweth) - check that value is read into 2nd byte
                this->SpiConfig_out(0, MPU9250_SPI_CONFIG_HZ);
                writeBuf[0] = MPU9250_REG_INT_STATUS | SPI_BITS_READ;
                writeBuf[1] = 0; // TODO(mereweth) - needed?
                this->SpiReadWrite_out(0, writeBufObj, readBufObj);
                if (readBuf[1] & MPU9250_BITS_INT_STATUS_FIFO_OVERFLOW) {
                    DEBUG_PRINT("FIFO overflow\n");
                    m_initState = INIT_FIFO_RESET;
                    return;
                }
                // TODO(mereweth) - check that value is read starting at 2nd byte
                writeBuf[0] = MPU9250_REG_FIFO_COUNTH | SPI_BITS_READ;
                writeBuf[1] = 0; // TODO(mereweth) - needed?
                readBufObj.setsize(3);
                this->SpiReadWrite_out(0, writeBufObj, readBufObj);
                readBufObj.setsize(2); // reset for everyone else
                // TODO(mereweth) - add endianness check to Fw
                uint16_t fifoLen = ((uint16_t) readBuf[1]) << 8 | (uint16_t) readBuf[2];
                DEBUG_PRINT("FIFO count %u; low 0x%x, high 0x%x\n",
                            fifoLen, readBuf[1], readBuf[2]);

                this->SpiConfig_out(0, MPU9250_SPI_FIFO_HZ);
                writeBuf[0] = MPU9250_REG_FIFO_R_W | SPI_BITS_READ;
                writeBuf[1] = 0; // TODO(mereweth) - needed?
                readBufObj.setsize(fifoLen);
                this->SpiReadWrite_out(0, writeBufObj, readBufObj);
                // TODO(mereweth) - parse data

                return;
            }

            // handle init/error
            switch(m_initState) {
                case INIT_RESET:
                    if (!m_cycleCount) { // first time in state
                        DEBUG_PRINT("MPU9250 enter INIT_RESET\n");
                        this->SpiConfig_out(0, MPU9250_SPI_CONFIG_HZ);
                        writeBuf[0] = MPU9250_REG_PWR_MGMT_1 | SPI_BITS_WRITE;
                        writeBuf[1] = MPU9250_BITS_H_RESET;
                        this->SpiReadWrite_out(0, writeBufObj, dummyReadBufObj);
                    }
                    else {
                        if (m_cycleCount > MPU9250_RESET_WAIT_CYCLES) {
                            m_cycleCount = 0;
                            m_initState = INIT_POWER_ON_1;
                            return;
                        }
                    }
                    m_cycleCount++;
                    break;
                case INIT_POWER_ON_1:
                    DEBUG_PRINT("MPU9250 enter INIT_POWER_ON_1\n");
                    this->SpiConfig_out(0, MPU9250_SPI_CONFIG_HZ);
                    writeBuf[0] = MPU9250_REG_PWR_MGMT_1 | SPI_BITS_WRITE;
                    writeBuf[1] = 0;
                    this->SpiReadWrite_out(0, writeBufObj, dummyReadBufObj);
                    m_initState = INIT_POWER_ON_2;
                    break;
                case INIT_POWER_ON_2:
                    DEBUG_PRINT("MPU9250 enter INIT_POWER_ON_2\n");
                    this->SpiConfig_out(0, MPU9250_SPI_CONFIG_HZ);
                    writeBuf[0] = MPU9250_REG_PWR_MGMT_2 | SPI_BITS_WRITE;
                    writeBuf[1] = 0;
                    this->SpiReadWrite_out(0, writeBufObj, dummyReadBufObj);
                    m_initState = INIT_I2C_RESET;
                    break;
                case INIT_I2C_RESET:
                    DEBUG_PRINT("MPU9250 enter INIT_I2C_RESET\n");
                    this->SpiConfig_out(0, MPU9250_SPI_CONFIG_HZ);
                    writeBuf[0] = MPU9250_REG_USER_CTRL | SPI_BITS_WRITE;
                    writeBuf[1] = MPU9250_BITS_USER_CTRL_I2C_MST_RST |
                                  MPU9250_BITS_USER_CTRL_I2C_IF_DIS;
                    this->SpiReadWrite_out(0, writeBufObj, dummyReadBufObj);
                    m_initState = INIT_FIFO_CONFIG;
                    break;
                // case INIT_FIFO_RESET:
                //     DEBUG_PRINT("MPU9250 enter INIT_FIFO_RESET\n");
                //     // this->set SPI clock to 1 MHZ
                //     writeBuf[0] = MPU9250_REG_USER_CTRL | SPI_BITS_WRITE;
                //     writeBuf[1] = MPU9250_BITS_USER_CTRL_FIFO_RST |
                //                   MPU9250_BITS_USER_CTRL_FIFO_EN;
                //     this->SpiReadWrite_out(0, writeBufObj, dummyReadBufObj);
                //     m_initState = INIT_FIFO_CONFIG;
                //     break;
                case INIT_FIFO_CONFIG:
                    DEBUG_PRINT("MPU9250 enter INIT_FIFO_CONFIG\n");
                    this->SpiConfig_out(0, MPU9250_SPI_CONFIG_HZ);
                    writeBuf[0] = MPU9250_REG_FIFO_EN | SPI_BITS_WRITE;
                    writeBuf[1] = MPU9250_BITS_FIFO_ENABLE_TEMP_OUT  |
                                  MPU9250_BITS_FIFO_ENABLE_GYRO_XOUT |
                                  MPU9250_BITS_FIFO_ENABLE_GYRO_YOUT |
                                  MPU9250_BITS_FIFO_ENABLE_GYRO_ZOUT |
                                  MPU9250_BITS_FIFO_ENABLE_ACCEL;
                    if (m_useMagnetometer) {
                        DEBUG_PRINT("MPU9250 use mag in FIFO\n");
                         // magnetometer data over I2C
                        writeBuf[1] |= MPU9250_BITS_FIFO_ENABLE_SLV0;
                    }
                    this->SpiReadWrite_out(0, writeBufObj, dummyReadBufObj);
                    m_initState = INIT_GEN_CONFIG; // TODO next state
                    break;
                case INIT_GEN_CONFIG:
                    DEBUG_PRINT("MPU9250 enter INIT_GEN_CONFIG\n");
                    this->SpiConfig_out(0, MPU9250_SPI_CONFIG_HZ);
                    writeBuf[0] = MPU9250_REG_CONFIG | SPI_BITS_WRITE;
                    // temperature & gyro DLPF, FIFO mode
                    writeBuf[1] = MPU9250_BITS_DLPF_CFG_250HZ |
                                  MPU9250_BITS_CONFIG_FIFO_MODE_OVERWRITE;
                    this->SpiReadWrite_out(0, writeBufObj, dummyReadBufObj);
                    m_initState = INIT_GYRO_CONFIG; // TODO next state
                    break;
                case INIT_GYRO_CONFIG:
                    DEBUG_PRINT("MPU9250 enter INIT_GYRO_CONFIG\n");
                    this->SpiConfig_out(0, MPU9250_SPI_CONFIG_HZ);
                    writeBuf[0] = MPU9250_REG_GYRO_CONFIG | SPI_BITS_WRITE;
                    // TODO(mereweth) - test gyro at MPU9250_BITS_BW_8800HZ - no DLPF control
                    writeBuf[1] = MPU9250_BITS_FS_2000DPS |
                                  MPU9250_BITS_BW_LTE3600HZ;
                    this->SpiReadWrite_out(0, writeBufObj, dummyReadBufObj);
                    m_initState = INIT_ACCEL_CONFIG_1; // TODO next state
                    break;
                case INIT_ACCEL_CONFIG_1:
                    DEBUG_PRINT("MPU9250 enter INIT_ACCEL_CONFIG_1\n");
                    this->SpiConfig_out(0, MPU9250_SPI_CONFIG_HZ);
                    writeBuf[0] = MPU9250_REG_ACCEL_CONFIG | SPI_BITS_WRITE;
                    writeBuf[1] = MPU9250_BITS_ACCEL_CONFIG_16G;
                    this->SpiReadWrite_out(0, writeBufObj, dummyReadBufObj);
                    m_initState = INIT_ACCEL_CONFIG_2; // TODO next state
                    break;
                case INIT_ACCEL_CONFIG_2:
                    DEBUG_PRINT("MPU9250 enter INIT_ACCEL_CONFIG_2\n");
                    this->SpiConfig_out(0, MPU9250_SPI_CONFIG_HZ);
                    writeBuf[0] = MPU9250_REG_ACCEL_CONFIG2 | SPI_BITS_WRITE;
                    // TODO(mereweth) - better performance with DLPF?
                    writeBuf[1] = MPU9250_BITS_ACCEL_CONFIG2_BW_1130HZ;
                    this->SpiReadWrite_out(0, writeBufObj, dummyReadBufObj);
                    m_initState = INIT_MAG_CONFIG; // TODO next state
                    break;
                case INIT_MAG_CONFIG:
                    DEBUG_PRINT("MPU9250 enter INIT_MAG_CONFIG\n");
                    if (m_useMagnetometer) {
                        DEBUG_PRINT("MPU9250 config mag\n");
                        this->SpiConfig_out(0, MPU9250_SPI_CONFIG_HZ);
                        this->SpiReadWrite_out(0, writeBufObj, dummyReadBufObj);
                    }
                    else {
                        m_initState = INIT_FIFO_RESET;
                        break;
                    }
                    break;
                case INIT_FIFO_RESET:
                    DEBUG_PRINT("MPU9250 enter INIT_FIFO_RESET\n");
                    this->SpiConfig_out(0, MPU9250_SPI_CONFIG_HZ);
                    writeBuf[0] = MPU9250_REG_USER_CTRL | SPI_BITS_READ;
                    writeBuf[1] = 0; // TODO(mereweth) - needed?
                    this->SpiReadWrite_out(0, writeBufObj, readBufObj);
                    // TODO(mereweth) - check that value is read into 2nd byte

                    writeBuf[0] = MPU9250_REG_USER_CTRL | SPI_BITS_WRITE;
                    writeBuf[1] = readBuf[1]                      |
                                  MPU9250_BITS_USER_CTRL_FIFO_RST |
                                  MPU9250_BITS_USER_CTRL_FIFO_EN;
                    this->SpiReadWrite_out(0, writeBufObj, dummyReadBufObj);

                    m_initState = INIT_COMPLETE; // TODO next state
                    break;
                case INIT_ERROR:
                    m_initState = INIT_RESET; // TODO(mereweth) - smarter recovery?
                    break;
                default:
                    FW_ASSERT(0, m_initState);
            }
        }
        else if (context == MPU9250_SCHED_CONTEXT_TLM) {

        }
    }

    void MPU9250ComponentImpl ::
      pingIn_handler(
          const NATIVE_INT_TYPE portNum,
          U32 key
      )
    {
        // TODO
    }

} // end namespace Drv
