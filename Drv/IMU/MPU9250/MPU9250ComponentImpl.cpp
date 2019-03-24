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

#include "Os/IntervalTimer.hpp"

#include "Drv/LinuxSpiDriver/LinuxSpiDriverComponentImplCfg.hpp"

#ifdef BUILD_DSPAL
#include <HAP_farf.h>
#define DEBUG_PRINT(x,...) FARF(ALWAYS,x,##__VA_ARGS__);
#else // BUILD_DSPAL
#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#endif // BUILD_DSPAL

#if defined BUILD_SDFLIGHT || defined BUILD_DSPAL || defined BUILD_TIR5
#define BIG_ENDIAN
#else // defined BUILD_SDFLIGHT || defined BUILD_DSPAL
#define LITTLE_ENDIAN
#endif // defined BUILD_SDFLIGHT || defined BUILD_DSPAL

#undef DEBUG_PRINT
#define DEBUG_PRINT(x,...)

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
        m_outMode(OUTPUT_1KHZ_DLPF_ACCEL_460HZ_GYRO_184HZ),
        m_opMode(OPMODE_INTERRUPT),
        m_gyroRawToRadS(0.0f),
        m_accelRawToMS2(0.0f),
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

    bool MPU9250ComponentImpl ::
      isReady()
    {
        return (INIT_COMPLETE == m_initState) ? true : false;
    }

    void MPU9250ComponentImpl ::
      setOutputMode(OutputMode mode) {
        m_outMode = mode;
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
        // outside of switch statement for clarity
        if (context == MPU9250_SCHED_CONTEXT_OPERATE) {
            BYTE writeBuf[MPU9250_FIFO_LEN + 1] = {0};
            Fw::Buffer writeBufObj(0, 0, (U64) writeBuf, 2);
            Fw::Buffer dummyReadBufObj(0, 0, 0, 0);
            BYTE readBuf[MPU9250_FIFO_LEN + 1] = {0}; // biggest read
            BYTE* readBufOffset = readBuf;
            Fw::Buffer readBufObj(0, 0, (U64) readBuf, 2); // most reads are a single byte

            if (m_initState == INIT_COMPLETE) {
                Os::IntervalTimer timer;
                if (m_opMode == OPMODE_INTERRUPT) {
                    // TODO(mereweth) - add port call with exact interrupt time
                    Fw::Time ImuNow = this->getTime();
                    timer.start();
                    // this is done when leaving last INIT phase
                    //this->SpiConfig_out(0, MPU9250_SPI_DATA_HZ);
                    writeBuf[0] = MPU9250_REG_ACCEL_XOUT_H | SPI_BITS_READ;
#ifdef BUILD_DSPAL || BUILD_LINUX
                    readBufObj.setsize(2 + MPU9250_REG_GYRO_ZOUT_L
                                       - MPU9250_REG_ACCEL_XOUT_H);
#else // most microcontrollers
                    writeBufObj.setsize(5 + MPU9250_REG_GYRO_ZOUT_L
                                       - MPU9250_REG_ACCEL_XOUT_H);
                    readBufObj.setsize(5 + MPU9250_REG_GYRO_ZOUT_L
                                       - MPU9250_REG_ACCEL_XOUT_H);

                    // Offset returned data by one half-word
                    readBufOffset = readBuf + 2;
#endif
                    //DEBUG_PRINT("MPU9250 before read\n");
                    this->SpiReadWrite_out(0, writeBufObj, readBufObj);
                    timer.stop();
                    DEBUG_PRINT("reg read %d bytes in %u usec\n",
                                readBufObj.getsize(), timer.getDiffUsec());

                    // raw register contents
                    /*if (this->isConnected_FIFORaw_OutputPort(0)) {
                        Fw::ExternalSerializeBuffer portBuff(readBuf,
                                                             readBufObj.getsize());
                        portBuff.setBuffLen(readBufObj.getsize());

                        Fw::SerializeStatus stat = this->FIFORaw_out(0, portBuff);
                        if (stat != Fw::FW_SERIALIZE_OK) {
                            DEBUG_PRINT("FIFORaw_out() serialize status error\n");
                        }
                    }*/

                    int16_t accel[3];
                    int16_t gyro[3];
                    int16_t temp;
                    NATIVE_UINT_TYPE sIdx;
                    NATIVE_UINT_TYPE sBase = 1;
                    // read starts at index 1 in read buffer
#ifdef LITTLE_ENDIAN // little-endian - x86
                    for (sIdx = 0; sIdx < 3; sIdx++) {
                        accel[sIdx] = ((int16_t) readBufOffset[2*sIdx+1+sBase]) << 8 |
                                        (int16_t) readBufOffset[2*sIdx+sBase];
                    }
                    sBase += 6;
                    temp = ((int16_t) readBufOffset[1+sBase]) << 8 |
                           (int16_t) readBufOffset[sBase];
                    sBase += 2;
                    for (sIdx = 0; sIdx < 3; sIdx++) {
                        gyro[sIdx] = ((int16_t) readBufOffset[2*sIdx+1+sBase]) << 8 |
                                        (int16_t) readBufOffset[2*sIdx+sBase];
                    }
#else // big endian - ARM & Hexagon
                    for (sIdx = 0; sIdx < 3; sIdx++) {
                        accel[sIdx] = ((int16_t) readBufOffset[2*sIdx+sBase]) << 8 |
                                        (int16_t) readBufOffset[1+2*sIdx+sBase];
                    }
                    sBase += 6;
                    temp = ((int16_t) readBufOffset[sBase]) << 8 |
                           (int16_t) readBufOffset[1+sBase];
                    sBase += 2;
                    for (sIdx = 0; sIdx < 3; sIdx++) {
                        gyro[sIdx] = ((int16_t) readBufOffset[2*sIdx+sBase]) << 8 |
                                        (int16_t) readBufOffset[1+2*sIdx+sBase];
                    }
#endif

                    F64 accelX = m_accelRawToMS2 * (float) accel[0];
                    F64 accelY = m_accelRawToMS2 * (float) accel[1];
                    F64 accelZ = m_accelRawToMS2 * (float) accel[2];

                    F64 gyroX = m_gyroRawToRadS * (float) gyro[0];
                    F64 gyroY = m_gyroRawToRadS * (float) gyro[1];
                    F64 gyroZ = m_gyroRawToRadS * (float) gyro[2];

                    // TODO(mereweth) - convert temperature; check for saturation
                    DEBUG_PRINT("Accel [ % 2.3f, % 2.3f, % 2.3f]; Gyro [% 2.3f, % 2.3f, % 2.3f]; Temp % 2.3f\n",
                                accelX, accelY, accelZ, gyroX, gyroY, gyroZ,
                                float(temp) / 361.0f + 35.0f);

                    {
                        using namespace ROS::std_msgs;
                        using namespace ROS::geometry_msgs;
                        using namespace ROS::sensor_msgs;

                        ImuNoCov imu(
                          // TODO(mereweth) - add/use time port from GPIO interrupt
                          // TODO(mereweth) - convert frame name to U32 idx
                          Header(m_cycleCount, ImuNow, 0/*Fw::EightyCharString("mpu9250")*/),
                          Quaternion(0, 0, 0, 1), // TODO(mereweth) - mag goes here
                          Vector3(gyroX, gyroY, gyroZ),
                          Vector3(accelX, accelY, accelZ)
                        ); // end Imu constructor
                        for (int i = 0; i < NUM_IMU_OUTPUT_PORTS; i++) {
                            if (this->isConnected_Imu_OutputPort(i)) {
                                this->Imu_out(i, imu);
                            }
                            else {
                                DEBUG_PRINT("MPU9250 Imu out port %d not connected\n", i);
                            }
                        }
                    }
                }
                else if (m_opMode == OPMODE_FIFO) {
                    timer.start();

                    // TODO(mereweth) - check that value is read into 2nd byte
                    this->SpiConfig_out(0, MPU9250_SPI_CONFIG_HZ);
                    writeBuf[0] = MPU9250_REG_INT_STATUS | SPI_BITS_READ;
                    writeBuf[1] = 0; // TODO(mereweth) - needed?
                    this->SpiReadWrite_out(0, writeBufObj, readBufObj);
                    if (readBuf[1] & MPU9250_BITS_INT_STATUS_FIFO_OVERFLOW) {
                        DEBUG_PRINT("FIFO overflow\n");
                        // TODO(mereweth) - only reset if in overwrite FIFO mode?
                        m_initState = INIT_FIFO_RESET;
                        return;
                    }
                    // TODO(mereweth) - check that value is read starting at 2nd byte
                    writeBuf[0] = MPU9250_REG_FIFO_COUNTH | SPI_BITS_READ;
                    writeBuf[1] = 0; // TODO(mereweth) - needed?
                    readBufObj.setsize(3);
                    this->SpiReadWrite_out(0, writeBufObj, readBufObj);
                    readBufObj.setsize(2); // reset for everyone else
                    // TODO(mereweth) - add endianness check to Fw - FIFO_COUNT_H first
#ifdef LITTLE_ENDIAN // little-endian - x86
                    uint16_t fifoLen = ((uint16_t) readBuf[2]) << 8 | (uint16_t) readBuf[1];
#else // big endian - ARM & Hexagon
                    uint16_t fifoLen = ((uint16_t) readBuf[1]) << 8 | (uint16_t) readBuf[2];
#endif
                    // get config execution time
                    timer.stop();
                    DEBUG_PRINT("FIFO count %u; low 0x%x, high 0x%x; calc in %u usec\n",
                                fifoLen, readBuf[1], readBuf[2], timer.getDiffUsec());

                    timer.start();
                    this->SpiConfig_out(0, MPU9250_SPI_DATA_HZ);
                    writeBuf[0] = MPU9250_REG_FIFO_R_W | SPI_BITS_READ;
                    writeBuf[1] = 0; // TODO(mereweth) - needed?
                    // TODO(mereweth) - DSP can read max of 512 bytes at once - check for full FIFO -> 513 bytes
                    readBufObj.setsize(fifoLen);
                    this->SpiReadWrite_out(0, writeBufObj, readBufObj);
                    // TODO(mereweth) - parse data
                    timer.stop();

                    DEBUG_PRINT("FIFO read in %u usec\n", timer.getDiffUsec());
                    if (this->isConnected_FIFORaw_OutputPort(0)) {
                        // read data starts at index 1
                        Fw::ExternalSerializeBuffer portBuff(readBuf + 1, fifoLen);
                        portBuff.setBuffLen(fifoLen);

                        Fw::SerializeStatus stat = this->FIFORaw_out(0, portBuff);
                        if (stat != Fw::FW_SERIALIZE_OK) {
                            DEBUG_PRINT("FIFORaw_out() serialize status error\n");
                        }
                    }
                }

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
                        DEBUG_PRINT("MPU9250 done in INIT_RESET\n");
                    }
                    else {
                        //DEBUG_PRINT("MPU9250 INIT_RESET cycle %d\n", m_cycleCount);
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
                case INIT_FIFO_CONFIG:
                    DEBUG_PRINT("MPU9250 enter INIT_FIFO_CONFIG\n");
                    if (m_opMode == OPMODE_FIFO) {
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
                    }
                    m_initState = INIT_GEN_CONFIG;
                    break;
                case INIT_GEN_CONFIG:
                    DEBUG_PRINT("MPU9250 enter INIT_GEN_CONFIG\n");
                    this->SpiConfig_out(0, MPU9250_SPI_CONFIG_HZ);
                    writeBuf[0] = MPU9250_REG_CONFIG | SPI_BITS_WRITE;
                    // temperature & gyro DLPF, FIFO mode
                    if (OUTPUT_1KHZ_DLPF_ACCEL_460HZ_GYRO_184HZ == m_outMode) {
                        // 184Hz low-pass gives data output at 1 kHz
                        writeBuf[1] = MPU9250_BITS_GYRO_DLPF_CFG_184HZ | //MPU9250_BITS_GYRO_DLPF_CFG_250HZ |
                                      MPU9250_BITS_CONFIG_FIFO_MODE_STOP; // MPU9250_BITS_CONFIG_FIFO_MODE_OVERWRITE;
                    }
                    if (OUTPUT_ACCEL_4KHZ_GYRO_8KHZ_DLPF_GYRO_3600KHZ == m_outMode) {
                        writeBuf[1] = MPU9250_BITS_GYRO_DLPF_CFG_3600HZ | //MPU9250_BITS_GYRO_DLPF_CFG_250HZ |
                                      MPU9250_BITS_CONFIG_FIFO_MODE_STOP; // MPU9250_BITS_CONFIG_FIFO_MODE_OVERWRITE;
                    }
                    this->SpiReadWrite_out(0, writeBufObj, dummyReadBufObj);
                    m_initState = INIT_GYRO_CONFIG;
                    break;
                case INIT_GYRO_CONFIG:
                    DEBUG_PRINT("MPU9250 enter INIT_GYRO_CONFIG\n");
                    this->SpiConfig_out(0, MPU9250_SPI_CONFIG_HZ);
                    writeBuf[0] = MPU9250_REG_GYRO_CONFIG | SPI_BITS_WRITE;
                    // TODO(mereweth) - test gyro at MPU9250_BITS_BW_8800HZ - no DLPF control
                    m_gyroRawToRadS = 2000.0f / 32768.0f * MPU9250_PI / 180.0f;
                    writeBuf[1] = MPU9250_BITS_FS_2000DPS |
                                  //MPU9250_BITS_GYRO_BW_8800HZ;
                                  MPU9250_BITS_GYRO_BW_LTE3600HZ;
                    this->SpiReadWrite_out(0, writeBufObj, dummyReadBufObj);
                    m_initState = INIT_ACCEL_CONFIG_1;
                    break;
                case INIT_ACCEL_CONFIG_1:
                    DEBUG_PRINT("MPU9250 enter INIT_ACCEL_CONFIG_1\n");
                    this->SpiConfig_out(0, MPU9250_SPI_CONFIG_HZ);
                    writeBuf[0] = MPU9250_REG_ACCEL_CONFIG | SPI_BITS_WRITE;
                    m_accelRawToMS2 = MPU9250_ONE_G / 2048.0f;
                    writeBuf[1] = MPU9250_BITS_ACCEL_CONFIG_16G;
                    this->SpiReadWrite_out(0, writeBufObj, dummyReadBufObj);
                    m_initState = INIT_ACCEL_CONFIG_2;
                    break;
                case INIT_ACCEL_CONFIG_2:
                    DEBUG_PRINT("MPU9250 enter INIT_ACCEL_CONFIG_2\n");
                    this->SpiConfig_out(0, MPU9250_SPI_CONFIG_HZ);
                    writeBuf[0] = MPU9250_REG_ACCEL_CONFIG2 | SPI_BITS_WRITE;

                    if (OUTPUT_1KHZ_DLPF_ACCEL_460HZ_GYRO_184HZ == m_outMode) {
                        // TODO(mereweth) - better performance with DLPF?
                        // 460Hz low-pass gives data output at 1 kHz
                        writeBuf[1] = MPU9250_BITS_ACCEL_BW_LTE_460HZ |
                                      MPU9250_BITS_ACCEL_DLPF_CFG_460HZ;
                    }
                    if (OUTPUT_ACCEL_4KHZ_GYRO_8KHZ_DLPF_GYRO_3600KHZ == m_outMode) {
                        // NOTE(mereweth) - dlpf cfg ignored with this bandwidth
                        writeBuf[1] = MPU9250_BITS_ACCEL_BW_1130HZ;
                    }
                    this->SpiReadWrite_out(0, writeBufObj, dummyReadBufObj);
                    m_initState = INIT_MAG_CONFIG;
                    break;
                case INIT_MAG_CONFIG:
                    DEBUG_PRINT("MPU9250 enter INIT_MAG_CONFIG\n");
                    if (m_useMagnetometer) {
                        DEBUG_PRINT("MPU9250 config mag\n");
                        this->SpiConfig_out(0, MPU9250_SPI_CONFIG_HZ);
                        this->SpiReadWrite_out(0, writeBufObj, dummyReadBufObj);
                    }
                    m_initState = INIT_INT_CONFIG;
                    break;
                case INIT_INT_CONFIG:
                    if (m_opMode == OPMODE_INTERRUPT) {
                        this->SpiConfig_out(0, MPU9250_SPI_CONFIG_HZ);
                        writeBuf[0] = MPU9250_REG_INT_ENABLE | SPI_BITS_WRITE;
                        writeBuf[1] = MPU9250_BITS_INT_EN_RAW_DATA_RDY;
                        this->SpiReadWrite_out(0, writeBufObj, dummyReadBufObj);
                    }
                    m_initState = INIT_FIFO_RESET;
                    break;
                case INIT_FIFO_RESET:
                    DEBUG_PRINT("MPU9250 enter INIT_FIFO_RESET\n");
                    if (m_opMode == OPMODE_FIFO) {
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
                    }
                    if (m_opMode == OPMODE_INTERRUPT) {
                        this->SpiConfig_out(0, MPU9250_SPI_DATA_HZ);
                    }
                    m_initState = INIT_COMPLETE;
                    this->log_ACTIVITY_HI_MPU9250_ImuInit();
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

} // end namespace Drv
