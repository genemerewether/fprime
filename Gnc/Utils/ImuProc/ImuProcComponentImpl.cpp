// ======================================================================
// \title  ImuProcImpl.cpp
// \author mereweth
// \brief  cpp file for ImuProc component implementation class
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


#include <Gnc/Utils/ImuProc/ImuProcComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

#ifdef BUILD_DSPAL
#include <HAP_farf.h>
#define DEBUG_PRINT(x,...) FARF(ALWAYS,x,##__VA_ARGS__);
#else // BUILD_DSPAL
#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#endif // BUILD_DSPAL

#undef DEBUG_PRINT
#define DEBUG_PRINT(x,...)

namespace Gnc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  ImuProcComponentImpl ::
#if FW_OBJECT_NAMES == 1
    ImuProcComponentImpl(
        const char *const compName
    ) :
      ImuProcComponentBase(compName),
#else
      ImuProcImpl(void),
#endif
      m_batchImu()
  {
      m_batchImu.setsamples_count(0);
      (void) m_batchImu.getsamples(batchImuMax);
  }

  void ImuProcComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    ImuProcComponentBase::init(instance);
  }

  ImuProcComponentImpl ::
    ~ImuProcComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void ImuProcComponentImpl ::
    prmTrigger_handler(
        const NATIVE_INT_TYPE portNum,
        FwPrmIdType dummy
    )
  {
      this->loadParameters();
  }

  void ImuProcComponentImpl ::
    HighRateImu_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::sensor_msgs::ImuNoCov &ImuNoCov
    )
  {
      // TODO (mereweth) - actually do processing and downsample

      for (int i = 0; i < NUM_DOWNSAMPLEDIMU_OUTPUT_PORTS; i++) {
          if (this->isConnected_DownsampledImu_OutputPort(i)) {
              this->DownsampledImu_out(i, ImuNoCov);
          }
          else {
              DEBUG_PRINT("ImuProc DownsampledImu out port %d not connected\n", i);
          }
      }

      U32 samplesCount = m_batchImu.getsamples_count();
      if ((samplesCount >= batchImuMax) ||
          (samplesCount >= FW_NUM_ARRAY_ELEMENTS(m_imuArray))) { // TODO(mereweth) - evr if greater?
          // NOTE(mereweth) - batchImu is full; send and reset
          NATIVE_INT_TYPE size = samplesCount;
          this->m_batchImu.setsamples(m_imuArray, size);
          for (int i = 0; i < NUM_BATCHIMU_OUTPUT_PORTS; i++) {
              if (this->isConnected_BatchImu_OutputPort(i)) {
                  this->BatchImu_out(i, this->m_batchImu);
              }
              else {
                  DEBUG_PRINT("ImuProc BatchImu out port %d not connected\n", i);
              }
          }

          samplesCount = 0;
      }

      m_imuArray[samplesCount++] = ImuNoCov; // post-increment
      m_batchImu.setsamples_count(samplesCount);
  }

  // ----------------------------------------------------------------------
  // Command handler implementations
  // ----------------------------------------------------------------------

  void ImuProcComponentImpl ::
    IMUPROC_InitParams_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq
    )
  {
    // TODO
    this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
  }

} // end namespace Gnc
