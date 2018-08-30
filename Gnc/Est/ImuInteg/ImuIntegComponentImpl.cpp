// ======================================================================
// \title  ImuIntegImpl.cpp
// \author mereweth
// \brief  cpp file for ImuInteg component implementation class
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


#include <Gnc/Est/ImuInteg/ImuIntegComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace Gnc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  ImuIntegComponentImpl ::
#if FW_OBJECT_NAMES == 1
    ImuIntegComponentImpl(
        const char *const compName
    ) :
      ImuIntegComponentBase(compName),
#else
    ImuIntegComponentImpl(void) :
      ImuIntegComponentImpl(void),
#endif
      imuInteg()
  {
      quest_gnc::WorldParams wParams = {9.80665f, 1.2f};
      (void) imuInteg.SetWorldParams(wParams);

  }

  void ImuIntegComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    ImuIntegComponentBase::init(instance);
  }

  ImuIntegComponentImpl ::
    ~ImuIntegComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void ImuIntegComponentImpl ::
    Imu_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::sensor_msgs::ImuNoCov &ImuNoCov
    )
  {
      // fill imuInteg IMU setter
  }

  void ImuIntegComponentImpl ::
    sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
      if (context == IMUINTEG_SCHED_CONTEXT_POS) {
      }
      else if (context == IMUINTEG_SCHED_CONTEXT_ATT) {

      }
      else if (context == IMUINTEG_SCHED_CONTEXT_TLM) {

      }
      else {
          // TODO(mereweth) - assert invalid context
      }
  }

} // end namespace Gnc
