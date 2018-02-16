// ======================================================================
// \title  RotorSDrvImpl.cpp
// \author mereweth
// \brief  cpp file for RotorSDrv component implementation class
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


#include <SIMREF/RotorSDrv/RotorSDrvComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace SIMREF {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  RotorSDrvComponentImpl ::
#if FW_OBJECT_NAMES == 1
    RotorSDrvComponentImpl(
        const char *const compName
    ) :
      RotorSDrvComponentBase(compName)
#else
    RotorSDrvImpl(void)
#endif
  {

  }

  void RotorSDrvComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    RotorSDrvComponentBase::init(instance);
  }

  RotorSDrvComponentImpl ::
    ~RotorSDrvComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void RotorSDrvComponentImpl ::
    sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    // TODO
  }

  void RotorSDrvComponentImpl ::
    pingIn_handler(
        const NATIVE_INT_TYPE portNum,
        U32 key
    )
  {
    // TODO
  }

  // ----------------------------------------------------------------------
  // Member function definitions
  // ----------------------------------------------------------------------

  //! Entry point for task waiting for messages
  void RotorSDrvComponentImpl ::
    intTaskEntry(void * ptr) {

      FW_ASSERT(ptr);
      RotorSDrvComponentImpl* compPtr = (RotorSDrvComponentImpl*) ptr;

      ros::NodeHandle n;
      ros::Subscriber sub = n.subscribe("clock", 1000,
                                        &RotorSDrvComponentImpl::odometryCallback,
                                        compPtr,
                                        ros::TransportHints().tcpNoDelay());

      ros::spin();
  }

  void RotorSDrvComponentImpl ::
    odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
      /*if (this->m_odometryCallbacks%100 == 0)
      {
          DEBUG_PRINT("Odometry callback %d, %f\n", this->m_callbacks,
                      msg->header.stamp.toSec());
      }
      this->m_callbacks++;*/

  }

} // end namespace SIMREF
