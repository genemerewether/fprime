// ======================================================================
// \title  LeeCtrlImpl.hpp
// \author mereweth
// \brief  hpp file for LeeCtrl component implementation class
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

#ifndef LeeCtrl_HPP
#define LeeCtrl_HPP

#include "Gnc/Ctrl/LeeCtrl/LeeCtrlComponentAc.hpp"
#include <Gnc/Ctrl/LeeCtrl/LeeCtrlComponentImplCfg.hpp>

#include "quest_gnc/ctrl/lee_control.h"
#include "quest_gnc/utils/multirotor_model.h"
#include "quest_gnc/utils/world_params.h"

namespace Gnc {

  class LeeCtrlComponentImpl :
    public LeeCtrlComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object LeeCtrl
      //!
      LeeCtrlComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object LeeCtrl
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object LeeCtrl
      //!
      ~LeeCtrlComponentImpl(void);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for odometry
      //!
      void odometry_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::nav_msgs::Odometry &Odometry
      );

      //! Handler implementation for sched
      //!
      void sched_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      U32 seq;

      F32 u_tlm[4];

      F32 thrust_x_tlm;
      F32 thrust_y_tlm;
      
      double mass;

      ROS::geometry_msgs::Point x_w;

      ROS::geometry_msgs::Quaternion w_q_b;

      ROS::geometry_msgs::Vector3 v_b;

      ROS::geometry_msgs::Vector3 omega_b;

      ROS::geometry_msgs::Vector3 a_w__comm;

      quest_gnc::multirotor::LeeControl leeControl;

    };

} // end namespace Gnc

#endif
