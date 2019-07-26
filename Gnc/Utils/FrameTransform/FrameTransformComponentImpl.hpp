// ======================================================================
// \title  FrameTransformImpl.hpp
// \author gene
// \brief  hpp file for FrameTransform component implementation class
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

#ifndef FrameTransform_HPP
#define FrameTransform_HPP

#include "Gnc/Utils/FrameTransform/FrameTransformComponentAc.hpp"

#include <Eigen/Geometry>

namespace Gnc {

  class FrameTransformComponentImpl :
    public FrameTransformComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object FrameTransform
      //!
      FrameTransformComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object FrameTransform
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object FrameTransform
      //!
      ~FrameTransformComponentImpl(void);

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PRIVATE:
      void parameterUpdated(FwPrmIdType id /*!< The parameter ID*/);

      void parametersLoaded();

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for prmTrigger
      //!
      void prmTrigger_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          FwPrmIdType dummy 
      );

      //! Handler implementation for odomInA
      //!
      void odomInA_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::nav_msgs::OdometryAccel &Odometry
      );

      //! Handler implementation for odomInB
      //!
      void odomInB_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::nav_msgs::OdometryAccel &Odometry
      );

    PRIVATE:

      // ----------------------------------------------------------------------
      // Command handler implementations
      // ----------------------------------------------------------------------

      //! Implementation for FTFO_InitParams command handler
      //!
      void FTFO_InitParams_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq /*!< The command sequence number*/
      );

      bool paramsInited;

      Eigen::Affine3d a_X_b; // takes point in b frame to a frame
      Eigen::Affine3d b_X_a; // takes point in a frame to b frame

      bool doRotX; // see params.xml
      bool doRotV; // see params.xml
    
    };

} // end namespace Gnc

#endif
