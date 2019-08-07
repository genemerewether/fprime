// ======================================================================
// \title  FlatOutputAdapterComponentImpl.hpp
// \author decoy
// \brief  hpp file for FlatOutputAdapter component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================

#ifndef FlatOutputAdapter_HPP
#define FlatOutputAdapter_HPP

#include "Gnc/Utils/FlatOutputAdapter/FlatOutputAdapterComponentAc.hpp"
#include <Eigen/Geometry>

namespace Gnc {

  class FlatOutputAdapterComponentImpl :
    public FlatOutputAdapterComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object FlatOutputAdapter
      //!
      FlatOutputAdapterComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object FlatOutputAdapter
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object FlatOutputAdapter
      //!
      ~FlatOutputAdapterComponentImpl(void);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for se3Cmd
      //!
      void se3Cmd_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::mav_msgs::Se3FeedForward &Se3FeedForward 
      );


    };

} // end namespace Gnc

#endif
